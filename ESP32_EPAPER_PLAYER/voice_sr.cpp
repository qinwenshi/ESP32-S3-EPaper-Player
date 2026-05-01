// voice_sr.cpp — Wake word + speech command recognition using ESP-SR
// Wake word : "你好小智"  (wn9_nihaoxiaozhi — real human-speech trained)
// Commands  : Chinese phrases (mn5q8_cn)
// Mic path  : ES8311 ADC → I2S1 slave (DIN=GPIO16, BCLK=GPIO15, WS=GPIO38)
//             software-resample audio-rate → 16 kHz for AFE
//
// IMPORTANT: voice partition "model" must be flashed at 0x400000 with srmodels.bin.

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

extern "C" {
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_afe_config.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_wn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"
}

#include "voice_sr.h"

// ── Hardware pins ────────────────────────────────────────────────────────────
#define MIC_BCLK_PIN  15
#define MIC_WS_PIN    38
#define MIC_DIN_PIN   16

#define VOICE_SR_HZ   16000   // AFE requires 16 kHz input

// ── Module state ─────────────────────────────────────────────────────────────
static volatile int      g_voice_cmd = VOICE_CMD_NONE;
static volatile uint32_t g_in_rate   = 44100;   // updated by voice_sr_set_input_rate()

static i2s_chan_handle_t         s_i2s1_rx   = nullptr;
static esp_afe_sr_data_t        *s_afe_data  = nullptr;
static const esp_afe_sr_iface_t *s_afe       = nullptr;
static const esp_mn_iface_t     *s_multinet  = nullptr;
static model_iface_data_t       *s_mn_data   = nullptr;

// ── Two-stage resampler with anti-aliasing: in_rate → 16 kHz ─────────────────
// Stage 1: decimate by 2 with 2-tap averaging (perfect LPF at in_rate/4 Hz,
//          e.g. 11025 Hz for 44100 input — exactly the Nyquist of half-rate)
// Stage 2: linear interpolation at ~1.38x ratio (22050→16000) with negligible distortion
static int resample_to_16k(const int16_t *in, int in_cnt, uint32_t in_rate,
                            int16_t *out, int out_max)
{
    uint32_t half_rate = in_rate / 2;  // 22050 Hz after stage-1 decimation
    int n = 0;
    uint64_t pos = 0;  // tracks n * half_rate (avoids division per output sample)

    while (n < out_max) {
        uint32_t i0 = (uint32_t)(pos / VOICE_SR_HZ);
        if ((int)(i0 * 2 + 3) >= in_cnt) break;

        // Stage-1: average adjacent pair → half-rate sample
        int s0 = ((int)in[i0*2 + 0] + in[i0*2 + 1]) >> 1;
        int s1 = ((int)in[i0*2 + 2] + in[i0*2 + 3]) >> 1;

        // Stage-2: linear interpolation between the two half-rate samples
        int f = (int)((pos % VOICE_SR_HZ) * 256 / VOICE_SR_HZ);
        out[n] = (int16_t)(((256 - f) * s0 + f * s1) >> 8);
        ++n;
        pos += half_rate;
    }
    return n;
}

// ── I2S1 slave init (reads mic from ES8311 via shared I2S0 clocks) ───────────
static esp_err_t mic_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 256;
    esp_err_t ret = i2s_new_channel(&chan_cfg, nullptr, &s_i2s1_rx);
    if (ret != ESP_OK) return ret;

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),   // unused in slave mode
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)MIC_BCLK_PIN,
            .ws   = (gpio_num_t)MIC_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = (gpio_num_t)MIC_DIN_PIN,
        },
    };

    ret = i2s_channel_init_std_mode(s_i2s1_rx, &std_cfg);
    if (ret != ESP_OK) return ret;

    return i2s_channel_enable(s_i2s1_rx);
}

// ── Voice task (runs on Core 0, priority 3) ───────────────────────────────────
// Feed task: Core 0 — reads I2S, resamples, feeds AFE continuously
static void feed_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));  // wait for I2S0 clocks to stabilise

    int afe_chunk      = s_afe->get_feed_chunksize(s_afe_data);
    int feed_ch        = s_afe->get_feed_channel_num(s_afe_data);
    // ROOT-CAUSE FIX: ESP32-audioI2S master uses 32-bit I2S slots.
    // Our 16-bit slave reads 2 frames per actual WS cycle:
    //   odd frames  → valid LEFT audio (MSB-16 of 32-bit LEFT slot)
    //   even frames → zeros (MSB-16 of 32-bit RIGHT slot = 0 for mono ES8311)
    // Fix: read 2× as many frames and take every other one (stereo[i*4] not stereo[i*2]).
    // Effective sample rate is still in_rate; actual audio frames = frames_read / 2.
    int max_i2s_frames = afe_chunk * 48000 / VOICE_SR_HZ * 2 + 8;

    int16_t *stereo    = (int16_t *)heap_caps_malloc(max_i2s_frames * 4,          MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    int16_t *mono      = (int16_t *)heap_caps_malloc((max_i2s_frames / 2 + 1) * 2, MALLOC_CAP_8BIT);
    int16_t *resampled = (int16_t *)heap_caps_malloc(afe_chunk * 2,               MALLOC_CAP_8BIT);
    int16_t *feed      = (int16_t *)heap_caps_malloc(afe_chunk * feed_ch * 2,     MALLOC_CAP_8BIT);

    if (!stereo || !mono || !resampled || !feed) {
        Serial.println("[VOICE] OOM in feed_task — aborted");
        heap_caps_free(stereo); heap_caps_free(mono);
        heap_caps_free(resampled); heap_caps_free(feed);
        vTaskDelete(nullptr);
        return;
    }

    Serial.printf("[VOICE] feed_task started — chunk=%d feed_ch=%d\n", afe_chunk, feed_ch);

    uint32_t last_rms_log = 0;

    for (;;) {
        uint32_t in_rate = g_in_rate;
        // Need 2× frames_needed because only half the I2S frames contain valid audio
        int frames_needed = afe_chunk * (int)in_rate / VOICE_SR_HZ * 2 + 4;
        if (frames_needed > max_i2s_frames) frames_needed = max_i2s_frames;

        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_i2s1_rx, stereo,
                                          frames_needed * 4, &bytes_read,
                                          pdMS_TO_TICKS(200));
        if (ret != ESP_OK || bytes_read < 8) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int frames_read = (int)(bytes_read / 4);
        // Only odd-indexed I2S frames contain actual audio (every 2nd frame is the
        // 32-bit RIGHT slot which is zero for mono ES8311).
        int actual_samples = frames_read / 2;

        // Extract valid audio: take stereo[i*4] (LEFT channel of every 2nd frame)
        for (int i = 0; i < actual_samples; i++)
            mono[i] = stereo[i * 4];

        // Resample actual_samples @ in_rate → afe_chunk @ 16 kHz
        int out_cnt = resample_to_16k(mono, actual_samples, in_rate, resampled, afe_chunk);
        if (out_cnt < afe_chunk)
            memset(resampled + out_cnt, 0, (afe_chunk - out_cnt) * sizeof(int16_t));

        // Fill feed buffer
        for (int i = 0; i < afe_chunk; i++) {
            feed[i * feed_ch] = resampled[i];
            for (int c = 1; c < feed_ch; c++) feed[i * feed_ch + c] = 0;
        }

        // Periodic mic-level diagnostic (every 5 s)
        {
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now_ms - last_rms_log > 5000) {
                last_rms_log = now_ms;
                int64_t s2 = 0, f2 = 0;
                for (int i = 0; i < actual_samples; i++) s2 += (int64_t)mono[i] * mono[i];
                for (int i = 0; i < afe_chunk;      i++) f2 += (int64_t)resampled[i] * resampled[i];
                Serial.printf("[VOICE] MicRMS=%d FeedRMS=%d rate=%u\n",
                    (int)sqrtf((float)(s2 / actual_samples)),
                    (int)sqrtf((float)(f2 / afe_chunk)),
                    in_rate);
            }
        }

        // Feed AFE — yield after feed so other tasks (watchdog, audio) get CPU time
        s_afe->feed(s_afe_data, feed);
        taskYIELD();
    }
}

// Detect task: Core 1 — fetches AFE results, runs WakeNet + MultiNet
static void detect_task(void *arg)
{
    Serial.println("[VOICE] detect_task started");

    bool mn_active = false;
    uint32_t last_diag = 0;

    for (;;) {
        afe_fetch_result_t *result = s_afe->fetch(s_afe_data);
        if (!result || result->ret_value == ESP_FAIL) {
            static uint32_t last_fail = 0;
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now - last_fail > 2000) { last_fail = now;
                Serial.printf("[VOICE] fetch failed ret=%d result=%p\n",
                    result ? result->ret_value : -99, result);
            }
            continue;
        }

        // Periodic diagnostics: dB volume + ring buffer fill (every 2 s)
        {
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now - last_diag > 2000) {
                last_diag = now;
                Serial.printf("[VOICE] vol=%.1fdB rbuf_free=%.2f vad=%d wakeup=%d\n",
                    result->data_volume,
                    result->ringbuff_free_pct,
                    result->vad_state,
                    result->wakeup_state);
            }
        }

        if (!mn_active) {
            if (result->wakeup_state != WAKENET_NO_DETECT)
                Serial.printf("[VOICE] wakeup_state=%d\n", result->wakeup_state);

            if (result->wakeup_state == WAKENET_DETECTED ||
                result->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
                Serial.println("[VOICE] *** Wake word detected: 你好小智! ***");
                s_multinet->clean(s_mn_data);
                mn_active = true;
            }
        } else {
            esp_mn_state_t mn_state = s_multinet->detect(s_mn_data, result->data);
            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *res = s_multinet->get_results(s_mn_data);
                if (res && res->num > 0) {
                    Serial.printf("[VOICE] Command %d detected\n", res->command_id[0]);
                    g_voice_cmd = res->command_id[0];
                }
                mn_active = false;
            } else if (mn_state == ESP_MN_STATE_TIMEOUT) {
                Serial.println("[VOICE] Command timeout");
                mn_active = false;
            }
        }
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void voice_sr_set_input_rate(uint32_t hz)
{
    if (hz > 0) g_in_rate = hz;
}

int voice_sr_get_cmd(void)
{
    int cmd = g_voice_cmd;
    if (cmd != VOICE_CMD_NONE) g_voice_cmd = VOICE_CMD_NONE;
    return cmd;
}

void voice_sr_init(void)
{
    Serial.println("[VOICE] Initializing voice recognition...");

    // 1. Load models from the "model" SPIFFS partition (srmodels.bin flashed at 0x400000)
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models || models->num == 0) {
        Serial.println("[VOICE] ERROR: No models found in 'model' partition.");
        Serial.println("[VOICE] Run 'make flash-model' to flash srmodels.bin first.");
        return;
    }
    Serial.printf("[VOICE] Models loaded: %d\n", models->num);
    for (int i = 0; i < models->num; i++) {
        Serial.printf("[VOICE]   [%d] %s\n", i, models->model_name[i]);
    }

    // 2. Find WakeNet and MultiNet model names
    // 2. Find WakeNet and MultiNet model names
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "nihaoxiaozhi");
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    if (!wn_name) {
        Serial.println("[VOICE] ERROR: wn9_nihaoxiaozhi not found in models");
        return;
    }
    if (!mn_name) {
        Serial.println("[VOICE] ERROR: mn5q8_cn not found in models");
        return;
    }
    Serial.printf("[VOICE] WakeNet: %s\n", wn_name);
    Serial.printf("[VOICE] MultiNet: %s\n", mn_name);

    // 3. Configure AFE (single mic "M", 16 kHz, LOW_COST)
    // Note: afe_config_init with AFE_TYPE_SR auto-sets wakenet_init=true.
    // We explicitly set wakenet_model_name to ensure the correct model is used.
    afe_config_t *afe_cfg = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!afe_cfg) {
        Serial.println("[VOICE] ERROR: afe_config_init failed");
        return;
    }
    afe_cfg->wakenet_model_name = wn_name;       // explicitly use wn9_nihaoxiaozhi
    afe_cfg->memory_alloc_mode  = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_cfg->ns_init            = false;          // keep NS disabled: causes OOM/reboot with LOW_COST mode
    afe_cfg->afe_perferred_core     = 1;          // AFE internal tasks on Core 1
    afe_cfg->afe_perferred_priority = 1;          // low priority — don't starve audio/UI
    Serial.printf("[VOICE] AFE PCM: rate=%d ch=%d mic=%d ref=%d wn=%s\n",
                  afe_cfg->pcm_config.sample_rate,
                  afe_cfg->pcm_config.total_ch_num,
                  afe_cfg->pcm_config.mic_num,
                  afe_cfg->pcm_config.ref_num,
                  afe_cfg->wakenet_model_name ? afe_cfg->wakenet_model_name : "(auto)");

    // 4. Create AFE instance
    s_afe = esp_afe_handle_from_config(afe_cfg);
    if (!s_afe) {
        Serial.println("[VOICE] ERROR: esp_afe_handle_from_config failed");
        afe_config_free(afe_cfg);
        return;
    }
    s_afe_data = s_afe->create_from_config(afe_cfg);
    afe_config_free(afe_cfg);
    if (!s_afe_data) {
        Serial.println("[VOICE] ERROR: AFE create_from_config failed");
        return;
    }

    // Explicitly enable WakeNet detection
    s_afe->enable_wakenet(s_afe_data);

    // Note: set_wakenet_threshold returns -1 for TTS models (fixed threshold 0.636)
    // We rely on the model's built-in threshold

    // 5. Create MultiNet instance and register Chinese commands
    s_multinet = esp_mn_handle_from_name(mn_name);
    if (!s_multinet) {
        Serial.println("[VOICE] ERROR: esp_mn_handle_from_name failed");
        return;
    }
    s_mn_data = s_multinet->create(mn_name, 6000);  // 6 s detection window
    if (!s_mn_data) {
        Serial.println("[VOICE] ERROR: multinet->create failed");
        return;
    }

    esp_mn_commands_alloc(s_multinet, s_mn_data);
    esp_mn_commands_add(VOICE_CMD_NEXT,     "xia yi shou");
    esp_mn_commands_add(VOICE_CMD_PREV,     "shang yi shou");
    esp_mn_commands_add(VOICE_CMD_PAUSE,    "zan ting");
    esp_mn_commands_add(VOICE_CMD_RESUME,   "ji xu bo fang");
    esp_mn_commands_add(VOICE_CMD_VOL_UP,   "sheng yin da yi dian");
    esp_mn_commands_add(VOICE_CMD_VOL_DOWN, "sheng yin xiao yi dian");
    esp_mn_error_t *mn_err = esp_mn_commands_update();
    if (mn_err && mn_err->num > 0) {
        Serial.printf("[VOICE] WARNING: %d command(s) could not be added:\n", mn_err->num);
        for (int i = 0; i < mn_err->num; i++) {
            Serial.printf("[VOICE]   cmd %d: %s\n",
                          mn_err->phrases[i]->command_id,
                          mn_err->phrases[i]->string);
        }
    }

    // 6. Init I2S1 in slave mode for mic input
    esp_err_t i2s_ret = mic_i2s_init();
    if (i2s_ret != ESP_OK) {
        Serial.printf("[VOICE] ERROR: I2S1 init failed: 0x%x\n", i2s_ret);
        return;
    }

    // 7. Launch feed task (Core 0) + detect task (Core 1) — matches official esp32-hal-sr pattern
    xTaskCreatePinnedToCore(feed_task,   "voice_feed",   4096, nullptr, 5, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskCreatePinnedToCore(detect_task, "voice_detect", 8192, nullptr, 5, nullptr, 1);

    Serial.println("[VOICE] Voice recognition started — say '你好小智' to activate");
}
