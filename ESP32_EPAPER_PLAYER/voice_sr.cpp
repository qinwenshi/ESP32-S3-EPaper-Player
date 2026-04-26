// voice_sr.cpp — Wake word + speech command recognition using ESP-SR
// Wake word : "Hi,喵喵"  (wn9_himiaomiao_tts)
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

// ── Linear resampler: arbitrary mono → 16 kHz mono ───────────────────────────
// Maps in_cnt input samples at in_rate Hz to output at VOICE_SR_HZ.
// Returns number of output samples written (≤ out_max).
static int resample_to_16k(const int16_t *in, int in_cnt, uint32_t in_rate,
                            int16_t *out, int out_max)
{
    int n = 0;
    uint64_t pos = 0;  // = n * in_rate; in-stream position for output sample n is pos/VOICE_SR_HZ
    while (n < out_max) {
        uint32_t i0 = (uint32_t)(pos / VOICE_SR_HZ);
        if ((int)(i0 + 1) >= in_cnt) break;
        int f = (int)((pos % VOICE_SR_HZ) * 256 / VOICE_SR_HZ);  // 0–255 fractional
        out[n] = (int16_t)(((256 - f) * (int)in[i0] + f * (int)in[i0 + 1]) >> 8);
        ++n;
        pos += in_rate;
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
static void voice_task(void *arg)
{
    // Brief delay so I2S0 clocks are stable before we enable slave reads
    vTaskDelay(pdMS_TO_TICKS(500));

    int afe_chunk = s_afe->get_feed_chunksize(s_afe_data);  // samples @ 16 kHz per feed
    int afe_ch    = s_afe->get_feed_channel_num(s_afe_data); // 1 (mic only)

    // Worst-case I2S frames per AFE chunk (at 48 kHz)
    int max_i2s_frames = afe_chunk * 48000 / VOICE_SR_HZ + 4;

    // Allocate working buffers
    int16_t *stereo = (int16_t *)heap_caps_malloc(max_i2s_frames * 4, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    int16_t *mono   = (int16_t *)heap_caps_malloc(max_i2s_frames * 2, MALLOC_CAP_8BIT);
    int16_t *feed   = (int16_t *)heap_caps_malloc(afe_chunk * afe_ch * 2, MALLOC_CAP_8BIT);

    if (!stereo || !mono || !feed) {
        Serial.println("[VOICE] OOM allocating buffers — task aborted");
        free(stereo); free(mono); free(feed);
        vTaskDelete(nullptr);
        return;
    }

    Serial.printf("[VOICE] Task started — AFE chunk=%d samples, channels=%d\n", afe_chunk, afe_ch);

    bool mn_active = false;

    for (;;) {
        uint32_t in_rate = g_in_rate;
        int frames_needed = afe_chunk * (int)in_rate / VOICE_SR_HZ + 2;
        if (frames_needed > max_i2s_frames) frames_needed = max_i2s_frames;

        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_i2s1_rx, stereo,
                                          frames_needed * 4, &bytes_read,
                                          pdMS_TO_TICKS(200));
        if (ret != ESP_OK || bytes_read < 4) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int frames_read = (int)(bytes_read / 4);

        // Stereo → mono (average L+R)
        for (int i = 0; i < frames_read; i++) {
            mono[i] = (int16_t)(((int)stereo[i * 2] + (int)stereo[i * 2 + 1]) / 2);
        }

        // Resample mono to 16 kHz
        int out_cnt = resample_to_16k(mono, frames_read, in_rate, feed, afe_chunk);
        if (out_cnt < afe_chunk) {
            memset(feed + out_cnt, 0, (afe_chunk - out_cnt) * sizeof(int16_t));
        }

        // Feed AFE
        s_afe->feed(s_afe_data, feed);

        // Fetch processed result (may block briefly)
        afe_fetch_result_t *result = s_afe->fetch(s_afe_data);
        if (!result) continue;

        if (!mn_active) {
            if (result->wakeup_state == WAKENET_DETECTED) {
                Serial.println("[VOICE] Wake word detected: Hi,喵喵!");
                s_multinet->clean(s_mn_data);
                mn_active = true;
            }
        } else {
            esp_mn_state_t mn_state = s_multinet->detect(s_mn_data, result->data);
            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *res = s_multinet->get_results(s_mn_data);
                if (res && res->num > 0) {
                    int cmd = res->command_id[0];
                    Serial.printf("[VOICE] Command %d detected\n", cmd);
                    g_voice_cmd = cmd;
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
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "himiaomiao");
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    if (!wn_name) {
        Serial.println("[VOICE] ERROR: wn9_himiaomiao_tts not found in models");
        return;
    }
    if (!mn_name) {
        Serial.println("[VOICE] ERROR: mn5q8_cn not found in models");
        return;
    }
    Serial.printf("[VOICE] WakeNet: %s\n", wn_name);
    Serial.printf("[VOICE] MultiNet: %s\n", mn_name);

    // 3. Configure AFE (single mic, no AEC, 16 kHz)
    afe_config_t *afe_cfg = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!afe_cfg) {
        Serial.println("[VOICE] ERROR: afe_config_init failed");
        return;
    }
    afe_cfg->wakenet_init       = true;
    afe_cfg->wakenet_model_name = wn_name;
    afe_cfg->memory_alloc_mode  = AFE_MEMORY_ALLOC_MORE_PSRAM;

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

    // 7. Launch voice task on Core 0
    xTaskCreatePinnedToCore(voice_task, "voice_task",
                            8192, nullptr,
                            3,    // priority (between cover_decode:2 and lvgl:5)
                            nullptr, 0);

    Serial.println("[VOICE] Voice recognition started — say 'Hi,喵喵' to activate");
}
