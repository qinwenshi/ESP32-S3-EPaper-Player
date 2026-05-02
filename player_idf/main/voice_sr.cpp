// voice_sr.cpp — Wake word + speech command recognition using ESP-SR
// Wake word : "Hi ESP"  (wn9_hiesp — official Espressif model)
// Commands  : English phrases (mn5q8_en)
// Mic path  : ES8311 ADC → I2S1 slave (32-bit Philips, BCLK/WS from I2S0 master)
//             I2S1 configured with slot_bit_width=32, data_bit_width=16.
//             Audio in high 16 bits of 32-bit L slot → (int16_t)(raw32 >> 16)
//             Software-resample audio-rate → 16 kHz → WakeNet / MultiNet (no AFE)
//
// IMPORTANT: voice partition "model" must be flashed at 0x400000 with srmodels.bin.
// For Hi ESP: use the official Arduino esp32s3 srmodels.bin (contains wn9_hiesp + mn5q8_en).

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <math.h>
static const char* TAG_VSR = "VOICE";

extern "C" {
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "esp_check.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"
}

#include "voice_sr.h"

#define VOICE_SR_HZ   16000   // WakeNet / MultiNet require 16 kHz input

// ── Module state ─────────────────────────────────────────────────────────────
static volatile int              g_voice_cmd    = VOICE_CMD_NONE;
static volatile uint32_t         g_in_rate      = 44100;
static volatile bool             s_listen_req   = false;
static volatile bool             s_mn_active    = false;

static i2s_chan_handle_t         s_mic_rx   = nullptr;  // I2S1 slave RX (mic capture)
static const esp_wn_iface_t     *s_wakenet  = nullptr;
static model_iface_data_t       *s_wn_data  = nullptr;
static const esp_mn_iface_t     *s_multinet = nullptr;
static model_iface_data_t       *s_mn_data  = nullptr;
static StreamBufferHandle_t      s_audio_buf = nullptr;
static int                       s_wn_chunk  = 0;
static int                       s_mn_chunk  = 0;

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

// ── I2S1 slave mic init ──────────────────────────────────────────────────────
// I2S1 is a SLAVE: receives BCLK/WS from I2S0 master (same pins).
// slot_bit_width=32 to match I2S0 master's 32-bit Philips format.
// ES8311 ADC output (16-bit) lands in bits[31:16] of each 32-bit slot.
#define MIC_BCLK_PIN   GPIO_NUM_15
#define MIC_WS_PIN     GPIO_NUM_38
#define MIC_DIN_PIN    GPIO_NUM_16

static esp_err_t mic_i2s_init(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num  = 4;
    chan_cfg.dma_frame_num = 256;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, nullptr, &s_mic_rx),
                        TAG_VSR, "I2S1 new_channel failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_BCLK_PIN,
            .ws   = MIC_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = MIC_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    // Match I2S0 master's 32-bit slot width so the BCLK frames align.
    // ES8311 ADC sends 16-bit audio left-justified: audio lands in bits[31:16].
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_mic_rx, &std_cfg),
                        TAG_VSR, "I2S1 init_std_mode failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_mic_rx),
                        TAG_VSR, "I2S1 enable failed");
    return ESP_OK;
}

// ── feed_task: Core 0 — reads I2S1 RX (slave), resamples → StreamBuffer ──────
static void feed_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));  // wait for I2S0 clocks to stabilise

    // max input frames to read per iteration (at highest possible rate, 48 kHz)
    int max_i2s_frames = s_wn_chunk * 48000 / VOICE_SR_HZ + 8;

    // 32-bit stereo: 8 bytes per frame
    int32_t *stereo    = (int32_t *)heap_caps_malloc(max_i2s_frames * 8,  MALLOC_CAP_SPIRAM);
    int16_t *mono      = (int16_t *)heap_caps_malloc(max_i2s_frames * 2,  MALLOC_CAP_SPIRAM);
    int16_t *resampled = (int16_t *)heap_caps_malloc(s_wn_chunk * 2,      MALLOC_CAP_SPIRAM);

    if (!stereo || !mono || !resampled) {
        ESP_LOGI(TAG_VSR, "OOM in feed_task — aborted");
        heap_caps_free(stereo); heap_caps_free(mono); heap_caps_free(resampled);
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG_VSR, "feed_task started — wn_chunk=%d", s_wn_chunk);

    uint32_t last_rms_log = 0;

    for (;;) {
        uint32_t in_rate = g_in_rate;
        int frames_needed = s_wn_chunk * (int)in_rate / VOICE_SR_HZ + 4;
        if (frames_needed > max_i2s_frames) frames_needed = max_i2s_frames;

        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_mic_rx, stereo,
                                          frames_needed * 8, &bytes_read,
                                          pdMS_TO_TICKS(200));
        if (ret != ESP_OK || bytes_read < 16) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // 32-bit stereo frame = 8 bytes → L (4 bytes) + R (4 bytes)
        // ES8311 ADC: audio in high 16 bits of L slot (Philips format)
        int actual_samples = (int)(bytes_read / 8);

        // Extract L channel with 4× software gain
        for (int i = 0; i < actual_samples; i++) {
            int v = (int)(int16_t)(stereo[i * 2] >> 16) * 4;
            mono[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
        }

        // Resample actual_samples @ in_rate → s_wn_chunk @ 16 kHz
        int out_cnt = resample_to_16k(mono, actual_samples, in_rate, resampled, s_wn_chunk);
        if (out_cnt < s_wn_chunk)
            memset(resampled + out_cnt, 0, (s_wn_chunk - out_cnt) * sizeof(int16_t));

        // Periodic mic-level diagnostic (every 5 s)
        {
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now_ms - last_rms_log > 5000) {
                last_rms_log = now_ms;
                int64_t s2 = 0, f2 = 0;
                for (int i = 0; i < actual_samples; i++) s2 += (int64_t)mono[i] * mono[i];
                for (int i = 0; i < s_wn_chunk;     i++) f2 += (int64_t)resampled[i] * resampled[i];
                ESP_LOGI(TAG_VSR, "MicRMS=%d FeedRMS=%d rate=%u frames=%d",
                    (int)sqrtf((float)(s2 / actual_samples)),
                    (int)sqrtf((float)(f2 / s_wn_chunk)),
                    in_rate, actual_samples);
                if (actual_samples >= 4) {
                    ESP_LOGI(TAG_VSR, "RAW32[0..3]: %d %d %d %d",
                        (int)(int16_t)(stereo[0] >> 16), (int)(int16_t)(stereo[2] >> 16),
                        (int)(int16_t)(stereo[4] >> 16), (int)(int16_t)(stereo[6] >> 16));
                }
            }
        }

        xStreamBufferSend(s_audio_buf, resampled,
                          s_wn_chunk * sizeof(int16_t), pdMS_TO_TICKS(50));
        taskYIELD();
    }
}


// ── detect_task: Core 1 — WakeNet + MultiNet on raw resampled audio ──────────
static void detect_task(void *arg)
{
    ESP_LOGI(TAG_VSR, "detect_task started");

    int16_t *wn_buf = (int16_t *)heap_caps_malloc(s_wn_chunk * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    int16_t *mn_buf = (int16_t *)heap_caps_malloc(s_mn_chunk * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!wn_buf || !mn_buf) {
        ESP_LOGI(TAG_VSR, "OOM in detect_task");
        heap_caps_free(wn_buf); heap_caps_free(mn_buf);
        vTaskDelete(nullptr);
        return;
    }

    bool mn_active = false;
    int  mn_pos    = 0;
    uint32_t last_diag = 0;

    for (;;) {
        size_t got = xStreamBufferReceive(s_audio_buf, wn_buf,
                                          s_wn_chunk * sizeof(int16_t),
                                          pdMS_TO_TICKS(500));
        if (got < (size_t)(s_wn_chunk * sizeof(int16_t))) continue;

        // Periodic diagnostic every 2 s
        {
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now - last_diag > 2000) {
                last_diag = now;
                int64_t s2 = 0;
                for (int i = 0; i < s_wn_chunk; i++) s2 += (int64_t)wn_buf[i] * wn_buf[i];
                ESP_LOGI(TAG_VSR, "RawRMS=%d mn_active=%d",
                    (int)sqrtf((float)(s2 / s_wn_chunk)), mn_active ? 1 : 0);
            }
        }

        if (!mn_active) {
            bool manual = s_listen_req;
            if (manual) s_listen_req = false;

            wakenet_state_t wn_state = s_wakenet->detect(s_wn_data, wn_buf);

            if (wn_state == WAKENET_DETECTED || wn_state == WAKENET_CHANNEL_VERIFIED)
                ESP_LOGI(TAG_VSR, "*** Wake word 'Hi ESP' detected! ***");

            if (manual)
                ESP_LOGI(TAG_VSR, "*** Manual listen activated — speak command ***");

            if (manual || wn_state == WAKENET_DETECTED || wn_state == WAKENET_CHANNEL_VERIFIED) {
                s_multinet->clean(s_mn_data);
                mn_active   = true;
                s_mn_active = true;
                mn_pos      = 0;
                ESP_LOGI(TAG_VSR, "MultiNet window open (chunk=%d)", s_mn_chunk);
            }
        }

        if (mn_active) {
            int space = s_mn_chunk - mn_pos;
            int copy  = (s_wn_chunk < space) ? s_wn_chunk : space;
            memcpy(mn_buf + mn_pos, wn_buf, copy * sizeof(int16_t));
            mn_pos += copy;

            if (mn_pos >= s_mn_chunk) {
                {
                    int64_t s2 = 0;
                    for (int i = 0; i < s_mn_chunk; i++) s2 += (int64_t)mn_buf[i] * mn_buf[i];
                    ESP_LOGI(TAG_VSR, "CMD DataRMS=%d",
                        (int)sqrtf((float)(s2 / s_mn_chunk)));
                }

                esp_mn_state_t mn_state = s_multinet->detect(s_mn_data, mn_buf);
                mn_pos = 0;

                if (mn_state == ESP_MN_STATE_DETECTED) {
                    esp_mn_results_t *res = s_multinet->get_results(s_mn_data);
                    if (res && res->num > 0) {
                        ESP_LOGI(TAG_VSR, "Command %d detected: %s",
                            res->command_id[0], res->string[0]);
                        g_voice_cmd = res->command_id[0];
                    }
                    mn_active   = false;
                    s_mn_active = false;
                } else if (mn_state == ESP_MN_STATE_TIMEOUT) {
                    ESP_LOGI(TAG_VSR, "Command timeout");
                    mn_active   = false;
                    s_mn_active = false;
                }
            }
        }
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

void voice_sr_set_input_rate(uint32_t hz)
{
    if (hz > 0) g_in_rate = hz;
}

void voice_sr_start_listen(void)
{
    s_listen_req = true;
    s_mn_active  = true;   // mark as active immediately so caller can mute before first chunk
}

bool voice_sr_is_listening(void)
{
    return s_mn_active;
}

int voice_sr_get_cmd(void)
{
    int cmd = g_voice_cmd;
    if (cmd != VOICE_CMD_NONE) g_voice_cmd = VOICE_CMD_NONE;
    return cmd;
}

void voice_sr_init(void)
{
    ESP_LOGI(TAG_VSR, "Initializing voice recognition (no-AFE direct WN+MN)...");

    // 1. Load models from the "model" SPIFFS partition
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models || models->num == 0) {
        ESP_LOGI(TAG_VSR, "ERROR: No models found in 'model' partition.");
        ESP_LOGI(TAG_VSR, "Run 'make flash-model' to flash srmodels.bin first.");
        return;
    }
    ESP_LOGI(TAG_VSR, "Models loaded: %d", models->num);
    for (int i = 0; i < models->num; i++)
        ESP_LOGI(TAG_VSR, "  [%d] %s", i, models->model_name[i]);

    // 2. Find WakeNet and MultiNet model names
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "hiesp");
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    if (!wn_name) { ESP_LOGI(TAG_VSR, "ERROR: wn9_hiesp not found in models"); return; }
    if (!mn_name) { ESP_LOGI(TAG_VSR, "ERROR: mn5q8_en not found in models"); return; }
    ESP_LOGI(TAG_VSR, "WakeNet: %s", wn_name);
    ESP_LOGI(TAG_VSR, "MultiNet: %s", mn_name);

    // 3. Create WakeNet directly (no AFE)
    s_wakenet = esp_wn_handle_from_name(wn_name);
    if (!s_wakenet) { ESP_LOGI(TAG_VSR, "ERROR: esp_wn_handle_from_name failed"); return; }
    s_wn_data = s_wakenet->create(wn_name, DET_MODE_90);
    if (!s_wn_data) { ESP_LOGI(TAG_VSR, "ERROR: wakenet->create failed"); return; }
    s_wn_chunk = s_wakenet->get_samp_chunksize(s_wn_data);
    ESP_LOGI(TAG_VSR, "WakeNet chunk=%d samples", s_wn_chunk);

    // 4. Create MultiNet directly
    s_multinet = esp_mn_handle_from_name(mn_name);
    if (!s_multinet) { ESP_LOGI(TAG_VSR, "ERROR: esp_mn_handle_from_name failed"); return; }
    s_mn_data = s_multinet->create(mn_name, 6000);
    if (!s_mn_data) { ESP_LOGI(TAG_VSR, "ERROR: multinet->create failed"); return; }
    s_mn_chunk = s_multinet->get_samp_chunksize(s_mn_data);
    ESP_LOGI(TAG_VSR, "MultiNet chunk=%d samples", s_mn_chunk);

    // 5. Register commands
    esp_mn_commands_alloc(s_multinet, s_mn_data);
    esp_mn_commands_add(VOICE_CMD_NEXT,     "change song");
    esp_mn_commands_add(VOICE_CMD_NEXT,     "skip song");
    esp_mn_commands_add(VOICE_CMD_PREV,     "previous song");
    esp_mn_commands_add(VOICE_CMD_PREV,     "go to previous");
    esp_mn_commands_add(VOICE_CMD_PAUSE,    "pause");
    esp_mn_commands_add(VOICE_CMD_PAUSE,    "stop music");
    esp_mn_commands_add(VOICE_CMD_RESUME,   "start music");
    esp_mn_commands_add(VOICE_CMD_RESUME,   "resume music");
    esp_mn_commands_add(VOICE_CMD_VOL_UP,   "volume up");
    esp_mn_commands_add(VOICE_CMD_VOL_UP,   "turn up");
    esp_mn_commands_add(VOICE_CMD_VOL_DOWN, "volume down");
    esp_mn_commands_add(VOICE_CMD_VOL_DOWN, "turn down");
    esp_mn_error_t *mn_err = esp_mn_commands_update();
    if (mn_err && mn_err->num > 0) {
        ESP_LOGI(TAG_VSR, "WARNING: %d command(s) could not be added:", mn_err->num);
        for (int i = 0; i < mn_err->num; i++)
            ESP_LOGI(TAG_VSR, "  cmd %d: %s",
                          mn_err->phrases[i]->command_id,
                          mn_err->phrases[i]->string);
    }

    // 6. Audio stream buffer: 40 WakeNet chunks in SPIRAM (avoids internal RAM pressure)
    s_audio_buf = xStreamBufferCreateWithCaps(s_wn_chunk * 40 * sizeof(int16_t),
                                              s_wn_chunk * sizeof(int16_t),
                                              MALLOC_CAP_SPIRAM);
    if (!s_audio_buf) {
        // fallback: try internal RAM with smaller buffer
        s_audio_buf = xStreamBufferCreate(s_wn_chunk * 8 * sizeof(int16_t),
                                          s_wn_chunk * sizeof(int16_t));
    }
    if (!s_audio_buf) { ESP_LOGI(TAG_VSR, "ERROR: stream buffer alloc failed"); return; }

    // 7. Init I2S1 slave for mic capture (32-bit slot, BCLK/WS from I2S0 master)
    if (mic_i2s_init(44100) != ESP_OK) {
        ESP_LOGI(TAG_VSR, "ERROR: mic I2S1 init failed");
        return;
    }

    // 8. Launch tasks (stacks explicitly in internal RAM)
    xTaskCreatePinnedToCoreWithCaps(feed_task,   "voice_feed",   4096, nullptr, 5, nullptr, 0,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskCreatePinnedToCoreWithCaps(detect_task, "voice_detect", 8192, nullptr, 5, nullptr, 1,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    ESP_LOGI(TAG_VSR, "Started — WakeNet chunk=%d MultiNet chunk=%d — say 'Hi ESP'",
                  s_wn_chunk, s_mn_chunk);
}
