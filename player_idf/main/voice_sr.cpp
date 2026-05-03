// voice_sr.cpp — Wake word + speech command recognition using ESP-SR AFE
// Wake word : "Hi ESP"  (wn9_hiesp — official Espressif model)
// Commands  : English phrases (mn5q8_en)
//
// Architecture (M = 1 mic, NO AEC, HIGH_PERF mode):
//   I2S1 slave mic (32-bit slot, BCLK/WS from I2S0 master)
//   → feed_task: mic downsampled to 16kHz, fed directly to AFE (single channel)
//   → AFE (WakeNet + AGC, no AEC): detect "Hi ESP"
//   → fetch_task: AFE output fed to MultiNet for command recognition
//
// AEC is disabled because I2S0 DMA latency (~185ms at 22050 Hz) exceeds filter
// coverage (4 × 32ms = 128ms), causing AEC to treat voice as echo and cancel it.
// Single-mic M mode gives clean signal to WakeNet without AEC over-suppression.
//
// IMPORTANT: voice partition "model" must be flashed at 0x400000 with srmodels.bin.

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
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_afe_config.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"
}

#include "voice_sr.h"
#include "audio.h"

#define VOICE_SR_HZ   16000   // WakeNet / MultiNet require 16 kHz input

// ── Module state ─────────────────────────────────────────────────────────────
static volatile int              g_voice_cmd    = VOICE_CMD_NONE;
static volatile uint32_t         g_in_rate      = 44100;
static volatile bool             s_listen_req   = false;
static volatile bool             s_mn_active    = false;

static i2s_chan_handle_t            s_mic_rx    = nullptr;  // I2S1 slave RX (mic)
static const esp_afe_sr_iface_t    *s_afe       = nullptr;  // AFE interface (AEC+WakeNet)
static esp_afe_sr_data_t           *s_afe_data  = nullptr;
static int                          s_afe_feed_chunk  = 0;  // samples/channel per AFE feed
static int                          s_afe_fetch_chunk = 0;  // samples per AFE fetch output
static const esp_mn_iface_t        *s_multinet  = nullptr;
static model_iface_data_t          *s_mn_data   = nullptr;
static int                          s_mn_chunk  = 0;

// ── Two-stage stateful resamplers: in_rate → 16 kHz ──────────────────────────
// One for mic channel, one for reference channel.
// State persists across iterations so WakeNet LSTM sees a continuous stream.
typedef struct { uint32_t in_rate; uint64_t pos; } rs_state_t;
static rs_state_t s_rs     = {0, 0};   // mic resampler state

static int resample_to_16k(rs_state_t *rs, const int16_t *in, int in_cnt,
                            uint32_t in_rate, int16_t *out, int out_max)
{
    if (rs->in_rate != in_rate) {
        rs->in_rate = in_rate;
        rs->pos     = 0;
    }

    int n = 0;
    uint32_t half_rate = in_rate / 2;

    if (half_rate >= VOICE_SR_HZ) {
        // in_rate >= 32000 Hz: average adjacent pairs first (anti-alias), then interpolate.
        // e.g. 44100 → pairs averaged to 22050 domain → linear interp to 16000.
        // pos advances by half_rate per output sample; i0 is index into the averaged domain.
        while (n < out_max) {
            uint32_t i0 = (uint32_t)(rs->pos / VOICE_SR_HZ);
            if ((int)(i0 * 2 + 3) >= in_cnt) break;

            int s0 = ((int)in[i0*2 + 0] + in[i0*2 + 1]) >> 1;
            int s1 = ((int)in[i0*2 + 2] + in[i0*2 + 3]) >> 1;

            int f  = (int)((rs->pos % VOICE_SR_HZ) * 256 / VOICE_SR_HZ);
            out[n] = (int16_t)(((256 - f) * s0 + f * s1) >> 8);
            ++n;
            rs->pos += half_rate;
        }
    } else {
        // in_rate < 32000 Hz (e.g. 22050, 16000): direct linear interpolation.
        // Do NOT pair-average — that would halve bandwidth to in_rate/2 (e.g. 5512 Hz),
        // cutting the 5–8 kHz range where 'S' / 'sh' consonants live and breaking WakeNet.
        // pos advances by in_rate per output sample; i0 indexes directly into in[].
        while (n < out_max) {
            uint32_t i0 = (uint32_t)(rs->pos / VOICE_SR_HZ);
            if ((int)(i0 + 1) >= in_cnt) break;

            int f  = (int)((rs->pos % VOICE_SR_HZ) * 256 / VOICE_SR_HZ);
            out[n] = (int16_t)(((256 - f) * (int)in[i0] + f * (int)in[i0 + 1]) >> 8);
            ++n;
            rs->pos += in_rate;
        }
    }

    uint32_t i0_last = (uint32_t)(rs->pos / VOICE_SR_HZ);
    rs->pos -= (uint64_t)i0_last * VOICE_SR_HZ;

    return n;
}

// ── I2S1 slave mic init ──────────────────────────────────────────────────────
#define MIC_BCLK_PIN   GPIO_NUM_15
#define MIC_WS_PIN     GPIO_NUM_38
#define MIC_DIN_PIN    GPIO_NUM_16

static esp_err_t mic_i2s_init(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num  = 8;
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
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    // 32-bit slot to match I2S0 master; ES8311 ADC audio in bits[31:16]
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_mic_rx, &std_cfg),
                        TAG_VSR, "I2S1 init_std_mode failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_mic_rx),
                        TAG_VSR, "I2S1 enable failed");
    return ESP_OK;
}

// ── feed_task: Core 0 ─────────────────────────────────────────────────────────
// Reads I2S1 mic, downsamples to 16kHz, feeds single-channel audio to AFE (M mode).
// Reference stream from audio.cpp is not used (AEC disabled).
static void feed_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));

    int max_i2s_frames = s_afe_feed_chunk * 48001 / VOICE_SR_HZ + 8;

    uint32_t *stereo  = (uint32_t *)heap_caps_malloc(max_i2s_frames * 8,   MALLOC_CAP_SPIRAM);
    int16_t  *mic_raw = (int16_t  *)heap_caps_malloc(max_i2s_frames * 2,   MALLOC_CAP_SPIRAM);
    int16_t  *mic_16k = (int16_t  *)heap_caps_malloc(s_afe_feed_chunk * 2, MALLOC_CAP_SPIRAM);

    if (!stereo || !mic_raw || !mic_16k) {
        ESP_LOGE(TAG_VSR, "feed_task OOM");
        heap_caps_free(stereo); heap_caps_free(mic_raw); heap_caps_free(mic_16k);
        vTaskDelete(nullptr); return;
    }

    uint32_t last_rms_log = 0;
    ESP_LOGI(TAG_VSR, "feed_task started — feed_chunk=%d", s_afe_feed_chunk);

    for (;;) {
        uint32_t in_rate = g_in_rate;
        int frames_needed = s_afe_feed_chunk * (int)in_rate / VOICE_SR_HZ + 4;
        if (frames_needed > max_i2s_frames) frames_needed = max_i2s_frames;

        // Read mic from I2S1
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(s_mic_rx, stereo,
                                          frames_needed * 8, &bytes_read,
                                          pdMS_TO_TICKS(200));
        if (ret != ESP_OK || bytes_read < 16) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        int actual_frames = (int)(bytes_read / 8);

        // Extract mic L channel: ES8311 ADC audio in bits[31:16] of 32-bit slot
        for (int i = 0; i < actual_frames; i++) {
            int v = (int)(int16_t)(stereo[i * 2] >> 16) * 2;  // 2× SW gain
            mic_raw[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
        }

        // Resample mic from in_rate → 16kHz
        int mic_cnt = resample_to_16k(&s_rs, mic_raw, actual_frames, in_rate,
                                       mic_16k, s_afe_feed_chunk);
        if (mic_cnt < s_afe_feed_chunk)
            memset(mic_16k + mic_cnt, 0, (s_afe_feed_chunk - mic_cnt) * 2);

        // Feed single-channel mic audio to AFE (M mode)
        s_afe->feed(s_afe_data, mic_16k);

        // Periodic mic-level diagnostic (every 5 s)
        {
            uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now_ms - last_rms_log > 5000) {
                last_rms_log = now_ms;
                int64_t s2 = 0, f2 = 0;
                for (int i = 0; i < actual_frames;    i++) s2 += (int64_t)mic_raw[i] * mic_raw[i];
                for (int i = 0; i < s_afe_feed_chunk; i++) f2 += (int64_t)mic_16k[i] * mic_16k[i];
                ESP_LOGI(TAG_VSR, "MicRMS=%d FeedRMS=%d rate=%u frames=%d",
                    (int)sqrtf((float)(s2 / actual_frames)),
                    (int)sqrtf((float)(f2 / s_afe_feed_chunk)),
                    in_rate, actual_frames);
                if (actual_frames >= 4) {
                    ESP_LOGI(TAG_VSR, "RAW32[0..3]: %d %d %d %d",
                        (int)(int16_t)(stereo[0] >> 16), (int)(int16_t)(stereo[2] >> 16),
                        (int)(int16_t)(stereo[4] >> 16), (int)(int16_t)(stereo[6] >> 16));
                }
            }
        }
    }
}

// ── fetch_task: Core 1 — polls AFE output, handles WakeNet + MultiNet ─────────
static void fetch_task(void *arg)
{
    ESP_LOGI(TAG_VSR, "fetch_task started");

    // MultiNet buffer: accumulate AFE fetch chunks until we have one MultiNet chunk
    int16_t *mn_buf = (int16_t *)heap_caps_malloc(s_mn_chunk * sizeof(int16_t),
                                                   MALLOC_CAP_SPIRAM);
    if (!mn_buf) {
        ESP_LOGE(TAG_VSR, "fetch_task OOM");
        vTaskDelete(nullptr); return;
    }

    bool     mn_active  = false;
    int      mn_pos     = 0;
    uint32_t last_diag  = 0;

    for (;;) {
        afe_fetch_result_t *result = s_afe->fetch(s_afe_data);
        if (!result || result->ret_value != ESP_OK) continue;

        int fetch_chunk = s_afe_fetch_chunk; // samples in result->data

        // Periodic RawRMS diagnostic every 2 s
        {
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now - last_diag > 2000) {
                last_diag = now;
                int64_t s2 = 0;
                for (int i = 0; i < fetch_chunk; i++)
                    s2 += (int64_t)result->data[i] * result->data[i];
                ESP_LOGI(TAG_VSR, "RawRMS=%d mn_active=%d wakeup=%d",
                    (int)sqrtf((float)(s2 / fetch_chunk)),
                    mn_active ? 1 : 0, (int)result->wakeup_state);
            }
        }

        if (!mn_active) {
            bool manual = s_listen_req;
            if (manual) { s_listen_req = false; }

            bool woke = (result->wakeup_state == WAKENET_DETECTED ||
                         result->wakeup_state == WAKENET_CHANNEL_VERIFIED);

            if (woke)
                ESP_LOGI(TAG_VSR, "*** Wake word 'Hi ESP' detected! ***");
            if (manual)
                ESP_LOGI(TAG_VSR, "*** Manual listen activated — speak command ***");

            if (woke || manual) {
                // Disable WakeNet in AFE during command window to avoid re-triggers
                s_afe->disable_wakenet(s_afe_data);
                s_multinet->clean(s_mn_data);
                mn_active   = true;
                s_mn_active = true;
                mn_pos      = 0;
                ESP_LOGI(TAG_VSR, "MultiNet window open (chunk=%d)", s_mn_chunk);
            }
        }

        if (mn_active) {
            // Feed AFE-processed audio (after AEC) to MultiNet
            int space = s_mn_chunk - mn_pos;
            int copy  = (fetch_chunk < space) ? fetch_chunk : space;
            memcpy(mn_buf + mn_pos, result->data, copy * sizeof(int16_t));
            mn_pos += copy;

            if (mn_pos >= s_mn_chunk) {
                int64_t s2 = 0;
                for (int i = 0; i < s_mn_chunk; i++) s2 += (int64_t)mn_buf[i] * mn_buf[i];
                ESP_LOGI(TAG_VSR, "CMD DataRMS=%d",
                    (int)sqrtf((float)(s2 / s_mn_chunk)));

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
                    s_afe->enable_wakenet(s_afe_data);
                } else if (mn_state == ESP_MN_STATE_TIMEOUT) {
                    ESP_LOGI(TAG_VSR, "Command timeout");
                    mn_active   = false;
                    s_mn_active = false;
                    s_afe->enable_wakenet(s_afe_data);
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
    s_mn_active  = true;
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
    ESP_LOGI(TAG_VSR, "Initializing voice recognition (AFE M-mode, no AEC)...");

    // 1. Load models from the "model" SPIFFS partition
    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models || models->num == 0) {
        ESP_LOGE(TAG_VSR, "No models in 'model' partition — flash srmodels.bin first");
        return;
    }
    ESP_LOGI(TAG_VSR, "Models loaded: %d", models->num);
    for (int i = 0; i < models->num; i++)
        ESP_LOGI(TAG_VSR, "  [%d] %s", i, models->model_name[i]);

    // 2. Explicitly find the WakeNet model name from partition
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, "hiesp");
    if (!wn_name) {
        ESP_LOGE(TAG_VSR, "wn9_hiesp not found in models partition — cannot init WakeNet");
        return;
    }
    ESP_LOGI(TAG_VSR, "WakeNet model: %s", wn_name);

    // 3. Configure AFE: single mic (M mode), no AEC.
    // AEC was disabled because I2S0 DMA latency (~185ms at 22050 Hz) exceeds
    // aec_filter_length=4 coverage (128ms), causing AEC to misalign reference
    // and cancel voice instead of echo. M mode gives clean signal to WakeNet.
    // AFE_MODE_HIGH_PERF for maximum wake word detection accuracy (per XiaoZhi reference).
    afe_config_t *afe_config = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (!afe_config) {
        ESP_LOGE(TAG_VSR, "afe_config_init failed");
        return;
    }
    afe_config->aec_init               = false;
    afe_config->se_init                = false;
    afe_config->ns_init                = false;
    afe_config->vad_init               = false;
    afe_config->agc_init               = true;
    afe_config->agc_mode               = AFE_AGC_MODE_WAKENET;
    afe_config->wakenet_init           = true;
    afe_config->wakenet_model_name     = wn_name;
    afe_config->wakenet_mode           = DET_MODE_95;
    afe_config->memory_alloc_mode      = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config->afe_ringbuf_size       = 50;
    afe_config->afe_perferred_core     = 0;
    afe_config->afe_perferred_priority = 5;
    afe_config = afe_config_check(afe_config);
    ESP_LOGI(TAG_VSR, "AFE wakenet_model_name after check: %s",
             afe_config->wakenet_model_name ? afe_config->wakenet_model_name : "NULL");

    s_afe = esp_afe_handle_from_config(afe_config);
    if (!s_afe) { ESP_LOGE(TAG_VSR, "esp_afe_handle_from_config failed"); return; }
    s_afe_data = s_afe->create_from_config(afe_config);
    if (!s_afe_data) { ESP_LOGE(TAG_VSR, "AFE create_from_config failed"); return; }

    s_afe_feed_chunk  = s_afe->get_feed_chunksize(s_afe_data);
    s_afe_fetch_chunk = s_afe->get_fetch_chunksize(s_afe_data);
    ESP_LOGI(TAG_VSR, "AFE feed_chunk=%d fetch_chunk=%d", s_afe_feed_chunk, s_afe_fetch_chunk);

    s_afe->print_pipeline(s_afe_data);

    // 4. Find MultiNet model
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    if (!mn_name) { ESP_LOGE(TAG_VSR, "mn5q8_en not found in models"); return; }
    ESP_LOGI(TAG_VSR, "MultiNet: %s", mn_name);

    s_multinet = esp_mn_handle_from_name(mn_name);
    if (!s_multinet) { ESP_LOGE(TAG_VSR, "esp_mn_handle_from_name failed"); return; }
    s_mn_data = s_multinet->create(mn_name, 6000);
    if (!s_mn_data) { ESP_LOGE(TAG_VSR, "multinet->create failed"); return; }
    s_mn_chunk = s_multinet->get_samp_chunksize(s_mn_data);
    ESP_LOGI(TAG_VSR, "MultiNet chunk=%d samples", s_mn_chunk);

    // 5. Register voice commands
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
        ESP_LOGW(TAG_VSR, "%d command(s) could not be added", mn_err->num);
        for (int i = 0; i < mn_err->num; i++)
            ESP_LOGW(TAG_VSR, "  cmd %d: %s",
                mn_err->phrases[i]->command_id, mn_err->phrases[i]->string);
    }

    // 6. Init I2S1 slave for mic
    if (mic_i2s_init(44100) != ESP_OK) {
        ESP_LOGE(TAG_VSR, "mic I2S1 init failed");
        return;
    }

    // 7. Drain any stale reference data accumulated before voice_sr started
    {
        StreamBufferHandle_t ref = audio_get_ref_stream();
        if (ref) {
            uint8_t drain[256];
            while (xStreamBufferReceive(ref, drain, sizeof(drain), 0) > 0) {}
        }
    }

    // 8. Launch tasks — use SPIRAM stacks to save scarce internal RAM (54 KB free)
    // fetch_task runs AEC + WakeNet inside afe->fetch(), needs ≥ 12 KB stack; use 16 KB.
    xTaskCreatePinnedToCoreWithCaps(feed_task,  "voice_feed",  8192,  nullptr, 5, nullptr, 0,
                                    MALLOC_CAP_SPIRAM);
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskCreatePinnedToCoreWithCaps(fetch_task, "voice_fetch", 16384, nullptr, 5, nullptr, 1,
                                    MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG_VSR, "Started — AFE M-mode (no AEC), HIGH_PERF — say 'Hi ESP'");
}
