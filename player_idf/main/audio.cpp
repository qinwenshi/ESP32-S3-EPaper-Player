// audio.cpp — MP3 playback via minimp3 + ESP-IDF I2S driver (ESP-IDF v5.x)
#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#include "minimp3_ex.h"
#include "audio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "audio";

// ── State ────────────────────────────────────────────────────────────────────
static i2s_chan_handle_t  s_i2s_tx       = nullptr;
static i2s_chan_handle_t  s_i2s_rx       = nullptr;  // duplex RX for mic capture
static audio_meta_cb_t    s_meta_cb      = nullptr;
static audio_image_cb_t   s_image_cb     = nullptr;
static audio_eof_cb_t     s_eof_cb       = nullptr;

static char               s_current_path[512];
static volatile bool      s_is_running   = false;
static volatile bool      s_stop_req     = false;
static volatile bool      s_pause_req    = false;
static volatile uint32_t  s_seek_to      = UINT32_MAX; // UINT32_MAX = no seek pending

static volatile uint64_t  s_decoded_samples = 0;
static volatile uint32_t  s_current_hz      = 44100;
static volatile uint32_t  s_duration_secs   = 0;
static volatile long      s_file_size        = 0;
static volatile long      s_audio_start_off  = 0; // byte offset where audio data begins

static TaskHandle_t       s_task_handle  = nullptr;

// Pins stored for rate-change reinit
static int s_pin_bclk, s_pin_ws, s_pin_dout, s_pin_mclk;

// ── I2S helpers ──────────────────────────────────────────────────────────────

static void audio_i2s_init(int bclk, int ws, int dout, int mclk, uint32_t rate)
{
    // Create I2S0 in DUPLEX mode: TX for speaker, RX for mic (ES8311 DOUT/DIN share clock)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 256;  // 256×8B=2KB/desc, well within DMA 4092B limit
    i2s_new_channel(&chan_cfg, &s_i2s_tx, &s_i2s_rx);

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)mclk,
            .bclk = (gpio_num_t)bclk,
            .ws   = (gpio_num_t)ws,
            .dout = (gpio_num_t)dout,
            .din  = GPIO_NUM_16,   // ES8311 ADC → mic capture
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    i2s_channel_init_std_mode(s_i2s_tx, &std_cfg);
    i2s_channel_init_std_mode(s_i2s_rx, &std_cfg);
    i2s_channel_enable(s_i2s_tx);
    i2s_channel_enable(s_i2s_rx);
}

// Reinitialise at a new sample rate (track changed rate mid-stream).
static void audio_i2s_set_rate(uint32_t rate)
{
    // Both TX and RX share the I2S0 clock — disable both for clean reconfiguration
    i2s_channel_disable(s_i2s_rx);
    i2s_channel_disable(s_i2s_tx);
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    i2s_channel_reconfig_std_clock(s_i2s_tx, &clk);
    i2s_channel_enable(s_i2s_tx);
    i2s_channel_enable(s_i2s_rx);
    ESP_LOGI(TAG, "I2S sample rate changed to %" PRIu32 " Hz", rate);
}

// ── ID3v2 parser ─────────────────────────────────────────────────────────────

// Read a 4-byte syncsafe integer (7 bits per byte).
static uint32_t read_syncsafe(const uint8_t *b)
{
    return ((uint32_t)(b[0] & 0x7f) << 21) |
           ((uint32_t)(b[1] & 0x7f) << 14) |
           ((uint32_t)(b[2] & 0x7f) <<  7) |
           ((uint32_t)(b[3] & 0x7f));
}

// Read a 4-byte big-endian integer.
static uint32_t read_be32(const uint8_t *b)
{
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] <<  8) |  (uint32_t)b[3];
}

static void parse_id3v2(FILE *f, audio_meta_cb_t meta_cb, audio_image_cb_t image_cb)
{
    uint8_t hdr[10];
    if (fread(hdr, 1, 10, f) != 10) return;
    if (hdr[0] != 'I' || hdr[1] != 'D' || hdr[2] != '3') return;

    uint8_t ver   = hdr[3]; // major version (3 or 4)
    uint8_t flags = hdr[5];

    uint32_t tag_sz = read_syncsafe(hdr + 6); // total tag size excluding 10-byte header

    // Skip extended header if present (flag bit 6)
    if (flags & 0x40) {
        uint8_t ext[4];
        if (fread(ext, 1, 4, f) != 4) return;
        uint32_t ext_sz = (ver >= 4) ? read_syncsafe(ext) : read_be32(ext);
        // ext_sz includes its own 4 bytes in v2.3 but not in v2.4
        uint32_t skip = (ver >= 4) ? (ext_sz - 4) : (ext_sz - 4);
        fseek(f, (long)skip, SEEK_CUR);
    }

    long frames_end = 10 + (long)tag_sz;
    uint8_t fhdr[10];
    uint8_t text_buf[256];

    while (ftell(f) + 10 <= frames_end) {
        if (fread(fhdr, 1, 10, f) != 10) break;

        // Stop on padding (zero frame-id byte)
        if (fhdr[0] == 0) break;

        // Frame size: syncsafe in v2.4, big-endian in v2.3
        uint32_t fsz = (ver >= 4) ? read_syncsafe(fhdr + 4) : read_be32(fhdr + 4);
        if (fsz == 0) continue;

        char fid[5];
        memcpy(fid, fhdr, 4);
        fid[4] = '\0';

        if (strcmp(fid, "TIT2") == 0 || strcmp(fid, "TPE1") == 0) {
            uint8_t enc = 0;
            if (fread(&enc, 1, 1, f) != 1) break;
            uint32_t text_len = fsz - 1;
            if (text_len >= sizeof(text_buf)) text_len = sizeof(text_buf) - 1;
            size_t got = fread(text_buf, 1, text_len, f);
            text_buf[got] = '\0';
            // Only handle Latin-1 (enc==0) and UTF-8 (enc==3)
            if ((enc == 0 || enc == 3) && meta_cb) {
                if (strcmp(fid, "TIT2") == 0) meta_cb("Title",  (char *)text_buf);
                else                           meta_cb("Artist", (char *)text_buf);
            }
            // Seek past any remaining bytes we didn't read
            long remaining = (long)fsz - 1 - (long)got;
            if (remaining > 0) fseek(f, remaining, SEEK_CUR);

        } else if (strcmp(fid, "APIC") == 0) {
            // Record the file position at the start of APIC payload
            long apic_off = ftell(f);
            if (image_cb) image_cb((uint32_t)apic_off, fsz);
            fseek(f, (long)fsz, SEEK_CUR);

        } else {
            fseek(f, (long)fsz, SEEK_CUR);
        }

        if (ftell(f) >= frames_end) break;
    }

    // Leave file pointer after the full ID3 tag
    fseek(f, frames_end, SEEK_SET);
}

// ── Duration estimation ───────────────────────────────────────────────────────

// Scan the first few MP3 frame headers to get the bitrate, then estimate total
// duration from file size.  Returns 0 if no valid frame found.
static uint32_t estimate_duration(FILE *f, long file_size, long audio_start)
{
    // Try to find the first valid MP3 sync word within the first 4 KB of audio data.
    const int SCAN_BYTES = 4096;
    uint8_t *scan = (uint8_t *)heap_caps_malloc(SCAN_BYTES, MALLOC_CAP_SPIRAM);
    if (!scan) return 0;
    fseek(f, audio_start, SEEK_SET);
    int n = (int)fread(scan, 1, SCAN_BYTES, f);

    static const int bitrate_table[16] = {
        0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0
    };

    uint32_t result = 0;
    for (int i = 0; i < n - 4; i++) {
        if (scan[i] != 0xFF || (scan[i+1] & 0xE0) != 0xE0) continue;
        // MPEG Layer III check
        if ((scan[i+1] & 0x06) != 0x02) continue;
        int br_idx = (scan[i+2] >> 4) & 0x0F;
        int br_kbps = bitrate_table[br_idx];
        if (br_kbps <= 0) continue;

        long audio_bytes = file_size - audio_start;
        result = (uint32_t)((audio_bytes * 8ULL) / ((uint32_t)br_kbps * 1000));
        break;
    }
    heap_caps_free(scan);
    return result;
}

// ── Playback task ─────────────────────────────────────────────────────────────

static void audio_task(void *arg)
{
    static const size_t READ_BUF_SIZE = 16 * 1024;

    // mp3dec_t has large float arrays (~11KB) — allocate in SPIRAM to save stack
    mp3dec_t *dec = (mp3dec_t *)heap_caps_malloc(sizeof(mp3dec_t), MALLOC_CAP_SPIRAM);
    if (!dec) { ESP_LOGE(TAG, "mp3dec alloc failed"); s_is_running = false; vTaskDelete(nullptr); return; }
    mp3dec_init(dec);

    uint8_t  *read_buf = (uint8_t  *)heap_caps_malloc(READ_BUF_SIZE,
                             MALLOC_CAP_SPIRAM);
    int16_t  *pcm_buf  = (int16_t  *)heap_caps_malloc(
                             MINIMP3_MAX_SAMPLES_PER_FRAME * 2 * sizeof(int16_t),
                             MALLOC_CAP_SPIRAM);
    int32_t  *i2s_buf  = (int32_t  *)heap_caps_malloc(
                             MINIMP3_MAX_SAMPLES_PER_FRAME * 2 * sizeof(int32_t),
                             MALLOC_CAP_SPIRAM);

    if (!read_buf || !pcm_buf || !i2s_buf) {
        ESP_LOGE(TAG, "Buffer alloc failed");
        goto cleanup;
    }

    {
        FILE *f = fopen(s_current_path, "rb");
        if (!f) {
            ESP_LOGE(TAG, "Cannot open %s", s_current_path);
            goto cleanup;
        }

        // File size
        fseek(f, 0, SEEK_END);
        long file_size = ftell(f);
        fseek(f, 0, SEEK_SET);
        s_file_size = file_size;

        // Parse tags (advances file pointer past ID3 block)
        parse_id3v2(f, s_meta_cb, s_image_cb);
        s_audio_start_off = ftell(f);

        // Duration estimate
        s_duration_secs = estimate_duration(f, file_size, s_audio_start_off);

        // Seek back to start of audio data
        fseek(f, s_audio_start_off, SEEK_SET);

        s_decoded_samples = 0;
        s_current_hz      = 44100;

        int buf_fill = 0;

        while (!s_stop_req) {
            if (s_pause_req) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            // Handle pending seek request
            uint32_t seek_to = s_seek_to;
            if (seek_to != UINT32_MAX) {
                s_seek_to = UINT32_MAX;
                if (s_duration_secs > 0) {
                    float ratio = (float)seek_to / (float)s_duration_secs;
                    long byte_off = s_audio_start_off +
                                    (long)(ratio * (float)(file_size - s_audio_start_off));
                    fseek(f, byte_off, SEEK_SET);
                    buf_fill = 0;
                    s_decoded_samples = (uint64_t)seek_to * s_current_hz;
                    mp3dec_init(dec); // reset decoder state
                }
            }

            // Refill read buffer (sliding window)
            int to_read = (int)(READ_BUF_SIZE - buf_fill);
            if (to_read > 0) {
                int got = (int)fread(read_buf + buf_fill, 1, (size_t)to_read, f);
                buf_fill += got;
            }

            if (buf_fill == 0) break; // EOF

            mp3dec_frame_info_t info;
            int samples = mp3dec_decode_frame(dec, read_buf, buf_fill, pcm_buf, &info);

            if (info.frame_bytes > 0) {
                memmove(read_buf, read_buf + info.frame_bytes,
                        (size_t)(buf_fill - info.frame_bytes));
                buf_fill -= info.frame_bytes;
            }

            if (samples > 0) {
                s_decoded_samples += (uint32_t)samples;

                if ((uint32_t)info.hz != s_current_hz) {
                    audio_i2s_set_rate((uint32_t)info.hz);
                    s_current_hz = (uint32_t)info.hz;
                }

                // Convert 16-bit stereo/mono → 32-bit stereo
                int ch         = info.channels;
                int out_frames = samples / ch;
                for (int i = 0; i < out_frames; i++) {
                    i2s_buf[i*2 + 0] = (int32_t)pcm_buf[i*ch + 0]             << 16;
                    i2s_buf[i*2 + 1] = (int32_t)pcm_buf[i*ch + (ch > 1 ? 1 : 0)] << 16;
                }
                size_t written = 0;
                i2s_channel_write(s_i2s_tx, i2s_buf,
                                  (size_t)(out_frames * 2 * sizeof(int32_t)),
                                  &written, pdMS_TO_TICKS(200));

            } else if (info.frame_bytes == 0) {
                // No frame found and no more data — true EOF
                if (feof(f)) break;
                // Otherwise skip a byte to avoid infinite loops on corrupt data
                if (buf_fill > 0) {
                    memmove(read_buf, read_buf + 1, (size_t)(buf_fill - 1));
                    buf_fill--;
                }
            }
        }

        fclose(f);
    }

cleanup:
    heap_caps_free(dec);
    heap_caps_free(read_buf);
    heap_caps_free(pcm_buf);
    heap_caps_free(i2s_buf);

    s_is_running = false;
    s_task_handle = nullptr;

    if (s_eof_cb && !s_stop_req) s_eof_cb();

    vTaskDelete(nullptr);
}

// ── Public API ────────────────────────────────────────────────────────────────

void audio_init(int bclk, int ws, int dout, int mclk)
{
    s_pin_bclk = bclk;
    s_pin_ws   = ws;
    s_pin_dout = dout;
    s_pin_mclk = mclk;
    audio_i2s_init(bclk, ws, dout, mclk, 44100);
    ESP_LOGI(TAG, "audio_init: bclk=%d ws=%d dout=%d mclk=%d", bclk, ws, dout, mclk);
}

void audio_set_callbacks(audio_meta_cb_t meta_cb,
                          audio_image_cb_t image_cb,
                          audio_eof_cb_t eof_cb)
{
    s_meta_cb  = meta_cb;
    s_image_cb = image_cb;
    s_eof_cb   = eof_cb;
}

bool audio_play(const char *vfs_path)
{
    if (s_is_running) audio_stop();

    strncpy(s_current_path, vfs_path, sizeof(s_current_path) - 1);
    s_current_path[sizeof(s_current_path) - 1] = '\0';

    s_stop_req  = false;
    s_pause_req = false;
    s_seek_to   = UINT32_MAX;
    s_is_running = true;

    // audio_task uses SPIRAM stack: mp3dec_scratch_t inside mp3dec_decode_frame
    // is ~17KB; SPIRAM stack avoids internal RAM pressure.
    // mp3dec_t (~11KB) stays on SPIRAM heap to keep stack usage minimal.
    BaseType_t rc = xTaskCreatePinnedToCoreWithCaps(audio_task, "audio",
                                            49152, nullptr,
                                            10, &s_task_handle, 1,
                                            MALLOC_CAP_SPIRAM);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task (need 16KB contiguous internal)");
        s_is_running = false;
        return false;
    }
    return true;
}

void audio_stop(void)
{
    s_stop_req = true;
    // Wait for the task to finish (up to 3 s)
    for (int i = 0; i < 150 && s_is_running; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    s_stop_req = false;
}

void audio_pause_resume(void)
{
    s_pause_req = !s_pause_req;
}

bool audio_is_running(void)
{
    return s_is_running;
}

uint32_t audio_get_current_time(void)
{
    uint32_t hz = s_current_hz;
    if (hz == 0) return 0;
    return (uint32_t)(s_decoded_samples / hz);
}

uint32_t audio_get_duration(void)
{
    return s_duration_secs;
}

void audio_seek(uint32_t seconds)
{
    s_seek_to = seconds;
}

i2s_chan_handle_t audio_get_i2s_rx(void)
{
    return s_i2s_rx;
}
