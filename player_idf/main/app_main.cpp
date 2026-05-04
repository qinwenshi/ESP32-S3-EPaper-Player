// app_main.cpp — ESP32-S3 e-Paper MP3 Player (ESP-IDF port)
//
// Ported from ESP32_EPAPER_PLAYER.ino (Arduino) to ESP-IDF v5.5
// All features preserved:
//   - MP3 playback via minimp3 + I2S
//   - LVGL 9.2 UI on 1.54" e-paper (200×200 SSD1681)
//   - ES8311 codec, ES8311 mic
//   - SD card with recursive scan + meta cache
//   - NVS position/track persistence
//   - Deep sleep on pause timeout, wake on BOOT
//   - Voice recognition (esp-sr WakeNet + MultiNet)
//   - Snail progress animation

#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "lvgl.h"

#include "epaper_driver_bsp.h"
#include "audio.h"
#include "sdcard.h"
#include "codec.h"
#include "nvs_state.h"
#include "buttons.h"
#include "voice_sr.h"
#include "sprite_anim.h"
#include "snail_icon.h"

static const char *TAG = "player";

// ── Millis helper (replaces Arduino millis()) ─────────────────────────────────
static inline uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}
static inline void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ── Pins ─────────────────────────────────────────────────────────────────────
#define EPD_MOSI   13
#define EPD_SCK    12
#define EPD_CS     11
#define EPD_DC     10
#define EPD_RST     9
#define EPD_BUSY    8
#define EPD_PWR     6

#define SD_CLK     39
#define SD_CMD     41
#define SD_D0      40

#define I2S_MCLK   14
#define I2S_BCLK   15
#define I2S_LRC    38
#define I2S_DOUT   45
#define I2S_DIN    16

#define CODEC_SDA  47
#define CODEC_SCL  48
#define AUDIO_PWR  42
#define PA_PIN     46
#define VBAT_LATCH 17

#define BOOT_BTN    0
#define PWR_BTN    18

#define SLEEP_TIMEOUT_MS  (2UL * 60UL * 1000UL)

// ── Display geometry ─────────────────────────────────────────────────────────
#define EPD_W 200
#define EPD_H 200

// Sprite area — 96×104 (half scale of 192×208 source frames), centred
#define SPRITE_DISP_W   96
#define SPRITE_DISP_H  104
#define SPRITE_X       ((EPD_W - SPRITE_DISP_W) / 2)   // = 52
#define SPRITE_Y         2

#define TITLE_Y    (SPRITE_Y + SPRITE_DISP_H + 4)       // = 110
#define ARTIST_Y   (TITLE_Y + 14)                       // = 124
#define FOLDER_Y   (ARTIST_Y + 13)                      // = 137
#define PROG_Y     (FOLDER_Y + 13 + 4)                  // = 154  (gap before bar)
#define PROG_H     3
#define SEP_Y      (PROG_Y + PROG_H + 4)                // = 161
#define CTRL_Y     (SEP_Y + 2)                          // = 163

#define SNAIL_OBJ_Y   (PROG_Y + PROG_H/2 - SNAIL_H/2)

// ── Hardware objects ──────────────────────────────────────────────────────────
static epaper_driver_display *epd = nullptr;

// ── LVGL ─────────────────────────────────────────────────────────────────────
static SemaphoreHandle_t  lvgl_mtx;
static lv_display_t      *disp = nullptr;

// ── UI widgets ───────────────────────────────────────────────────────────────
static lv_obj_t *img_sprite;
static lv_obj_t *lbl_title;
static lv_obj_t *lbl_artist;
static lv_obj_t *lbl_folder;
static lv_obj_t *lbl_time;
static lv_obj_t *bar_prog;
static lv_obj_t *lbl_ctrl;
static lv_obj_t *lbl_play_icon;
static lv_obj_t *lbl_vol;
static lv_obj_t *img_snail;

LV_FONT_DECLARE(font_cubic11_14)
LV_FONT_DECLARE(font_cubic11_11)

// ── Player state ─────────────────────────────────────────────────────────────
static std::vector<std::string> tracks;
static int     cur_track  = 0;
static bool    is_playing = false;
static char    g_title[128]  = "No Track";
static char    g_artist[64]  = "";
static char    g_folder[64]  = "";
static char    g_cur_file[256] = "";
static uint32_t g_track_start_ms  = 0;
static bool     g_meta_needs_save = false;
static bool     g_full_refresh_pending = false;
static uint32_t g_last_track_btn_ms    = 0;

// ── Voice mute management ─────────────────────────────────────────────────────
static bool     g_voice_muted      = false;
static uint8_t  g_saved_vol        = 40;
static bool     g_voice_was_paused = false;

// ── Volume levels (4 steps) ───────────────────────────────────────────────────
static const int VOL_LEVELS[] = {25, 50, 75, 100};
static const int VOL_N        = 4;
static int       g_vol_idx    = 1;   // default: index 1 = 50%

// ── Double-click detection for BOOT ──────────────────────────────────────────
static uint32_t s_boot_click1_ms = 0;  // time of first short BOOT press; 0=none pending

// ── NVS playback state ────────────────────────────────────────────────────────
static uint32_t g_restore_pos_sec  = 0;
static uint32_t g_last_nvs_save_ms = 0;

// ── EPD partial-refresh throttle ─────────────────────────────────────────────
static uint32_t g_last_flush_ms    = 0;
static bool     g_silent_flush     = false;
static int      g_partial_count    = 0;    // partial refreshes since last full refresh
#define MIN_FLUSH_MS                 160
#define PARTIAL_FULL_REFRESH_INTERVAL 500  // fallback: force full refresh every N partials if no track change

// ── EOF auto-advance debounce ─────────────────────────────────────────────────
static uint32_t g_eof_debounce = 0;

// ── Deep sleep ────────────────────────────────────────────────────────────────
static uint32_t g_paused_since_ms = 0;

// ── Prescan task ──────────────────────────────────────────────────────────────
static volatile bool g_prescan_allowed = false;
static int           g_prescan_idx     = 0;

static bool g_display_fresh = false;

// ── Audio EOF flag (set from audio callback) ──────────────────────────────────
static volatile bool g_audio_eof = false;

// ─────────────────────────────────────────────────────────────────────────────
// LVGL flush callback
// ─────────────────────────────────────────────────────────────────────────────
static void epd_flush_cb(lv_display_t *d, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t now = millis();
    int32_t wait = (int32_t)(MIN_FLUSH_MS - (now - g_last_flush_ms));
    if (wait > 0) vTaskDelay(pdMS_TO_TICKS(wait));
    g_last_flush_ms = millis();

    uint16_t *buf = (uint16_t*)px_map;
    for (int32_t y = 0; y < EPD_H; y++) {
        for (int32_t x = 0; x < EPD_W; x++) {
            uint16_t c = buf[y * EPD_W + x];
            uint8_t col = (c > 0x7FFF) ? DRIVER_COLOR_WHITE : DRIVER_COLOR_BLACK;
            epd->EPD_DrawColorPixel(EPD_W - 1 - y, x, col);
        }
    }
    if (!g_silent_flush) {
        epd->EPD_DisplayPart();
        if (++g_partial_count >= PARTIAL_FULL_REFRESH_INTERVAL) {
            g_full_refresh_pending = true;  // trigger full refresh in main_task
            ESP_LOGI(TAG, "EPD: partial_count=%d → full refresh queued", g_partial_count);
        } else if (g_partial_count % 5 == 0) {
            ESP_LOGI(TAG, "EPD: partial_count=%d/%d", g_partial_count, PARTIAL_FULL_REFRESH_INTERVAL);
        }
    }
    lv_display_flush_ready(d);
}

// ─────────────────────────────────────────────────────────────────────────────
// LVGL tick task
// ─────────────────────────────────────────────────────────────────────────────
static void lvgl_tick_task(void*)
{
    for (;;) { vTaskDelay(pdMS_TO_TICKS(2)); lv_tick_inc(2); }
}

// ─────────────────────────────────────────────────────────────────────────────
// Volume helpers  (declared here so build_screen() can use them)
// ─────────────────────────────────────────────────────────────────────────────
static const char *vol_label_str(int idx)
{
    static const char *labels[] = {"|...", "||..", "|||.", "||||"};
    if (idx < 0) idx = 0;
    if (idx >= VOL_N) idx = VOL_N - 1;
    return labels[idx];
}

// ─────────────────────────────────────────────────────────────────────────────
// Build player screen
// ─────────────────────────────────────────────────────────────────────────────
static void build_screen()
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);

    // ── Sprite widget (replaces static cover art) ──
    img_sprite = lv_image_create(scr);
    lv_obj_set_size(img_sprite, SPRITE_DISP_W, SPRITE_DISP_H);
    lv_obj_set_pos(img_sprite, SPRITE_X, SPRITE_Y);
    sprite_anim_init(img_sprite);

    lbl_title = lv_label_create(scr);
    lv_obj_set_width(lbl_title, EPD_W - 4);
    lv_obj_set_pos(lbl_title, 2, TITLE_Y);
    lv_obj_set_style_text_font(lbl_title, &font_cubic11_14, 0);
    lv_label_set_long_mode(lbl_title, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_title, g_title);

    lbl_artist = lv_label_create(scr);
    lv_obj_set_width(lbl_artist, 120);
    lv_obj_set_pos(lbl_artist, 2, ARTIST_Y);
    lv_obj_set_style_text_font(lbl_artist, &font_cubic11_11, 0);
    lv_label_set_long_mode(lbl_artist, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_artist, "");

    lbl_folder = lv_label_create(scr);
    lv_obj_set_width(lbl_folder, EPD_W - 4);
    lv_obj_set_pos(lbl_folder, 2, FOLDER_Y);
    lv_obj_set_style_text_font(lbl_folder, &font_cubic11_11, 0);
    lv_label_set_long_mode(lbl_folder, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_folder, "");

    lbl_time = lv_label_create(scr);
    lv_obj_set_width(lbl_time, 76);
    lv_obj_set_pos(lbl_time, EPD_W - 78, ARTIST_Y);
    lv_obj_set_style_text_font(lbl_time, &font_cubic11_11, 0);
    lv_obj_set_style_text_align(lbl_time, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_text(lbl_time, "0:00/0:00");

    bar_prog = lv_bar_create(scr);
    lv_obj_set_size(bar_prog, EPD_W - 4, PROG_H);
    lv_obj_set_pos(bar_prog, 2, PROG_Y);
    lv_bar_set_range(bar_prog, 0, 1000);
    lv_bar_set_value(bar_prog, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_prog, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_bg_color(bar_prog, lv_color_black(), LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_prog, 2, 0);
    lv_obj_set_style_radius(bar_prog, 2, LV_PART_INDICATOR);

    img_snail = lv_image_create(scr);
    lv_image_set_src(img_snail, &snail_dsc);
    lv_obj_set_pos(img_snail, 2, SNAIL_OBJ_Y);

    lv_obj_t *sep = lv_obj_create(scr);
    lv_obj_set_size(sep, EPD_W, 1);
    lv_obj_set_pos(sep, 0, SEP_Y);
    lv_obj_set_style_bg_color(sep, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sep, 0, 0);
    lv_obj_clear_flag(sep, LV_OBJ_FLAG_SCROLLABLE);

    lbl_ctrl = lv_label_create(scr);
    lv_obj_set_width(lbl_ctrl, EPD_W - 22);
    lv_obj_set_pos(lbl_ctrl, 2, CTRL_Y);
    lv_obj_set_style_text_font(lbl_ctrl, &font_cubic11_11, 0);
    lv_label_set_text(lbl_ctrl, "BOOT:暂停 双击:音量\nPWR:下一首/上一首");

    lbl_play_icon = lv_label_create(scr);
    lv_obj_set_width(lbl_play_icon, 20);
    lv_obj_set_pos(lbl_play_icon, EPD_W - 21, CTRL_Y + 2);
    lv_obj_set_style_text_font(lbl_play_icon, &lv_font_montserrat_12, 0);
    lv_label_set_text(lbl_play_icon, LV_SYMBOL_PLAY);

    // ── Volume icon — top-right corner (clear of sprite x=52..148) ──
    lbl_vol = lv_label_create(scr);
    lv_obj_set_width(lbl_vol, 46);
    lv_obj_set_pos(lbl_vol, EPD_W - 48, SPRITE_Y + 2);  // x=152, y=4
    lv_obj_set_style_text_font(lbl_vol, &font_cubic11_11, 0);
    lv_obj_set_style_text_align(lbl_vol, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_text(lbl_vol, vol_label_str(g_vol_idx));
}

// ─────────────────────────────────────────────────────────────────────────────
// Volume helpers
// ─────────────────────────────────────────────────────────────────────────────
static void cycle_volume()
{
    g_vol_idx = (g_vol_idx + 1) % VOL_N;
    codec_set_volume(VOL_LEVELS[g_vol_idx]);
    if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        lv_label_set_text(lbl_vol, vol_label_str(g_vol_idx));
        xSemaphoreGive(lvgl_mtx);
    }
    ESP_LOGI(TAG, "Volume: level %d → %d%%", g_vol_idx + 1, VOL_LEVELS[g_vol_idx]);
}

// ─────────────────────────────────────────────────────────────────────────────
// refresh_ui
// ─────────────────────────────────────────────────────────────────────────────
static void refresh_ui()
{
    lv_label_set_text(lbl_title,  g_title[0] ? g_title : "No Track");
    lv_label_set_text(lbl_artist, g_artist);
    lv_label_set_text(lbl_folder, g_folder);

    uint32_t cur = audio_get_current_time();
    uint32_t tot = audio_get_duration();
    char tbuf[24];
    snprintf(tbuf, sizeof(tbuf), "%" PRIu32 ":%02" PRIu32 "/%" PRIu32 ":%02" PRIu32,
             cur/60, cur%60, tot/60, tot%60);
    lv_label_set_text(lbl_time, tbuf);

    int32_t val = (tot > 0) ? (int32_t)(cur * 1000 / tot) : 0;
    lv_bar_set_value(bar_prog, val, LV_ANIM_OFF);

    int32_t sx = 2 + (int32_t)(val * (EPD_W - 4 - SNAIL_W) / 1000);
    lv_obj_set_x(img_snail, sx);

    lv_label_set_text(lbl_play_icon, is_playing ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
}

// ─────────────────────────────────────────────────────────────────────────────
// EPD helpers
// ─────────────────────────────────────────────────────────────────────────────
static void epd_full_refresh()
{
    g_partial_count = 0;
    ESP_LOGI(TAG, "EPD full refresh");
    // EPD_Init_Partial() changes registers 0x37, 0x3C (BorderWaveform→0x80) and
    // others that prevent a proper full-waveform flash if we only swap the LUT.
    // The only reliable way to return to full-refresh mode is EPD_Init(), which
    // does: HW reset + SW reset + driver output / data-entry / border / temp /
    // cursor setup + loads WF_Full_1IN54 LUT.
    epd->EPD_Init();
    // Write current buffer to both display RAMs (new frame + base frame) and
    // activate with TurnOnDisplay(0xC7) → full waveform → visible black→white flash.
    epd->EPD_DisplayPartBaseImage();
    // Restore partial-refresh mode for subsequent LVGL-driven updates.
    epd->EPD_Init_Partial();
}

static void epd_save_frame()
{
    uint8_t *tmp = (uint8_t*)heap_caps_malloc(5000, MALLOC_CAP_SPIRAM);
    if (!tmp) return;
    epd->EPD_GetBuffer(tmp);
    sdcard_write_file("/sdcard/.epd_frame", tmp, 5000);
    heap_caps_free(tmp);
}

static bool epd_restore_frame()
{
    uint8_t *tmp = (uint8_t*)heap_caps_malloc(5000, MALLOC_CAP_SPIRAM);
    if (!tmp) return false;
    bool ok = sdcard_read_file("/sdcard/.epd_frame", tmp, 5000);
    if (ok) epd->EPD_WriteFrameToRAMSilent(tmp);
    heap_caps_free(tmp);
    return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// ID3v2 text parser (same logic as Arduino version)
// ─────────────────────────────────────────────────────────────────────────────
static bool parse_id3_text(const char *mp3_path,
                             char *out_title,  size_t title_sz,
                             char *out_artist, size_t artist_sz)
{
    FILE *f = fopen(mp3_path, "rb");
    if (!f) return false;
    uint8_t hdr[10];
    if (fread(hdr, 1, 10, f) != 10 || memcmp(hdr, "ID3", 3) != 0) { fclose(f); return false; }
    uint8_t  ver    = hdr[3];
    uint8_t  flags  = hdr[5];
    uint32_t tag_sz = ((uint32_t)(hdr[6]&0x7F)<<21)|((uint32_t)(hdr[7]&0x7F)<<14)
                     |((uint32_t)(hdr[8]&0x7F)<<7) | (uint32_t)(hdr[9]&0x7F);
    if (flags & 0x40) {
        uint8_t eh[4];
        if (fread(eh, 1, 4, f) == 4) {
            uint32_t ehsz = ((uint32_t)eh[0]<<24)|((uint32_t)eh[1]<<16)|((uint32_t)eh[2]<<8)|eh[3];
            if (ehsz > 4) fseek(f, ehsz - 4, SEEK_CUR);
        }
    }
    bool got_title = false, got_artist = false;
    uint32_t read = 0;
    while (read < tag_sz && !(got_title && got_artist)) {
        uint8_t fhdr[10];
        if (fread(fhdr, 1, 10, f) != 10) break;
        if (fhdr[0] == 0) break;
        read += 10;
        char fid[5] = {(char)fhdr[0],(char)fhdr[1],(char)fhdr[2],(char)fhdr[3],0};
        uint32_t fsz = (ver >= 4)
            ? (((uint32_t)(fhdr[4]&0x7F)<<21)|((uint32_t)(fhdr[5]&0x7F)<<14)
              |((uint32_t)(fhdr[6]&0x7F)<<7) | (uint32_t)(fhdr[7]&0x7F))
            : (((uint32_t)fhdr[4]<<24)|((uint32_t)fhdr[5]<<16)|((uint32_t)fhdr[6]<<8)|fhdr[7]);
        if (fsz == 0 || read + fsz > tag_sz + 10) break;
        bool is_t = !strcmp(fid,"TIT2"), is_a = !strcmp(fid,"TPE1");
        if (is_t || is_a) {
            uint8_t enc = 0; fread(&enc, 1, 1, f);
            uint32_t tlen = fsz - 1;
            char  *dst = is_t ? out_title  : out_artist;
            size_t dsz = is_t ? title_sz   : artist_sz;
            if (enc == 0 || enc == 3) {
                uint32_t r = (tlen < dsz - 1) ? tlen : (uint32_t)(dsz - 1);
                fread(dst, 1, r, f); dst[r] = '\0';
                if (tlen > r) fseek(f, tlen - r, SEEK_CUR);
            } else {
                fseek(f, tlen, SEEK_CUR);
            }
            if (is_t) got_title  = true;
            if (is_a) got_artist = true;
        } else {
            fseek(f, fsz, SEEK_CUR);
        }
        read += fsz;
    }
    fclose(f);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Meta prescan task
// ─────────────────────────────────────────────────────────────────────────────
static void meta_prescan_task(void *)
{
    for (;;) {
        if (!g_prescan_allowed || tracks.empty()) {
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        int  total    = (int)tracks.size();
        bool did_work = false;
        for (int i = 0; i < total; i++) {
            if (!g_prescan_allowed) break;
            int idx = (g_prescan_idx + i) % total;
            const char *mp3 = tracks[idx].c_str();
            if (!strcmp(mp3, g_cur_file)) continue;

            // Check if valid .meta already exists
            TrackMeta existing;
            if (sdcard_load_meta(mp3, existing)) continue;

            if (!g_prescan_allowed) break;

            char title[128] = {}, artist[64] = {};
            std::string fname(mp3);
            size_t s = fname.rfind('/'), d = fname.rfind('.');
            std::string base = fname.substr(s + 1, d - s - 1);
            size_t sep = base.find(" - ");
            if (sep != std::string::npos) {
                strncpy(artist, base.substr(0, sep).c_str(), sizeof(artist)-1);
                strncpy(title,  base.substr(sep + 3).c_str(), sizeof(title)-1);
            } else {
                strncpy(title, base.c_str(), sizeof(title)-1);
            }

            parse_id3_text(mp3, title, sizeof(title), artist, sizeof(artist));
            if (!g_prescan_allowed) break;

            TrackMeta meta = {};
            strncpy(meta.title,  title,  sizeof(meta.title)-1);
            strncpy(meta.artist, artist, sizeof(meta.artist)-1);
            meta.has_cover = false;
            sdcard_save_meta(mp3, meta);

            ESP_LOGI(TAG, "[prescan] %s — %s", artist, title);
            g_prescan_idx = (idx + 1) % total;
            did_work = true;
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }
        if (!did_work) {
            g_prescan_idx = 0;
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Audio callbacks
// ─────────────────────────────────────────────────────────────────────────────
static void on_meta(const char *key, const char *value)
{
    if (strncmp(key, "Title: ", 7) == 0) {
        if (xSemaphoreTake(lvgl_mtx, portMAX_DELAY) == pdTRUE) {
            strncpy(g_title, value, sizeof(g_title)-1);
            lv_label_set_text(lbl_title, g_title);
            xSemaphoreGive(lvgl_mtx);
        }
        g_meta_needs_save = true;
    } else if (strncmp(key, "Artist: ", 8) == 0) {
        if (xSemaphoreTake(lvgl_mtx, portMAX_DELAY) == pdTRUE) {
            strncpy(g_artist, value, sizeof(g_artist)-1);
            lv_label_set_text(lbl_artist, g_artist);
            xSemaphoreGive(lvgl_mtx);
        }
        g_meta_needs_save = true;
    } else if (strncmp(key, "SampleRate: ", 12) == 0) {
        uint32_t sr = (uint32_t)atoi(value);
        if (sr >= 8000 && sr <= 96000) {
            codec_set_sample_rate(sr);
            voice_sr_set_input_rate(sr);
            ESP_LOGI(TAG, "Sample rate -> %u Hz", sr);
        }
    }
}

static void on_eof(void)
{
    g_audio_eof = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// play_track
// ─────────────────────────────────────────────────────────────────────────────
// direction: +1 = forward/next, -1 = backward/prev, 0 = restart/auto
static void play_track(int idx, int direction = 0)
{
    if (tracks.empty()) return;
    idx = ((idx % (int)tracks.size()) + (int)tracks.size()) % (int)tracks.size();
    cur_track  = idx;
    is_playing = true;

    nvs_state_set_int("track", idx);
    nvs_state_set_uint("pos", 0);
    g_restore_pos_sec = 0;
    g_eof_debounce    = 0;
    g_audio_eof       = false;
    g_prescan_allowed = false;

    // Sprite: establish IDLE as the base state, then show direction animation
    if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
        sprite_anim_set_state(SPRITE_STATE_IDLE, false);  // set base = idle
        if (direction > 0)
            sprite_anim_set_state(SPRITE_STATE_RUNNING_RIGHT, true);
        else if (direction < 0)
            sprite_anim_set_state(SPRITE_STATE_RUNNING_LEFT, true);
        else
            sprite_anim_set_state(SPRITE_STATE_RUNNING, true);
        xSemaphoreGive(lvgl_mtx);
    }

    const char *path = tracks[idx].c_str();
    strncpy(g_cur_file, path, sizeof(g_cur_file)-1);

    memset(g_title,  0, sizeof(g_title));
    memset(g_artist, 0, sizeof(g_artist));
    memset(g_folder, 0, sizeof(g_folder));
    g_meta_needs_save = false;
    g_track_start_ms  = millis();

    std::string fname(path);
    size_t s = fname.rfind('/'), d = fname.rfind('.');
    std::string base = fname.substr(s + 1, d - s - 1);
    // Extract parent folder name (one level up from the file)
    if (s != std::string::npos && s > 0) {
        size_t s2 = fname.rfind('/', s - 1);
        std::string folder = (s2 != std::string::npos)
                             ? fname.substr(s2 + 1, s - s2 - 1)
                             : fname.substr(0, s);
        strncpy(g_folder, folder.c_str(), sizeof(g_folder)-1);
    }
    size_t sep = base.find(" - ");
    if (sep != std::string::npos) {
        strncpy(g_artist, base.substr(0, sep).c_str(), sizeof(g_artist)-1);
        strncpy(g_title,  base.substr(sep + 3).c_str(), sizeof(g_title)-1);
    } else {
        strncpy(g_title, base.c_str(), sizeof(g_title)-1);
    }

    audio_stop();

    ESP_LOGI("player", "play_track: free internal=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    // Load cached metadata (title/artist only — no cover art)
    TrackMeta meta = {};
    if (sdcard_load_meta(path, meta) || strlen(meta.title) > 0) {
        strncpy(g_title,  meta.title,  sizeof(g_title)-1);
        strncpy(g_artist, meta.artist, sizeof(g_artist)-1);
    }

    audio_play(path);

    if (!g_display_fresh) {
        if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(500)) == pdTRUE) {
            refresh_ui();
            xSemaphoreGive(lvgl_mtx);
        }
        g_full_refresh_pending = true;
    }
    g_display_fresh = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Deep sleep
// ─────────────────────────────────────────────────────────────────────────────
static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "[SLEEP] Entering deep sleep");

    nvs_state_set_int("track", cur_track);
    nvs_state_set_uint("pos", audio_get_current_time());

    audio_stop();
    delay_ms(50);

    gpio_set_level((gpio_num_t)PA_PIN,    0);   // amp off
    gpio_set_level((gpio_num_t)AUDIO_PWR, 1);   // analog off

    epd_save_frame();

    esp_sleep_enable_ext1_wakeup(1ULL << BOOT_BTN, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();
}

// ─────────────────────────────────────────────────────────────────────────────
// Main task (replaces Arduino loop)
// ─────────────────────────────────────────────────────────────────────────────
static void main_task(void *)
{
    static uint32_t last_ui_tick   = 0;
    static bool     s_was_playing  = true;

    for (;;) {
        // ── Button: BOOT ──
        uint32_t held = 0;
        if (buttons_boot_fired(&held)) {
            uint32_t now = millis();
            if (now - g_last_track_btn_ms > 500) {
                if (held > 50) {
                    // Short press: check for double-click (volume) vs single-click (pause)
                    if (s_boot_click1_ms != 0 && (now - s_boot_click1_ms) < 400) {
                        // ── Double-click → cycle volume ──
                        s_boot_click1_ms = 0;
                        cycle_volume();
                    } else {
                        // ── First click — defer for 350 ms to catch potential double-click ──
                        s_boot_click1_ms = now;
                    }
                }
            }
        }

        // ── Pending single-click: fire pause/resume after 350 ms double-click window ──
        {
            uint32_t now = millis();
            if (s_boot_click1_ms != 0 && (now - s_boot_click1_ms) >= 350) {
                s_boot_click1_ms = 0;
                is_playing = !is_playing;
                audio_pause_resume();
                g_prescan_allowed = !is_playing;
                if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                    lv_label_set_text(lbl_play_icon,
                        is_playing ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
                    sprite_anim_set_state(
                        is_playing ? SPRITE_STATE_IDLE : SPRITE_STATE_WAITING, false);
                    xSemaphoreGive(lvgl_mtx);
                }
            }
        }

        // ── Restore volume after voice ──
        if (g_voice_muted && !voice_sr_is_listening()) {
            codec_dac_mute(false);
            codec_set_volume(g_saved_vol);
            g_voice_muted = false;
            if (g_voice_was_paused) {
                audio_pause_resume();
                is_playing = false;
                g_voice_was_paused = false;
                if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                    lv_label_set_text(lbl_play_icon, LV_SYMBOL_PAUSE);
                    sprite_anim_set_state(SPRITE_STATE_WAITING, false);
                    xSemaphoreGive(lvgl_mtx);
                }
            }
            ESP_LOGI(TAG, "[VOICE] DAC restored");
        }

        // ── Button: PWR ──
        if (buttons_pwr_fired(&held)) {
            uint32_t now = millis();
            if (now - g_last_track_btn_ms > 500) {
                g_last_track_btn_ms = now;
                if (held > 800) {
                    play_track(cur_track - 1, -1);
                } else {
                    play_track(cur_track + 1, +1);
                }
            }
        }

        // ── Voice commands ──
        {
            int vcmd = voice_sr_get_cmd();
            if (vcmd != VOICE_CMD_NONE) {
                ESP_LOGI(TAG, "[VOICE] Executing command %d", vcmd);
                switch (vcmd) {
                    case VOICE_CMD_NEXT:
                        play_track(cur_track + 1, +1);
                        break;
                    case VOICE_CMD_PREV:
                        play_track(cur_track - 1, -1);
                        break;
                    case VOICE_CMD_PAUSE:
                        if (is_playing) {
                            is_playing = false; audio_pause_resume();
                            g_prescan_allowed = true;
                            if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                                lv_label_set_text(lbl_play_icon, LV_SYMBOL_PAUSE);
                                sprite_anim_set_state(SPRITE_STATE_WAITING, false);
                                xSemaphoreGive(lvgl_mtx);
                            }
                        }
                        break;
                    case VOICE_CMD_RESUME:
                        if (!is_playing) {
                            is_playing = true; audio_pause_resume();
                            g_prescan_allowed = false;
                            if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                                lv_label_set_text(lbl_play_icon, LV_SYMBOL_PLAY);
                                sprite_anim_set_state(SPRITE_STATE_IDLE, false);
                                xSemaphoreGive(lvgl_mtx);
                            }
                        }
                        break;
                    case VOICE_CMD_VOL_UP:
                        codec_set_volume(std::min(100, (int)codec_get_volume() + 10));
                        if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                            sprite_anim_set_state(SPRITE_STATE_JUMPING, true);
                            xSemaphoreGive(lvgl_mtx);
                        }
                        break;
                    case VOICE_CMD_VOL_DOWN:
                        codec_set_volume(std::max(0, (int)codec_get_volume() - 10));
                        break;
                    default: break;
                }
            }
        }

        // ── Deferred EPD full refresh ──
        if (g_full_refresh_pending) {
            g_full_refresh_pending = false;
            last_ui_tick = millis();
            epd_full_refresh();
            epd_save_frame();
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // ── Restore position from NVS ──
        if (g_restore_pos_sec > 0 && audio_is_running()) {
            ESP_LOGI(TAG, "Restoring position to %us", g_restore_pos_sec);
            audio_seek(g_restore_pos_sec);
            g_restore_pos_sec = 0;
        }

        // ── Periodic NVS position save ──
        {
            uint32_t now = millis();
            if (is_playing && audio_is_running() && now - g_last_nvs_save_ms > 3000) {
                g_last_nvs_save_ms = now;
                nvs_state_set_uint("pos", audio_get_current_time());
            }
        }

        // ── Auto-advance on EOF ──
        if (is_playing && !tracks.empty() && (g_audio_eof || !audio_is_running())) {
            if (g_eof_debounce == 0) g_eof_debounce = millis();
            if (millis() - g_eof_debounce > 800) {
                g_eof_debounce = 0;
                g_audio_eof    = false;
                g_full_refresh_pending = true;  // full refresh between tracks
                play_track(cur_track + 1, 0);  // direction=0 → 'running' animation
            }
        } else {
            g_eof_debounce = 0;
        }

        // ── Save metadata cache ──
        if (g_meta_needs_save && g_track_start_ms > 0 &&
            millis() - g_track_start_ms > 8000) {
            g_meta_needs_save = false;
            TrackMeta meta = {};
            strncpy(meta.title,  g_title,  sizeof(meta.title)-1);
            strncpy(meta.artist, g_artist, sizeof(meta.artist)-1);
            meta.has_cover = false;
            sdcard_save_meta(g_cur_file, meta);
        }

        // ── Periodic UI refresh + sprite tick (every 300 ms) ──
        uint32_t now = millis();
        if (now - last_ui_tick >= 300) {
            last_ui_tick = now;
            if (xSemaphoreTake(lvgl_mtx, 0) == pdTRUE) {
                sprite_anim_tick();
                if ((now / 300) % 4 == 0) refresh_ui();  // full text refresh every ~1.2 s
                xSemaphoreGive(lvgl_mtx);
            }
        }

        // ── Deep sleep ──
        if (is_playing != s_was_playing) {
            s_was_playing = is_playing;
            g_paused_since_ms = is_playing ? 0 : millis();
        }
        if (!is_playing && g_paused_since_ms != 0 &&
            millis() - g_paused_since_ms > SLEEP_TIMEOUT_MS) {
            enter_deep_sleep();
        }

        // ── LVGL handler ──
        if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
            lv_timer_handler();
            xSemaphoreGive(lvgl_mtx);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// app_main (replaces Arduino setup())
// ─────────────────────────────────────────────────────────────────────────────
// All initialization runs in this task (large stack, Core 1)
static void setup_task(void *)
{
    // ── Power rails ──
    gpio_config_t out_cfg = {};
    out_cfg.mode      = GPIO_MODE_OUTPUT;
    out_cfg.pin_bit_mask = (1ULL << VBAT_LATCH) | (1ULL << EPD_PWR)
                         | (1ULL << AUDIO_PWR)  | (1ULL << PA_PIN);
    gpio_config(&out_cfg);
    gpio_set_level((gpio_num_t)VBAT_LATCH, 1);
    gpio_set_level((gpio_num_t)EPD_PWR,    0);   // e-paper ON (active LOW)
    gpio_set_level((gpio_num_t)AUDIO_PWR,  0);   // audio ON   (active LOW)
    gpio_set_level((gpio_num_t)PA_PIN,     1);   // amp ON     (active HIGH)
    delay_ms(50);

    // ── NVS ──
    nvs_state_init();

    // ── SD card ──
    custom_lcd_spi_t spi_cfg = {};
    spi_cfg.cs         = EPD_CS;
    spi_cfg.dc         = EPD_DC;
    spi_cfg.rst        = EPD_RST;
    spi_cfg.busy       = EPD_BUSY;
    spi_cfg.mosi       = EPD_MOSI;
    spi_cfg.scl        = EPD_SCK;
    spi_cfg.spi_host   = SPI2_HOST;
    spi_cfg.buffer_len = (EPD_W * EPD_H) / 8;
    epd = new epaper_driver_display(EPD_W, EPD_H, spi_cfg);
    epd->EPD_Init();

    // ── SD card ──
    if (sdcard_mount(SD_CLK, SD_CMD, SD_D0)) {
        sdcard_scan_tracks(tracks);
        ESP_LOGI(TAG, "Found %d tracks", (int)tracks.size());
    } else {
        ESP_LOGE(TAG, "SD mount failed");
    }

    // ── NVS: restore saved track/position ──
    int      saved_track = nvs_state_get_int("track", 0);
    uint32_t saved_pos   = nvs_state_get_uint("pos",  0);
    if (saved_track < 0 || saved_track >= (int)tracks.size()) saved_track = 0;

    // ── Pre-load title/artist for boot render ──
    if (!tracks.empty()) {
        const char *boot_path = tracks[saved_track].c_str();
        strncpy(g_cur_file, boot_path, sizeof(g_cur_file)-1);
        TrackMeta meta = {};
        if (sdcard_load_meta(boot_path, meta) || strlen(meta.title) > 0) {
            strncpy(g_title,  meta.title,  sizeof(g_title)-1);
            strncpy(g_artist, meta.artist, sizeof(g_artist)-1);
        }
        if (!g_title[0]) {
            std::string fname(boot_path);
            size_t s = fname.rfind('/'), d = fname.rfind('.');
            std::string base = fname.substr(s + 1, d - s - 1);
            size_t sep = base.find(" - ");
            if (sep != std::string::npos) {
                strncpy(g_artist, base.substr(0, sep).c_str(), sizeof(g_artist)-1);
                strncpy(g_title,  base.substr(sep + 3).c_str(), sizeof(g_title)-1);
            } else {
                strncpy(g_title, base.c_str(), sizeof(g_title)-1);
            }
        }
    }

    // ── Try to restore last EPD frame ──
    bool has_saved_frame = epd_restore_frame();
    if (!has_saved_frame) epd->EPD_Clear();

    // ── LVGL ──
    lv_init();
    lvgl_mtx = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(lvgl_tick_task,    "lv_tick",  1024,  nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCoreWithCaps(meta_prescan_task, "prescan",   8192,  nullptr, 1, nullptr, 0,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    disp = lv_display_create(EPD_W, EPD_H);
    uint8_t *lvgl_buf = (uint8_t*)heap_caps_malloc(EPD_W * EPD_H * 2, MALLOC_CAP_SPIRAM);
    configASSERT(lvgl_buf);
    lv_display_set_buffers(disp, lvgl_buf, nullptr, EPD_W * EPD_H * 2,
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(disp, epd_flush_cb);

    build_screen();

    if (xSemaphoreTake(lvgl_mtx, portMAX_DELAY) == pdTRUE) {
        refresh_ui();
        // Start in review (loading) state; play_track() will transition to idle/waiting
        sprite_anim_set_state(tracks.empty() ? SPRITE_STATE_FAILED : SPRITE_STATE_REVIEW, false);
        xSemaphoreGive(lvgl_mtx);
    }

    g_silent_flush = true;
    lv_obj_invalidate(lv_screen_active());
    lv_timer_handler();
    g_silent_flush = false;

    if (has_saved_frame) {
        epd->EPD_ReloadPartialLut();
        epd->EPD_DisplayPart();
    } else {
        epd->EPD_DisplayPartBaseImage();
        epd->EPD_Init_Partial();
    }
    epd_save_frame();

    // ── Codec + audio ──
    audio_init(I2S_BCLK, I2S_LRC, I2S_DOUT, I2S_MCLK);
    audio_set_callbacks(on_meta, nullptr, on_eof);
    delay_ms(10);

    if (!codec_init(CODEC_SDA, CODEC_SCL, 400000)) {
        ESP_LOGE(TAG, "ES8311 not found! Check wiring.");
    }
    codec_set_sample_rate(44100);
    codec_set_volume(VOL_LEVELS[g_vol_idx]);  // default: level 2 = 50%
    codec_enable_mic(true);
    codec_set_mic_gain(7);   // 42 dB max hardware PGA — no SW gain anymore, use full HW gain
    codec_read_all();

    // ── Voice recognition ──
    ESP_LOGI("player", "Before voice_sr: free internal=%u SPIRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    voice_sr_init();
    ESP_LOGI("player", "After  voice_sr: free internal=%u SPIRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // ── Buttons ──
    buttons_init(BOOT_BTN, PWR_BTN);

    // ── Auto-play ──
    if (!tracks.empty()) {
        g_display_fresh = true;
        play_track(saved_track);
        if (saved_pos > 0) g_restore_pos_sec = saved_pos;
    } else {
        if (xSemaphoreTake(lvgl_mtx, portMAX_DELAY) == pdTRUE) {
            lv_label_set_text(lbl_title, "SD空/无MP3");
            sprite_anim_set_state(SPRITE_STATE_FAILED, false);
            xSemaphoreGive(lvgl_mtx);
        }
    }

    // ── Main loop task on Core 1 ──
    xTaskCreatePinnedToCoreWithCaps(main_task, "main", 24576, nullptr, 5, nullptr, 1,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    vTaskDelete(nullptr);  // setup_task done
}

// Minimal app_main — just starts setup_task with a large stack.
// The IDF "main_task" default stack (3.5 KB) is far too small for full init.
extern "C" void app_main(void)
{
    xTaskCreatePinnedToCoreWithCaps(setup_task, "setup", 32768, nullptr, 5, nullptr, 1,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
