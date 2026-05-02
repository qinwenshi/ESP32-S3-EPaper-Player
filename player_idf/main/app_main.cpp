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
#include "esp32s3/rom/tjpgd.h"

#include "lvgl.h"

#include "epaper_driver_bsp.h"
#include "audio.h"
#include "sdcard.h"
#include "codec.h"
#include "nvs_state.h"
#include "buttons.h"
#include "voice_sr.h"
#include "default_cover.h"
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

#define ART_SIZE   80
#define ART_X      ((EPD_W - ART_SIZE) / 2)
#define ART_Y       2

#define TITLE_Y    (ART_Y + ART_SIZE + 3)
#define ARTIST_Y   (TITLE_Y + 16)
#define PROG_Y     (ARTIST_Y + 12)
#define PROG_H     3
#define SEP1_Y     (PROG_Y + PROG_H + 2)

#define LYR0_Y     (SEP1_Y + 3)
#define LYR1_Y     (LYR0_Y + 16)
#define LYR2_Y     (LYR1_Y + 13)

#define SEP2_Y     (LYR2_Y + 13)
#define CTRL_Y     (SEP2_Y + 2)

#define SNAIL_OBJ_Y   (PROG_Y + PROG_H/2 - SNAIL_H/2)

// ── Cover art buffer (I1 format) ──────────────────────────────────────────────
#define COVER_W      ART_SIZE
#define COVER_H      ART_SIZE
#define COVER_STRIDE ((COVER_W + 7) / 8)

static uint8_t cover_buf[2 * 4 + COVER_STRIDE * COVER_H];

static lv_image_dsc_t cover_dsc = {
    .header = {
        .magic  = LV_IMAGE_HEADER_MAGIC,
        .cf     = LV_COLOR_FORMAT_I1,
        .flags  = 0,
        .w      = COVER_W,
        .h      = COVER_H,
        .stride = COVER_STRIDE,
    },
    .data_size = sizeof(cover_buf),
    .data      = cover_buf,
};

// ── Hardware objects ──────────────────────────────────────────────────────────
static epaper_driver_display *epd = nullptr;

// ── LVGL ─────────────────────────────────────────────────────────────────────
static SemaphoreHandle_t  lvgl_mtx;
static lv_display_t      *disp = nullptr;

// ── UI widgets ───────────────────────────────────────────────────────────────
static lv_obj_t *img_cover;
static lv_obj_t *lbl_title;
static lv_obj_t *lbl_artist;
static lv_obj_t *lbl_time;
static lv_obj_t *bar_prog;
static lv_obj_t *lbl_lyr0;
static lv_obj_t *lbl_lyr1;
static lv_obj_t *lbl_lyr2;
static lv_obj_t *lbl_ctrl;
static lv_obj_t *lbl_play_icon;
static lv_obj_t *img_snail;

LV_FONT_DECLARE(font_cubic11_14)
LV_FONT_DECLARE(font_cubic11_11)

// ── Player state ─────────────────────────────────────────────────────────────
static std::vector<std::string> tracks;
static int     cur_track  = 0;
static bool    is_playing = false;
static char    g_title[128]  = "No Track";
static char    g_artist[64]  = "";
static char    g_lyr[3][128] = {"","",""};
static bool    g_cover_dirty = false;
static uint32_t g_art_pos = 0, g_art_len = 0;
static char    g_cur_file[256] = "";
static uint32_t g_track_start_ms  = 0;
static bool     g_meta_needs_save = false;
static bool     g_full_refresh_pending = false;
static uint32_t g_last_track_btn_ms    = 0;

// ── Voice mute management ─────────────────────────────────────────────────────
static bool     g_voice_muted      = false;
static uint8_t  g_saved_vol        = 40;
static bool     g_voice_was_paused = false;

// ── NVS playback state ────────────────────────────────────────────────────────
static uint32_t g_restore_pos_sec  = 0;
static uint32_t g_last_nvs_save_ms = 0;

// ── EPD partial-refresh throttle ─────────────────────────────────────────────
static uint32_t g_last_flush_ms = 0;
static bool     g_silent_flush  = false;
#define MIN_FLUSH_MS 160

// ── EOF auto-advance debounce ─────────────────────────────────────────────────
static uint32_t g_eof_debounce = 0;

// ── Deep sleep ────────────────────────────────────────────────────────────────
static uint32_t g_paused_since_ms = 0;

// ── Cover decode task ─────────────────────────────────────────────────────────
struct CoverDecodeReq { char path[256]; uint32_t file_pos; uint32_t file_len; };
static CoverDecodeReq    g_cover_req;
static SemaphoreHandle_t g_cover_sem   = nullptr;
static volatile bool     g_cover_busy  = false;

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
// Build player screen
// ─────────────────────────────────────────────────────────────────────────────
static void load_default_cover()
{
    static_assert(sizeof(default_cover_data) == sizeof(cover_buf),
                  "default_cover_data size must match cover_buf");
    memcpy(cover_buf, default_cover_data, sizeof(cover_buf));
}

static void build_screen()
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);

    img_cover = lv_image_create(scr);
    lv_obj_set_size(img_cover, ART_SIZE, ART_SIZE);
    lv_obj_set_pos(img_cover, ART_X, ART_Y);
    lv_obj_set_style_border_width(img_cover, 1, 0);
    lv_obj_set_style_border_color(img_cover, lv_color_black(), 0);
    load_default_cover();
    lv_image_set_src(img_cover, &cover_dsc);

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

    lv_obj_t *sep1 = lv_obj_create(scr);
    lv_obj_set_size(sep1, EPD_W, 1);
    lv_obj_set_pos(sep1, 0, SEP1_Y);
    lv_obj_set_style_bg_color(sep1, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(sep1, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sep1, 0, 0);
    lv_obj_clear_flag(sep1, LV_OBJ_FLAG_SCROLLABLE);

    lbl_lyr0 = lv_label_create(scr);
    lv_obj_set_width(lbl_lyr0, EPD_W - 4);
    lv_obj_set_pos(lbl_lyr0, 2, LYR0_Y);
    lv_obj_set_style_text_font(lbl_lyr0, &font_cubic11_14, 0);
    lv_obj_set_style_bg_color(lbl_lyr0, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lbl_lyr0, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(lbl_lyr0, lv_color_white(), 0);
    lv_obj_set_style_pad_ver(lbl_lyr0, 1, 0);
    lv_label_set_long_mode(lbl_lyr0, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_lyr0, "");

    lbl_lyr1 = lv_label_create(scr);
    lv_obj_set_width(lbl_lyr1, EPD_W - 4);
    lv_obj_set_pos(lbl_lyr1, 2, LYR1_Y);
    lv_obj_set_style_text_font(lbl_lyr1, &font_cubic11_11, 0);
    lv_label_set_long_mode(lbl_lyr1, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_lyr1, "");

    lbl_lyr2 = lv_label_create(scr);
    lv_obj_set_width(lbl_lyr2, EPD_W - 4);
    lv_obj_set_pos(lbl_lyr2, 2, LYR2_Y);
    lv_obj_set_style_text_font(lbl_lyr2, &font_cubic11_11, 0);
    lv_label_set_long_mode(lbl_lyr2, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_lyr2, "");

    img_snail = lv_image_create(scr);
    lv_image_set_src(img_snail, &snail_dsc);
    lv_obj_set_pos(img_snail, 2, SNAIL_OBJ_Y);

    lv_obj_t *sep2 = lv_obj_create(scr);
    lv_obj_set_size(sep2, EPD_W, 1);
    lv_obj_set_pos(sep2, 0, SEP2_Y);
    lv_obj_set_style_bg_color(sep2, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(sep2, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sep2, 0, 0);
    lv_obj_clear_flag(sep2, LV_OBJ_FLAG_SCROLLABLE);

    lbl_ctrl = lv_label_create(scr);
    lv_obj_set_width(lbl_ctrl, EPD_W - 22);
    lv_obj_set_pos(lbl_ctrl, 2, CTRL_Y);
    lv_obj_set_style_text_font(lbl_ctrl, &font_cubic11_11, 0);
    lv_label_set_text(lbl_ctrl, "BOOT:暂停  PWR:下一首\nBOOT长按:上一首");

    lbl_play_icon = lv_label_create(scr);
    lv_obj_set_width(lbl_play_icon, 18);
    lv_obj_set_pos(lbl_play_icon, EPD_W - 20, CTRL_Y + 6);
    lv_obj_set_style_text_font(lbl_play_icon, &lv_font_montserrat_12, 0);
    lv_label_set_text(lbl_play_icon, LV_SYMBOL_PLAY);
}

// ─────────────────────────────────────────────────────────────────────────────
// refresh_ui
// ─────────────────────────────────────────────────────────────────────────────
static void refresh_ui()
{
    lv_label_set_text(lbl_title,  g_title[0] ? g_title : "No Track");
    lv_label_set_text(lbl_artist, g_artist);
    lv_label_set_text(lbl_lyr0,   g_lyr[0]);
    lv_label_set_text(lbl_lyr1,   g_lyr[1]);
    lv_label_set_text(lbl_lyr2,   g_lyr[2]);

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
    epd->EPD_LoadFullLut();
    g_silent_flush = true;
    lv_obj_invalidate(lv_screen_active());
    lv_timer_handler();
    g_silent_flush = false;
    epd->EPD_DisplayPartBaseImage();
    epd->EPD_ReloadPartialLut();
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
// JPEG → Floyd-Steinberg dither → cover_buf (I1 format)
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t jpegWork[3100];

struct JpegCtx {
    FILE    *file;
    uint32_t remaining;
    uint8_t *gray;
    int      outW, outH;
};

static UINT jpeg_input(JDEC *jd, BYTE *buf, UINT len)
{
    JpegCtx *ctx = (JpegCtx*)jd->device;
    if (len > ctx->remaining) len = ctx->remaining;
    if (buf) fread(buf, 1, len, ctx->file);
    else     fseek(ctx->file, len, SEEK_CUR);
    ctx->remaining -= len;
    return len;
}

static UINT jpeg_output(JDEC *jd, void *bmp, JRECT *rect)
{
    JpegCtx *ctx = (JpegCtx*)jd->device;
    uint8_t *src = (uint8_t*)bmp;
    int sw = rect->right - rect->left + 1;
    int sh = rect->bottom - rect->top + 1;
    for (int r = 0; r < sh; r++) {
        int dy = rect->top + r;
        if (dy >= ctx->outH) continue;
        for (int c = 0; c < sw; c++) {
            int dx = rect->left + c;
            if (dx >= ctx->outW) continue;
            uint8_t R = src[(r*sw+c)*3+0];
            uint8_t G = src[(r*sw+c)*3+1];
            uint8_t B = src[(r*sw+c)*3+2];
            ctx->gray[dy * ctx->outW + dx] = (R*77 + G*150 + B*29) >> 8;
        }
    }
    return 1;
}

static void decode_cover(const char *path, uint32_t file_pos, uint32_t file_len);

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
// Cover decode
// ─────────────────────────────────────────────────────────────────────────────
static void decode_cover(const char *path, uint32_t file_pos, uint32_t file_len)
{
    uint8_t *gray = (uint8_t*)heap_caps_malloc(COVER_W * COVER_H, MALLOC_CAP_SPIRAM);
    if (!gray) return;
    memset(gray, 128, COVER_W * COVER_H);

    FILE *f = fopen(path, "rb");
    if (!f) { heap_caps_free(gray); return; }

    fseek(f, file_pos, SEEK_SET);
    uint32_t scanned = 0;
    bool found = false;
    while (scanned + 1 < file_len) {
        uint8_t b; fread(&b, 1, 1, f); scanned++;
        if (b == 0xFF) {
            uint8_t nb; fread(&nb, 1, 1, f);
            if (nb == 0xD8) {
                fseek(f, -2, SEEK_CUR); found = true; break;
            } else {
                fseek(f, -1, SEEK_CUR);
            }
        }
    }

    if (found) {
        JpegCtx ctx = { f, file_len - scanned, gray, COVER_W, COVER_H };
        JDEC jd;
        if (jd_prepare(&jd, jpeg_input, jpegWork, sizeof(jpegWork), &ctx) == JDR_OK) {
            uint8_t scale = 0;
            while (scale < 3 && (jd.width >> scale) > (uint16_t)COVER_W) scale++;
            jd_decomp(&jd, jpeg_output, scale);
        }
    }
    fclose(f);

    int16_t *err = (int16_t*)heap_caps_malloc(COVER_W * COVER_H * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!err) { heap_caps_free(gray); return; }
    for (int i = 0; i < COVER_W * COVER_H; i++) err[i] = gray[i];

    lv_color32_t *pal = (lv_color32_t*)cover_buf;
    pal[0] = (lv_color32_t){.blue=0,   .green=0,   .red=0,   .alpha=255};
    pal[1] = (lv_color32_t){.blue=255, .green=255, .red=255, .alpha=255};
    uint8_t *bits = cover_buf + 2 * sizeof(lv_color32_t);
    memset(bits, 0, COVER_STRIDE * COVER_H);

    for (int y = 0; y < COVER_H; y++) {
        for (int x = 0; x < COVER_W; x++) {
            int16_t old = err[y*COVER_W + x];
            int16_t clamp = (old < 0) ? 0 : (old > 255) ? 255 : old;
            uint8_t bit  = (clamp >= 128) ? 1 : 0;
            int16_t qerr = clamp - (bit ? 255 : 0);
            if (bit) bits[y*COVER_STRIDE + (x>>3)] |= (0x80 >> (x&7));
            if (x+1 < COVER_W) { err[y*COVER_W+x+1] += qerr*7/16; }
            if (y+1 < COVER_H) {
                if (x > 0)         { err[(y+1)*COVER_W+x-1] += qerr*3/16; }
                                     err[(y+1)*COVER_W+x]   += qerr*5/16;
                if (x+1 < COVER_W) { err[(y+1)*COVER_W+x+1] += qerr*1/16; }
            }
        }
    }
    heap_caps_free(err);
    heap_caps_free(gray);

    if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(500)) == pdTRUE) {
        g_cover_dirty = true;
        lv_image_set_src(img_cover, &cover_dsc);
        xSemaphoreGive(lvgl_mtx);
    }

    TrackMeta meta = {};
    strncpy(meta.title,  g_title,  sizeof(meta.title)-1);
    strncpy(meta.artist, g_artist, sizeof(meta.artist)-1);
    meta.has_cover = true;
    // pack cover bits into meta.cover_data
    lv_color32_t *mpal = (lv_color32_t*)meta.cover_data;
    mpal[0] = (lv_color32_t){.blue=0,   .green=0,   .red=0,   .alpha=255};
    mpal[1] = (lv_color32_t){.blue=255, .green=255, .red=255, .alpha=255};
    memcpy(meta.cover_data + 8, bits, COVER_STRIDE * COVER_H);
    sdcard_save_meta(g_cur_file, meta);

    g_meta_needs_save = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Cover decode task (Core 0)
// ─────────────────────────────────────────────────────────────────────────────
static void cover_decode_task(void *)
{
    for (;;) {
        xSemaphoreTake(g_cover_sem, portMAX_DELAY);
        CoverDecodeReq req;
        memcpy(&req, &g_cover_req, sizeof(req));
        g_cover_busy = true;
        if (req.file_len > 0 && req.path[0])
            decode_cover(req.path, req.file_pos, req.file_len);
        g_cover_busy = false;
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
        if (sr == 44100 || sr == 48000 || sr == 32000) {
            codec_set_sample_rate(sr);
            voice_sr_set_input_rate(sr);
            ESP_LOGI(TAG, "Sample rate -> %u Hz", sr);
        }
    }
}

static void on_image(uint32_t file_offset, uint32_t byte_len)
{
    g_art_pos = file_offset;
    g_art_len = byte_len;
}

static void on_eof(void)
{
    g_audio_eof = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// play_track
// ─────────────────────────────────────────────────────────────────────────────
static void play_track(int idx)
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

    const char *path = tracks[idx].c_str();
    strncpy(g_cur_file, path, sizeof(g_cur_file)-1);

    memset(g_title,  0, sizeof(g_title));
    memset(g_artist, 0, sizeof(g_artist));
    memset(g_lyr,    0, sizeof(g_lyr));
    g_art_pos = g_art_len = 0;
    g_cover_dirty = false;
    g_meta_needs_save = false;
    g_track_start_ms  = millis();

    std::string fname(path);
    size_t s = fname.rfind('/'), d = fname.rfind('.');
    std::string base = fname.substr(s + 1, d - s - 1);
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
    // Load cached metadata
    TrackMeta meta = {};
    bool has_cached_cover = sdcard_load_meta(path, meta);
    if (has_cached_cover || strlen(meta.title) > 0) {
        strncpy(g_title,  meta.title,  sizeof(g_title)-1);
        strncpy(g_artist, meta.artist, sizeof(g_artist)-1);
        if (has_cached_cover) {
            memcpy(cover_buf, meta.cover_data, sizeof(cover_buf));
        }
    }

    audio_play(path);

    if (!g_display_fresh) {
        if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(500)) == pdTRUE) {
            refresh_ui();
            if (has_cached_cover) {
                lv_image_set_src(img_cover, &cover_dsc);
            } else {
                load_default_cover();
                lv_image_set_src(img_cover, &cover_dsc);
            }
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
    static uint32_t last_art_check = 0;
    static bool     art_pending    = false;
    static bool     s_was_playing  = true;

    for (;;) {
        // ── Button: BOOT ──
        uint32_t held = 0;
        if (buttons_boot_fired(&held)) {
            uint32_t now = millis();
            if (now - g_last_track_btn_ms > 500) {
                if (held > 1200) {
                    g_last_track_btn_ms = now;
                    if (!g_voice_muted) {
                        g_saved_vol   = codec_get_volume();
                        g_voice_muted = true;
                        codec_dac_mute(true);

                        g_voice_was_paused = !is_playing;
                        if (!is_playing) {
                            is_playing = true;
                            audio_pause_resume();
                            ESP_LOGI(TAG, "[VOICE] Resumed I2S clock for mic");
                        }
                        delay_ms(300);
                    }
                    voice_sr_start_listen();
                    ESP_LOGI(TAG, "[VOICE] Long press — speak command now");
                } else if (held > 50) {
                    is_playing = !is_playing;
                    audio_pause_resume();
                    g_prescan_allowed = !is_playing;
                    if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                        lv_label_set_text(lbl_play_icon,
                            is_playing ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
                        xSemaphoreGive(lvgl_mtx);
                    }
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
                    play_track(cur_track - 1);
                } else {
                    play_track(cur_track + 1);
                }
            }
        }

        // ── Voice commands ──
        {
            int vcmd = voice_sr_get_cmd();
            if (vcmd != VOICE_CMD_NONE) {
                ESP_LOGI(TAG, "[VOICE] Executing command %d", vcmd);
                switch (vcmd) {
                    case VOICE_CMD_NEXT:    play_track(cur_track + 1); break;
                    case VOICE_CMD_PREV:    play_track(cur_track - 1); break;
                    case VOICE_CMD_PAUSE:
                        if (is_playing) {
                            is_playing = false; audio_pause_resume();
                            g_prescan_allowed = true;
                            if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                                lv_label_set_text(lbl_play_icon, LV_SYMBOL_PAUSE);
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
                                xSemaphoreGive(lvgl_mtx);
                            }
                        }
                        break;
                    case VOICE_CMD_VOL_UP:
                        codec_set_volume(std::min(100, (int)codec_get_volume() + 10));
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
                play_track(cur_track + 1);
            }
        } else {
            g_eof_debounce = 0;
        }

        // ── Cover art decode ──
        if (!art_pending && g_art_len > 0 && g_cur_file[0]) {
            art_pending    = true;
            last_art_check = millis();
        }
        if (art_pending && (millis() - last_art_check > 200)) {
            art_pending = false;
            uint32_t pos = g_art_pos, len = g_art_len;
            g_art_pos = g_art_len = 0;
            if (len > 0 && !g_cover_busy) {
                strncpy(g_cover_req.path, g_cur_file, sizeof(g_cover_req.path)-1);
                g_cover_req.file_pos = pos;
                g_cover_req.file_len = len;
                xSemaphoreGive(g_cover_sem);
            }
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

        // ── Periodic UI refresh ──
        uint32_t now = millis();
        if (now - last_ui_tick >= 1000) {
            last_ui_tick = now;
            if (xSemaphoreTake(lvgl_mtx, 0) == pdTRUE) {
                refresh_ui();
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
    bool boot_has_cover = false;
    if (!tracks.empty()) {
        const char *boot_path = tracks[saved_track].c_str();
        strncpy(g_cur_file, boot_path, sizeof(g_cur_file)-1);
        TrackMeta meta = {};
        boot_has_cover = sdcard_load_meta(boot_path, meta);
        if (boot_has_cover || strlen(meta.title) > 0) {
            strncpy(g_title,  meta.title,  sizeof(g_title)-1);
            strncpy(g_artist, meta.artist, sizeof(g_artist)-1);
            if (boot_has_cover) memcpy(cover_buf, meta.cover_data, sizeof(cover_buf));
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
    g_cover_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCoreWithCaps(cover_decode_task, "cover_dec", 16384, nullptr, 2, nullptr, 0,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
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
        if (boot_has_cover) lv_image_set_src(img_cover, &cover_dsc);
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
    audio_set_callbacks(on_meta, on_image, on_eof);
    delay_ms(10);

    if (!codec_init(CODEC_SDA, CODEC_SCL, 400000)) {
        ESP_LOGE(TAG, "ES8311 not found! Check wiring.");
    }
    codec_set_sample_rate(44100);
    codec_set_volume(40);
    codec_enable_mic(true);
    codec_set_mic_gain(7);
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
