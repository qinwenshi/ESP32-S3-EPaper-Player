/*
 * ESP32-S3 e-Paper MP3 Player  (LVGL edition)
 * Board: Waveshare ESP32-S3-ePaper-1.54  (SSD1681, 200×200 B&W)
 *
 * Layout (200×200):
 *  y=  0.. 94  Album art 90×90 (centred)
 *  y= 97..113  Song title  (Cubic-11 14 px, truncated)
 *  y=115..125  Artist  +  time right-aligned  (11 px)
 *  y=127..135  Progress bar
 *  y=136       Horizontal rule
 *  y=139..154  Current lyric line  (14 px, inverted)
 *  y=155..167  Next lyric  (11 px)
 *  y=168..180  Next+1 lyric  (11 px)
 *  y=181       Horizontal rule
 *  y=183..194  Controls guide  (11 px)
 *
 * Libraries:
 *   LVGL 9.2.2, ESP32-audioI2S-master (v3.4.5),
 *   Waveshare epaper_driver_bsp (src/display/)
 */

#include <Arduino.h>
#include <Wire.h>
#include <SD_MMC.h>
#include <Audio.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
#include <esp32s3/rom/tjpgd.h>
#include <lvgl.h>
#include "src/display/epaper_driver_bsp.h"
#include "es8311_lib.h"
#include <vector>
#include <string>
#include "voice_sr.h"

// ── Pins ─────────────────────────────────────────────────────────────────────
#define EPD_MOSI   13
#define EPD_SCK    12
#define EPD_CS     11
#define EPD_DC     10
#define EPD_RST     9
#define EPD_BUSY    8
#define EPD_PWR     6   // LOW = e-paper 3.3 V rail ON

#define SD_CLK     39
#define SD_CMD     41
#define SD_D0      40

#define I2S_MCLK   14
#define I2S_BCLK   15
#define I2S_LRC    38
#define I2S_DOUT   45
#define I2S_DIN    16   // ES8311 ADC → mic input (shared BCLK/WS with I2S1 slave)

#define CODEC_SDA  47
#define CODEC_SCL  48
#define AUDIO_PWR  42   // LOW  = audio analog rail ON  (active LOW!)
#define PA_PIN     46   // HIGH = amplifier ON           (active HIGH)
#define VBAT_LATCH 17

#define BOOT_BTN    0
#define PWR_BTN    18

// ── Display geometry ─────────────────────────────────────────────────────────
#define EPD_W 200
#define EPD_H 200

// ── Layout constants (y positions) ───────────────────────────────────────────
#define ART_SIZE   80
#define ART_X      ((EPD_W - ART_SIZE) / 2)   // 60
#define ART_Y       2

#define TITLE_Y    (ART_Y + ART_SIZE + 3)      // 85
#define ARTIST_Y   (TITLE_Y + 16)              // 101
#define PROG_Y     (ARTIST_Y + 12)             // 113
#define PROG_H     3
#define SEP1_Y     (PROG_Y + PROG_H + 2)       // 118

#define LYR0_Y     (SEP1_Y + 3)               // 121  current lyric
#define LYR1_Y     (LYR0_Y + 16)              // 137  next lyric
#define LYR2_Y     (LYR1_Y + 13)              // 150  next+1

#define SEP2_Y     (LYR2_Y + 13)              // 163
#define CTRL_Y     (SEP2_Y + 2)               // 165

// ── Cover-art buffer (I1 = 1-bit indexed, palette + bitmap) ──────────────────
#define COVER_W     ART_SIZE
#define COVER_H     ART_SIZE
#define COVER_STRIDE ((COVER_W + 7) / 8)       // 12 bytes/row

// 2 palette entries (lv_color32_t = 4 B each) + bitmap rows
static uint8_t cover_buf[2 * 4 + COVER_STRIDE * COVER_H];  // 8 + 1080 = 1088 B

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
static epaper_driver_display *epd   = nullptr;
static Audio                  audio;
static ES8311                 codec;

// ── LVGL ─────────────────────────────────────────────────────────────────────
static SemaphoreHandle_t  lvgl_mtx;
static lv_display_t      *disp       = nullptr;

// ── UI widgets ───────────────────────────────────────────────────────────────
static lv_obj_t *img_cover;
static lv_obj_t *lbl_title;
static lv_obj_t *lbl_artist;
static lv_obj_t *lbl_time;
static lv_obj_t *bar_prog;
static lv_obj_t *lbl_lyr0;  // current (inverted)
static lv_obj_t *lbl_lyr1;  // next
static lv_obj_t *lbl_lyr2;  // next+1
static lv_obj_t *lbl_ctrl;
static lv_obj_t *lbl_play_icon;  // ▶ / ⏸ state icon

// Font forward declarations (defined in font_cubic11_14.c / font_cubic11_11.c)
LV_FONT_DECLARE(font_cubic11_14)
LV_FONT_DECLARE(font_cubic11_11)

// ── Player state ─────────────────────────────────────────────────────────────
static std::vector<String> tracks;
static int     cur_track   = 0;
static bool    is_playing  = false;
static char    g_title[128]  = "No Track";
static char    g_artist[64]  = "";
static char    g_lyr[3][128] = {"","",""};
static bool    g_cover_dirty = false;
static uint32_t g_art_pos = 0, g_art_len = 0;
static char    g_cur_file[256] = "";
static uint32_t g_track_start_ms  = 0;
static bool     g_meta_needs_save = false;
static bool     g_full_refresh_pending = false; // deferred EPD full refresh
static uint32_t g_last_track_btn_ms    = 0;     // debounce for track buttons

// ── Voice command mute management ─────────────────────────────────────────────
// When long-pressing BOOT, we mute the speaker and trigger manual listen mode.
// The speaker is restored once voice_sr_is_listening() returns false.
static bool     g_voice_muted      = false;
static uint8_t  g_saved_vol        = 70;
static bool     g_voice_was_paused = false;  // music was paused when voice activated

// ── NVS playback state ────────────────────────────────────────────────────────
static Preferences  prefs;
static uint32_t     g_restore_pos_sec = 0;   // position to seek to; cleared after seek
static uint32_t     g_last_nvs_save_ms = 0;  // for periodic position save

// ── EPD partial-refresh throttle ─────────────────────────────────────────────
static uint32_t g_last_flush_ms = 0;
static bool     g_silent_flush  = false;  // when true, populate buffer without EPD_DisplayPart
#define MIN_FLUSH_MS 160

// ── EOF auto-advance debounce (file-scope so play_track can reset it) ─────────
static uint32_t g_eof_debounce = 0;

// ── Cover decode task (Core 0) ────────────────────────────────────────────────
struct CoverDecodeReq {
    char     path[256];
    uint32_t file_pos;
    uint32_t file_len;
};
static CoverDecodeReq       g_cover_req;
static SemaphoreHandle_t    g_cover_sem    = nullptr;  // binary: signals Core 0 task
static volatile bool        g_cover_busy   = false;    // Core 0 is decoding

// ── Prescan task (Core 0) ─────────────────────────────────────────────────────
static volatile bool g_prescan_allowed = false; // set true on pause, false on play
static int           g_prescan_idx     = 0;     // next track index to check

// ── Boot display skip ─────────────────────────────────────────────────────────
static bool g_display_fresh = false; // when true, play_track() skips display update

// ─────────────────────────────────────────────────────────────────────────────
// LVGL flush callback  (called from lv_timer_handler, inside lvgl_mtx)
// ─────────────────────────────────────────────────────────────────────────────
static void epd_flush_cb(lv_display_t *d, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t now = millis();
    int32_t wait = (int32_t)(MIN_FLUSH_MS - (now - g_last_flush_ms));
    if (wait > 0) vTaskDelay(pdMS_TO_TICKS(wait));
    g_last_flush_ms = millis();

    // FULL render mode → px_map covers entire EPD_W × EPD_H frame in RGB565
    uint16_t *buf = (uint16_t*)px_map;
    for (int32_t y = 0; y < EPD_H; y++) {
        for (int32_t x = 0; x < EPD_W; x++) {
            uint16_t c = buf[y * EPD_W + x];
            // Simple mid-point threshold: white if > 0x7FFF
            uint8_t col = (c > 0x7FFF) ? DRIVER_COLOR_WHITE : DRIVER_COLOR_BLACK;
            epd->EPD_DrawColorPixel(x, y, col);
        }
    }
    if (!g_silent_flush) {
        epd->EPD_DisplayPart();
    }
    lv_display_flush_ready(d);
}

// ─────────────────────────────────────────────────────────────────────────────
// LVGL tick task  (runs independently, no mutex needed)
// ─────────────────────────────────────────────────────────────────────────────
static void lvgl_tick_task(void*)
{
    for (;;) { vTaskDelay(pdMS_TO_TICKS(2)); lv_tick_inc(2); }
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

    // ── Album art ──
    img_cover = lv_image_create(scr);
    lv_obj_set_size(img_cover, ART_SIZE, ART_SIZE);
    lv_obj_set_pos(img_cover, ART_X, ART_Y);
    lv_obj_set_style_border_width(img_cover, 1, 0);
    lv_obj_set_style_border_color(img_cover, lv_color_black(), 0);
    // placeholder: dark grey fill
    lv_obj_set_style_bg_color(img_cover, lv_color_hex(0x888888), 0);
    lv_obj_set_style_bg_opa(img_cover, LV_OPA_COVER, 0);

    // ── Song title ──
    lbl_title = lv_label_create(scr);
    lv_obj_set_width(lbl_title, EPD_W - 4);
    lv_obj_set_pos(lbl_title, 2, TITLE_Y);
    lv_obj_set_style_text_font(lbl_title, &font_cubic11_14, 0);
    lv_label_set_long_mode(lbl_title, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_title, g_title);

    // ── Artist (left) + time (right) ──
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

    // ── Progress bar ──
    bar_prog = lv_bar_create(scr);
    lv_obj_set_size(bar_prog, EPD_W - 4, PROG_H);
    lv_obj_set_pos(bar_prog, 2, PROG_Y);
    lv_bar_set_range(bar_prog, 0, 1000);
    lv_bar_set_value(bar_prog, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_prog, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_bg_color(bar_prog, lv_color_black(), LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_prog, 2, 0);
    lv_obj_set_style_radius(bar_prog, 2, LV_PART_INDICATOR);

    // ── Separator 1 ──
    lv_obj_t *sep1 = lv_obj_create(scr);
    lv_obj_set_size(sep1, EPD_W, 1);
    lv_obj_set_pos(sep1, 0, SEP1_Y);
    lv_obj_set_style_bg_color(sep1, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(sep1, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sep1, 0, 0);
    lv_obj_clear_flag(sep1, LV_OBJ_FLAG_SCROLLABLE);

    // ── Current lyric (inverted, background only shown when text present) ──
    lbl_lyr0 = lv_label_create(scr);
    lv_obj_set_width(lbl_lyr0, EPD_W - 4);
    lv_obj_set_pos(lbl_lyr0, 2, LYR0_Y);
    lv_obj_set_style_text_font(lbl_lyr0, &font_cubic11_14, 0);
    lv_obj_set_style_bg_color(lbl_lyr0, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lbl_lyr0, LV_OPA_TRANSP, 0);   // hidden until lyrics arrive
    lv_obj_set_style_text_color(lbl_lyr0, lv_color_white(), 0);
    lv_obj_set_style_pad_ver(lbl_lyr0, 1, 0);
    lv_label_set_long_mode(lbl_lyr0, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_lyr0, "");

    // ── Next lyric ──
    lbl_lyr1 = lv_label_create(scr);
    lv_obj_set_width(lbl_lyr1, EPD_W - 4);
    lv_obj_set_pos(lbl_lyr1, 2, LYR1_Y);
    lv_obj_set_style_text_font(lbl_lyr1, &font_cubic11_11, 0);
    lv_label_set_long_mode(lbl_lyr1, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_lyr1, "");

    // ── Next+1 lyric ──
    lbl_lyr2 = lv_label_create(scr);
    lv_obj_set_width(lbl_lyr2, EPD_W - 4);
    lv_obj_set_pos(lbl_lyr2, 2, LYR2_Y);
    lv_obj_set_style_text_font(lbl_lyr2, &font_cubic11_11, 0);
    lv_label_set_long_mode(lbl_lyr2, LV_LABEL_LONG_DOT);
    lv_label_set_text(lbl_lyr2, "");

    // ── Separator 2 ──
    lv_obj_t *sep2 = lv_obj_create(scr);
    lv_obj_set_size(sep2, EPD_W, 1);
    lv_obj_set_pos(sep2, 0, SEP2_Y);
    lv_obj_set_style_bg_color(sep2, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(sep2, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sep2, 0, 0);
    lv_obj_clear_flag(sep2, LV_OBJ_FLAG_SCROLLABLE);

    // ── Controls guide (narrow to leave room for play icon) ──
    lbl_ctrl = lv_label_create(scr);
    lv_obj_set_width(lbl_ctrl, EPD_W - 22);   // leave 20px on right for icon
    lv_obj_set_pos(lbl_ctrl, 2, CTRL_Y);
    lv_obj_set_style_text_font(lbl_ctrl, &font_cubic11_11, 0);
    lv_label_set_text(lbl_ctrl, "BOOT:暂停  PWR:下一首\nBOOT长按:上一首");

    // ── Play / Pause icon (Montserrat-12 symbol font, right-aligned in ctrl area) ──
    lbl_play_icon = lv_label_create(scr);
    lv_obj_set_width(lbl_play_icon, 18);
    lv_obj_set_pos(lbl_play_icon, EPD_W - 20, CTRL_Y + 6);  // vertically centred in ctrl area
    lv_obj_set_style_text_font(lbl_play_icon, &lv_font_montserrat_12, 0);
    lv_label_set_text(lbl_play_icon, LV_SYMBOL_PLAY);
}

// ─────────────────────────────────────────────────────────────────────────────
// Update UI (call with lvgl_mtx held)
// ─────────────────────────────────────────────────────────────────────────────
static void refresh_ui()
{
    lv_label_set_text(lbl_title,  g_title[0] ? g_title : "No Track");
    lv_label_set_text(lbl_artist, g_artist);
    lv_label_set_text(lbl_lyr0,   g_lyr[0]);
    lv_label_set_text(lbl_lyr1,   g_lyr[1]);
    lv_label_set_text(lbl_lyr2,   g_lyr[2]);

    uint32_t cur = audio.getAudioCurrentTime();
    uint32_t tot = audio.getAudioFileDuration();
    char tbuf[24];
    snprintf(tbuf, sizeof(tbuf), "%u:%02u/%u:%02u",
             cur/60, cur%60, tot/60, tot%60);
    lv_label_set_text(lbl_time, tbuf);
    lv_bar_set_value(bar_prog, (tot > 0) ? (int32_t)(cur * 1000 / tot) : 0, LV_ANIM_OFF);

    // Play / pause icon
    lv_label_set_text(lbl_play_icon, is_playing ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
}

// ─────────────────────────────────────────────────────────────────────────────
// JPEG decode → Floyd-Steinberg dither → cover_buf (I1 format)
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t  jpegWork[3100];

struct JpegCtx {
    File    *file;
    uint32_t remaining;
    uint8_t *gray;       // COVER_W × COVER_H grayscale scratch
    int      outW, outH;
};

static UINT jpeg_input(JDEC *jd, BYTE *buf, UINT len)
{
    JpegCtx *ctx = (JpegCtx*)jd->device;
    if (len > ctx->remaining) len = ctx->remaining;
    if (buf) ctx->file->read(buf, len);
    else     ctx->file->seek(ctx->file->position() + len);
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

static void decode_cover(const char *path, uint32_t file_pos, uint32_t file_len)
{
    uint8_t *gray = (uint8_t*)heap_caps_malloc(COVER_W * COVER_H, MALLOC_CAP_SPIRAM);
    if (!gray) return;
    memset(gray, 128, COVER_W * COVER_H);

    File f = SD_MMC.open(path);
    if (!f) { heap_caps_free(gray); return; }

    // Scan for JPEG SOI (0xFF 0xD8) within APIC payload
    f.seek(file_pos);
    uint32_t scanned = 0;
    bool found = false;
    while (scanned + 1 < file_len) {
        uint8_t b = f.read(); scanned++;
        if (b == 0xFF && f.peek() == 0xD8) { f.seek(f.position() - 1); found = true; break; }
    }

    if (found) {
        JpegCtx ctx = { &f, file_len - scanned, gray, COVER_W, COVER_H };
        JDEC jd;
        if (jd_prepare(&jd, jpeg_input, jpegWork, sizeof(jpegWork), &ctx) == JDR_OK) {
            uint8_t scale = 0;
            while (scale < 3 && (jd.width >> scale) > (uint16_t)COVER_W) scale++;
            jd_decomp(&jd, jpeg_output, scale);
        }
    }
    f.close();

    // Floyd-Steinberg dither gray → I1 bitmap
    // Work on a copy so we can propagate errors
    int16_t *err = (int16_t*)heap_caps_malloc(COVER_W * COVER_H * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!err) { heap_caps_free(gray); return; }
    for (int i = 0; i < COVER_W * COVER_H; i++) err[i] = gray[i];

    // Palette: index 0 = black, index 1 = white
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
            if (bit) bits[y*COVER_STRIDE + (x>>3)] |=  (0x80 >> (x&7));
            // Distribute error
            if (x+1 < COVER_W)               err[y*COVER_W+x+1]         += qerr*7/16;
            if (y+1 < COVER_H) {
                if (x > 0)                    err[(y+1)*COVER_W+x-1]     += qerr*3/16;
                                              err[(y+1)*COVER_W+x]       += qerr*5/16;
                if (x+1 < COVER_W)            err[(y+1)*COVER_W+x+1]     += qerr*1/16;
            }
        }
    }
    heap_caps_free(err);
    heap_caps_free(gray);

    // Signal UI update
    if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(500)) == pdTRUE) {
        g_cover_dirty = true;
        lv_image_set_src(img_cover, &cover_dsc);
        xSemaphoreGive(lvgl_mtx);
    }
    // Save full metadata cache (title + artist + cover now all ready)
    g_meta_needs_save = false;
    save_meta_cache(g_cur_file, true);
}

// ─────────────────────────────────────────────────────────────────────────────
// Cover decode task — runs on Core 0, triggered via g_cover_sem
// Keeps Core 1 (audio.loop) free during expensive JPEG + dither work
// ─────────────────────────────────────────────────────────────────────────────
static void cover_decode_task(void *)
{
    for (;;) {
        // Block until a decode request arrives
        xSemaphoreTake(g_cover_sem, portMAX_DELAY);

        // Take a local snapshot of the request (written by Core 1 before Give)
        CoverDecodeReq req;
        memcpy(&req, &g_cover_req, sizeof(req));

        g_cover_busy = true;
        if (req.file_len > 0 && req.path[0]) {
            decode_cover(req.path, req.file_pos, req.file_len);
        }
        g_cover_busy = false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// SD file scan
// ─────────────────────────────────────────────────────────────────────────────
static void scan_tracks()
{
    tracks.clear();
    for (const char *dir : {"/music", "/"}) {
        File root = SD_MMC.open(dir);
        if (!root || !root.isDirectory()) continue;
        File f;
        while ((f = root.openNextFile())) {
            String nm = String(f.name());
            if (!f.isDirectory() && (nm.endsWith(".mp3") || nm.endsWith(".MP3"))
                    && !nm.startsWith("._"))  // skip macOS resource fork files
                tracks.push_back(String(dir) + (String(dir)=="/" ? "" : "/") + nm);
            f.close();
        }
        root.close();
        if (!tracks.empty()) break;
    }
}

// Full EPD refresh then switch to partial mode (NO hardware reset → only 1 visible update)
// ─────────────────────────────────────────────────────────────────────────────
static void epd_full_refresh()
{
    epd->EPD_LoadFullLut();          // switch to full LUT, no hardware reset
    // Render LVGL frame into buffer silently (no EPD_DisplayPart)
    g_silent_flush = true;
    lv_obj_invalidate(lv_screen_active());
    lv_timer_handler();
    g_silent_flush = false;
    // One visible full update: writes buffer to both RAM banks + full LUT trigger
    epd->EPD_DisplayPartBaseImage();
    epd->EPD_ReloadPartialLut();     // back to fast partial mode, no hardware reset
}

// ─────────────────────────────────────────────────────────────────────────────
// EPD frame persistence — save/restore the 1-bit pixel buffer for boot-skip
// ─────────────────────────────────────────────────────────────────────────────
static void epd_save_frame()
{
    uint8_t *tmp = (uint8_t*)heap_caps_malloc(5000, MALLOC_CAP_SPIRAM);
    if (!tmp) return;
    epd->EPD_GetBuffer(tmp);
    File f = SD_MMC.open("/.epd_frame", FILE_WRITE);
    if (f) { f.write(tmp, 5000); f.close(); }
    heap_caps_free(tmp);
}

// Returns true if a saved frame was found and loaded into EPD hardware RAM banks
static bool epd_restore_frame()
{
    uint8_t *tmp = (uint8_t*)heap_caps_malloc(5000, MALLOC_CAP_SPIRAM);
    if (!tmp) return false;
    File f = SD_MMC.open("/.epd_frame", FILE_READ);
    bool ok = (f && f.read(tmp, 5000) == 5000);
    if (f) f.close();
    if (ok) epd->EPD_WriteFrameToRAMSilent(tmp);
    heap_caps_free(tmp);
    return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// ID3v2 text-only parser — reads TIT2/TPE1, skips APIC (handles v2.3 + v2.4)
// ─────────────────────────────────────────────────────────────────────────────
static bool parse_id3_text(const char *mp3_path,
                             char *out_title,  size_t title_sz,
                             char *out_artist, size_t artist_sz)
{
    File f = SD_MMC.open(mp3_path, FILE_READ);
    if (!f) return false;
    uint8_t hdr[10];
    if (f.read(hdr, 10) != 10 || memcmp(hdr, "ID3", 3) != 0) { f.close(); return false; }
    uint8_t  ver    = hdr[3];
    uint8_t  flags  = hdr[5];
    uint32_t tag_sz = ((uint32_t)(hdr[6]&0x7F)<<21)|((uint32_t)(hdr[7]&0x7F)<<14)
                     |((uint32_t)(hdr[8]&0x7F)<<7) | (uint32_t)(hdr[9]&0x7F);
    if (flags & 0x40) {   // skip extended header
        uint8_t eh[4];
        if (f.read(eh, 4) == 4) {
            uint32_t ehsz = ((uint32_t)eh[0]<<24)|((uint32_t)eh[1]<<16)|((uint32_t)eh[2]<<8)|eh[3];
            if (ehsz > 4) f.seek(f.position() + ehsz - 4);
        }
    }
    bool got_title = false, got_artist = false;
    uint32_t read = 0;
    while (read < tag_sz && !(got_title && got_artist)) {
        uint8_t fhdr[10];
        if (f.read(fhdr, 10) != 10) break;
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
            uint8_t enc = 0; f.read(&enc, 1);
            uint32_t tlen = fsz - 1;
            char  *dst = is_t ? out_title  : out_artist;
            size_t dsz = is_t ? title_sz   : artist_sz;
            if (enc == 0 || enc == 3) {  // ISO-8859-1 or UTF-8
                uint32_t r = min(tlen, (uint32_t)(dsz - 1));
                f.read((uint8_t*)dst, r); dst[r] = '\0';
                if (tlen > r) f.seek(f.position() + tlen - r);
            } else {
                f.seek(f.position() + tlen);  // skip UTF-16
            }
            if (is_t) got_title  = true;
            if (is_a) got_artist = true;
        } else {
            if (!f.seek(f.position() + fsz)) break;
        }
        read += fsz;
    }
    f.close();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Meta prescan task (Core 0) — builds .meta text files while paused
// Stops immediately when g_prescan_allowed becomes false
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
            int        idx = (g_prescan_idx + i) % total;
            const char *mp3 = tracks[idx].c_str();
            if (!strcmp(mp3, g_cur_file)) continue;  // skip currently playing

            // Check if valid .meta already exists
            String mpath = meta_cache_path(String(mp3));
            {
                File chk = SD_MMC.open(mpath.c_str(), FILE_READ);
                if (chk) {
                    char magic[4] = {};
                    bool ok = (chk.read((uint8_t*)magic, 4) == 4 && !memcmp(magic, "META", 4));
                    chk.close();
                    if (ok) continue;
                }
            }
            if (!g_prescan_allowed) break;

            // Filename fallback (Artist - Title convention)
            char title[128] = {}, artist[64] = {};
            {
                String fname = String(mp3);
                int s = fname.lastIndexOf('/'), d = fname.lastIndexOf('.');
                String base = fname.substring(s + 1, d);
                int sep = base.indexOf(" - ");
                if (sep > 0) {
                    strlcpy(artist, base.substring(0, sep).c_str(), sizeof(artist));
                    strlcpy(title,  base.substring(sep + 3).c_str(), sizeof(title));
                } else {
                    strlcpy(title, base.c_str(), sizeof(title));
                }
            }

            // Try ID3v2 text tags (may update title/artist above)
            parse_id3_text(mp3, title, sizeof(title), artist, sizeof(artist));
            if (!g_prescan_allowed) break;

            // Write minimal .meta (no cover pixel data)
            File f = SD_MMC.open(mpath.c_str(), FILE_WRITE);
            if (f) {
                f.write((const uint8_t*)"META", 4);
                f.write((uint8_t*)title,  128);
                f.write((uint8_t*)artist,  64);
                f.write((uint8_t)0);  // has_cover = false
                f.close();
                Serial.printf("[prescan] %s — %s\n", artist, title);
            }
            g_prescan_idx = (idx + 1) % total;
            did_work = true;
            vTaskDelay(pdMS_TO_TICKS(50));  // yield between files
            break;  // recheck g_prescan_allowed next outer iteration
        }
        if (!did_work) {
            g_prescan_idx = 0;
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Metadata cache (.meta files stored next to each .mp3 on SD)
// Binary format: "META" + title[128] + artist[64] + has_cover(1) + cover_bits[COVER_STRIDE*COVER_H]
// ─────────────────────────────────────────────────────────────────────────────
static String meta_cache_path(const String &mp3) {
    int dot = mp3.lastIndexOf('.');
    return (dot > 0 ? mp3.substring(0, dot) : mp3) + ".meta";
}

static bool load_meta_cache(const char *mp3_path) {
    String mp = meta_cache_path(String(mp3_path));
    File f = SD_MMC.open(mp.c_str(), FILE_READ);
    if (!f) return false;
    char magic[4];
    if (f.read((uint8_t*)magic, 4) != 4 || memcmp(magic, "META", 4) != 0) {
        f.close(); return false;
    }
    f.read((uint8_t*)g_title,  sizeof(g_title));
    f.read((uint8_t*)g_artist, sizeof(g_artist));
    uint8_t has_cover = 0;
    f.read(&has_cover, 1);
    if (has_cover) {
        // Reconstruct palette (always black/white)
        lv_color32_t *pal = (lv_color32_t*)cover_buf;
        pal[0] = (lv_color32_t){.blue=0,   .green=0,   .red=0,   .alpha=255};
        pal[1] = (lv_color32_t){.blue=255, .green=255, .red=255, .alpha=255};
        uint8_t *bits = cover_buf + 2 * sizeof(lv_color32_t);
        f.read(bits, COVER_STRIDE * COVER_H);
    }
    f.close();
    return has_cover > 0;
}

static void save_meta_cache(const char *mp3_path, bool has_cover) {
    String mp = meta_cache_path(String(mp3_path));
    File f = SD_MMC.open(mp.c_str(), FILE_WRITE);
    if (!f) return;
    f.write((const uint8_t*)"META", 4);
    f.write((uint8_t*)g_title,  sizeof(g_title));
    f.write((uint8_t*)g_artist, sizeof(g_artist));
    f.write((uint8_t)(has_cover ? 1 : 0));
    if (has_cover) {
        uint8_t *bits = cover_buf + 2 * sizeof(lv_color32_t);
        f.write(bits, COVER_STRIDE * COVER_H);
    }
    f.close();
    Serial.printf("Meta cache saved: %s\n", mp.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Start a track
// ─────────────────────────────────────────────────────────────────────────────
static void play_track(int idx)
{
    if (tracks.empty()) return;
    idx = ((idx % (int)tracks.size()) + (int)tracks.size()) % (int)tracks.size();
    cur_track  = idx;
    is_playing = true;

    // Save track index to NVS; reset stored position (user chose a new track)
    prefs.putInt("track", idx);
    prefs.putUInt("pos", 0);
    g_restore_pos_sec = 0;   // cancel any pending seek from startup restore
    g_eof_debounce    = 0;   // prevent stale debounce from firing on new track
    g_prescan_allowed = false;  // stop background prescan while playing

    const char *path = tracks[idx].c_str();
    strlcpy(g_cur_file, path, sizeof(g_cur_file));

    // Reset display state
    memset(g_title,  0, sizeof(g_title));
    memset(g_artist, 0, sizeof(g_artist));
    memset(g_lyr,    0, sizeof(g_lyr));
    g_art_pos = g_art_len = 0;
    g_cover_dirty = false;
    g_meta_needs_save = false;
    g_track_start_ms  = millis();

    // Filename as fallback — try to split "Artist - Title" convention
    String fname = tracks[idx];
    int s = fname.lastIndexOf('/'), d = fname.lastIndexOf('.');
    String base = fname.substring(s + 1, d);
    int sep = base.indexOf(" - ");
    if (sep > 0) {
        strlcpy(g_artist, base.substring(0, sep).c_str(), sizeof(g_artist));
        strlcpy(g_title,  base.substring(sep + 3).c_str(), sizeof(g_title));
    } else {
        strlcpy(g_title, base.c_str(), sizeof(g_title));
    }

    // Stop current audio and wait for the audio task to fully release SD
    audio.stopSong();
    uint32_t stop_t = millis();
    while (audio.isRunning() && millis() - stop_t < 300) {
        audio.loop();   // drain the audio task's stop sequence
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Try loading cached metadata first — instant display
    bool has_cached_cover = load_meta_cache(path);

    audio.connecttoFS(SD_MMC, path);

    // Update LVGL state (fast — no EPD operations here)
    // Skip on first boot: setup() already rendered the correct state
    if (!g_display_fresh) {
        if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(500)) == pdTRUE) {
            refresh_ui();
            if (has_cached_cover) {
                lv_image_set_src(img_cover, &cover_dsc);
            } else {
                lv_image_set_src(img_cover, NULL);
                lv_obj_set_style_bg_color(img_cover, lv_color_hex(0x888888), 0);
                lv_obj_set_style_bg_opa(img_cover, LV_OPA_COVER, 0);
            }
            xSemaphoreGive(lvgl_mtx);
        }
        // Defer EPD full refresh to loop() so audio.loop() can run first
        g_full_refresh_pending = true;
    }
    g_display_fresh = false;  // consume — subsequent calls always update display
}

// ─────────────────────────────────────────────────────────────────────────────
// Audio callback (v3.4.5 lambda API)
// ─────────────────────────────────────────────────────────────────────────────
static void setup_audio_callbacks()
{
    Audio::audio_info_callback = [](Audio::msg_t m) {
        switch (m.e) {

        case Audio::evt_id3data:
            if (m.msg) {
                if (strncmp(m.msg, "Title: ", 7) == 0) {
                    xSemaphoreTake(lvgl_mtx, portMAX_DELAY);
                    strlcpy(g_title, m.msg + 7, sizeof(g_title));
                    lv_label_set_text(lbl_title, g_title);
                    xSemaphoreGive(lvgl_mtx);
                    g_meta_needs_save = true;
                } else if (strncmp(m.msg, "Artist: ", 8) == 0) {
                    xSemaphoreTake(lvgl_mtx, portMAX_DELAY);
                    strlcpy(g_artist, m.msg + 8, sizeof(g_artist));
                    lv_label_set_text(lbl_artist, g_artist);
                    xSemaphoreGive(lvgl_mtx);
                    g_meta_needs_save = true;
                } else if (strncmp(m.msg, "SampleRate: ", 12) == 0) {
                    uint32_t sr = atoi(m.msg + 12);
                    if (sr == 44100 || sr == 48000 || sr == 32000) {
                        codec.setSampleRate(sr);
                        voice_sr_set_input_rate(sr);  // keep voice resampler in sync
                        Serial.printf("Codec sample rate -> %u Hz\n", sr);
                    }
                }
            }
            break;

        case Audio::evt_image:
            // vec = { file_offset, byte_length }
            if (m.vec.size() >= 2) {
                g_art_pos = m.vec[0];
                g_art_len = m.vec[1];
                // Decode on next loop iteration to avoid blocking audio task
            }
            break;

        case Audio::evt_lyrics:
            if (m.msg) {
                xSemaphoreTake(lvgl_mtx, portMAX_DELAY);
                strlcpy(g_lyr[2], g_lyr[1], sizeof(g_lyr[2]));
                strlcpy(g_lyr[1], g_lyr[0], sizeof(g_lyr[1]));
                strlcpy(g_lyr[0], m.msg,    sizeof(g_lyr[0]));
                lv_label_set_text(lbl_lyr0, g_lyr[0]);
                lv_label_set_text(lbl_lyr1, g_lyr[1]);
                lv_label_set_text(lbl_lyr2, g_lyr[2]);
                // Show inverted background only when there's a current lyric
                lv_obj_set_style_bg_opa(lbl_lyr0,
                    (g_lyr[0][0] != '\0') ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
                xSemaphoreGive(lvgl_mtx);
            }
            break;

        case Audio::evt_eof:
            is_playing = false;
            break;

        default: break;
        }
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// Buttons  (interrupt-safe flags)
// ─────────────────────────────────────────────────────────────────────────────
static volatile bool     g_boot_flag   = false;
static volatile bool     g_pwr_flag    = false;
static volatile uint32_t g_boot_down   = 0;   // time of press
static volatile uint32_t g_boot_held   = 0;   // duration (set on release)
static volatile uint32_t g_pwr_down    = 0;
static volatile uint32_t g_pwr_held    = 0;

void IRAM_ATTR isr_boot() {
    if (digitalRead(BOOT_BTN) == LOW) {
        g_boot_down = millis();
    } else {
        g_boot_held = millis() - g_boot_down;
        g_boot_flag = true;
    }
}
void IRAM_ATTR isr_pwr() {
    if (digitalRead(PWR_BTN) == LOW) {
        g_pwr_down = millis();
    } else {
        g_pwr_held = millis() - g_pwr_down;
        g_pwr_flag = true;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);

    // Power rails
    pinMode(VBAT_LATCH, OUTPUT); digitalWrite(VBAT_LATCH, HIGH);
    pinMode(EPD_PWR,    OUTPUT); digitalWrite(EPD_PWR,    LOW);   // e-paper ON  (active LOW)
    pinMode(AUDIO_PWR,  OUTPUT); digitalWrite(AUDIO_PWR,  LOW);   // audio ON    (active LOW!)
    pinMode(PA_PIN,     OUTPUT); digitalWrite(PA_PIN,     HIGH);  // amplifier ON (active HIGH)
    delay(50);

    // ── NVS ──
    prefs.begin("player", false);  // r/w namespace

    // ── e-Paper driver ──
    custom_lcd_spi_t spi_cfg = {};
    spi_cfg.cs         = EPD_CS;
    spi_cfg.dc         = EPD_DC;
    spi_cfg.rst        = EPD_RST;
    spi_cfg.busy       = EPD_BUSY;
    spi_cfg.mosi       = EPD_MOSI;
    spi_cfg.scl        = EPD_SCK;
    spi_cfg.spi_host   = SPI2_HOST;
    spi_cfg.buffer_len = (EPD_W * EPD_H) / 8;   // 5000 bytes
    epd = new epaper_driver_display(EPD_W, EPD_H, spi_cfg);
    epd->EPD_Init();
    // NOTE: no EPD_Clear() here — we try to restore the saved frame instead

    // ── SD card (moved up — needed for saved frame + meta preload) ──
    SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
    if (SD_MMC.begin("/sdcard", true /*1-bit*/)) {
        scan_tracks();
        Serial.printf("Found %d tracks\n", (int)tracks.size());
    } else {
        Serial.println("SD mount failed");
    }

    // ── NVS: restore saved track/position ──
    int      saved_track = prefs.getInt("track", 0);
    uint32_t saved_pos   = prefs.getUInt("pos",   0);
    if (saved_track < 0 || saved_track >= (int)tracks.size()) saved_track = 0;

    // ── Pre-load title/artist for the saved track so the boot render shows it ──
    bool boot_has_cover = false;
    if (!tracks.empty()) {
        const char *boot_path = tracks[saved_track].c_str();
        strlcpy(g_cur_file, boot_path, sizeof(g_cur_file));
        boot_has_cover = load_meta_cache(boot_path);
        if (!g_title[0]) {
            String fname = String(boot_path);
            int s = fname.lastIndexOf('/'), d = fname.lastIndexOf('.');
            String base = fname.substring(s + 1, d);
            int sep = base.indexOf(" - ");
            if (sep > 0) {
                strlcpy(g_artist, base.substring(0, sep).c_str(), sizeof(g_artist));
                strlcpy(g_title,  base.substring(sep + 3).c_str(), sizeof(g_title));
            } else {
                strlcpy(g_title, base.c_str(), sizeof(g_title));
            }
        }
    }

    // ── Try to restore last EPD frame (prevents full flash on every boot) ──
    bool has_saved_frame = epd_restore_frame();
    if (!has_saved_frame) epd->EPD_Clear();

    // ── LVGL ──
    lv_init();
    lvgl_mtx = xSemaphoreCreateMutex();
    // LVGL tick + cover decode + prescan all run on Core 0, freeing Core 1 for audio.loop()
    xTaskCreatePinnedToCore(lvgl_tick_task,    "lv_tick",  1024,  nullptr, 5, nullptr, 0);
    g_cover_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(cover_decode_task, "cover_dec", 8192, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(meta_prescan_task, "prescan",   4096, nullptr, 1, nullptr, 0);

    disp = lv_display_create(EPD_W, EPD_H);
    uint8_t *lvgl_buf = (uint8_t*)heap_caps_malloc(
        EPD_W * EPD_H * 2, MALLOC_CAP_SPIRAM);
    configASSERT(lvgl_buf);
    lv_display_set_buffers(disp, lvgl_buf, nullptr, EPD_W * EPD_H * 2,
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(disp, epd_flush_cb);

    // ── Build UI ──
    build_screen();

    // Populate widgets with pre-loaded track state before first render
    if (xSemaphoreTake(lvgl_mtx, portMAX_DELAY) == pdTRUE) {
        refresh_ui();
        if (boot_has_cover) lv_image_set_src(img_cover, &cover_dsc);
        xSemaphoreGive(lvgl_mtx);
    }

    // Render silently into EPD internal buffer (no EPD_DisplayPart inside flush cb)
    g_silent_flush = true;
    lv_obj_invalidate(lv_screen_active());
    lv_timer_handler();
    g_silent_flush = false;

    // Boot EPD update: partial-only if saved frame available, full refresh otherwise
    if (has_saved_frame) {
        // EPD_ReloadPartialLut() does NOT do a hardware reset — 0x26 RAM stays intact
        epd->EPD_ReloadPartialLut();
        epd->EPD_DisplayPart();   // only changed pixels vs saved frame
    } else {
        epd->EPD_DisplayPartBaseImage();  // first boot: full refresh + set base image
        epd->EPD_Init_Partial();
    }
    epd_save_frame();  // persist for next boot

    // ── Codec + audio ──
    // IMPORTANT: setPinout() must come FIRST — it starts I2S and generates MCLK on GPIO14.
    // ES8311 uses MCLK for its PLL; init without MCLK means clock dividers don't lock.
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT, I2S_MCLK);
    audio.setVolume(21);         // max software volume — use codec for level control
    delay(10);                   // let MCLK stabilise before codec init

    if (!codec.begin(CODEC_SDA, CODEC_SCL, 400000)) {
        Serial.println("ES8311 not found! Check wiring.");
    }
    codec.setSampleRate(44100);  // override default 48k; reconfigures for 44.1kHz MCLK
    codec.setVolume(70);         // ~70% (reg32=0xB2, near 0dB) — adjust as needed
    codec.enableMicrophone(true);  // enable ES8311 ADC path for voice recognition
    codec.setMicrophoneGain(7);    // max PGA gain (0-7)
    // Print register dump to confirm codec state
    Serial.println("=== ES8311 registers ===");
    codec.read_all();
    Serial.println("========================");
    setup_audio_callbacks();

    // ── Voice recognition (wake word "Hi,喵喵" + Chinese commands) ──
    voice_sr_init();

    // ── Buttons ──
    pinMode(BOOT_BTN, INPUT_PULLUP);
    pinMode(PWR_BTN,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BOOT_BTN), isr_boot, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PWR_BTN),  isr_pwr,  CHANGE);

    // ── Auto-play: start saved track (display already rendered above) ──
    if (!tracks.empty()) {
        g_display_fresh = true;     // tell play_track() to skip display update
        play_track(saved_track);    // starts audio; saves track=saved_track, pos=0 to NVS

        // Restore position: seek as soon as audio.isRunning() in loop()
        if (saved_pos > 0) {
            g_restore_pos_sec = saved_pos;
        }
    } else {
        xSemaphoreTake(lvgl_mtx, portMAX_DELAY);
        lv_label_set_text(lbl_title, "SD空/无MP3");
        xSemaphoreGive(lvgl_mtx);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
static uint32_t last_ui_tick   = 0;
static uint32_t last_art_check = 0;
static bool     art_pending    = false;

void loop()
{
    audio.loop();

    // ── Button: BOOT ──
    if (g_boot_flag) {
        g_boot_flag = false;
        uint32_t held = g_boot_held;
        uint32_t now  = millis();
        if (now - g_last_track_btn_ms > 500) {   // 500ms cooldown
            if (held > 1200) {
                // Long press (>1.2s) → voice command mode
                // Mute speaker so mic only hears user's voice (no music echo)
                g_last_track_btn_ms = now;
                if (!g_voice_muted) {
                    g_saved_vol   = codec.getVolume();
                    g_voice_muted = true;
                    codec.dacMute(true);   // cut DAC→ADC coupling at digital stage

                    // I2S1 (mic) is a slave — it needs I2S0's BCLK/WS to work.
                    // If playback is paused, I2S0 clock is dead → mic reads zeros.
                    // Temporarily resume at 0 volume to keep the clock alive.
                    g_voice_was_paused = !is_playing;
                    if (!is_playing) {
                        is_playing = true;
                        audio.pauseResume();
                        Serial.println("[VOICE] Resumed I2S clock for mic");
                    }
                    delay(300);   // let DAC mute settle + allow frame-phase detection
                }
                voice_sr_start_listen();
                Serial.println("[VOICE] Long press — speak command now");
            } else if (held > 50) {
                // Short press → play/pause
                is_playing = !is_playing;
                audio.pauseResume();
                g_prescan_allowed = !is_playing;
                if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                    lv_label_set_text(lbl_play_icon,
                        is_playing ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
                    xSemaphoreGive(lvgl_mtx);
                }
            }
        }
    }

    // ── Restore volume after voice command window closes ──
    if (g_voice_muted && !voice_sr_is_listening()) {
        codec.dacMute(false);          // re-enable DAC
        codec.setVolume(g_saved_vol);  // restore to pre-voice volume
        g_voice_muted = false;
        // Re-pause if music was paused before voice activation
        if (g_voice_was_paused) {
            audio.pauseResume();
            is_playing = false;
            g_voice_was_paused = false;
            if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                lv_label_set_text(lbl_play_icon, LV_SYMBOL_PAUSE);
                xSemaphoreGive(lvgl_mtx);
            }
        }
        Serial.println("[VOICE] DAC restored");
    }

    // ── Button: PWR ──
    if (g_pwr_flag) {
        g_pwr_flag = false;
        uint32_t now = millis();
        if (now - g_last_track_btn_ms > 500) {
            g_last_track_btn_ms = now;
            if (g_pwr_held > 800) {
                play_track(cur_track - 1);   // long press → previous track
            } else {
                play_track(cur_track + 1);   // short press → next track
            }
        }
    }

    // ── Voice commands (wake word: "你好小智") ──
    {
        int vcmd = voice_sr_get_cmd();
        if (vcmd != VOICE_CMD_NONE) {
            Serial.printf("[VOICE] Executing command %d\n", vcmd);
            switch (vcmd) {
                case VOICE_CMD_NEXT:
                    play_track(cur_track + 1);
                    break;
                case VOICE_CMD_PREV:
                    play_track(cur_track - 1);
                    break;
                case VOICE_CMD_PAUSE:
                    if (is_playing) {
                        is_playing = false;
                        audio.pauseResume();
                        g_prescan_allowed = true;
                        if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                            lv_label_set_text(lbl_play_icon, LV_SYMBOL_PAUSE);
                            xSemaphoreGive(lvgl_mtx);
                        }
                    }
                    break;
                case VOICE_CMD_RESUME:
                    if (!is_playing) {
                        is_playing = true;
                        audio.pauseResume();
                        g_prescan_allowed = false;
                        if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
                            lv_label_set_text(lbl_play_icon, LV_SYMBOL_PLAY);
                            xSemaphoreGive(lvgl_mtx);
                        }
                    }
                    break;
                case VOICE_CMD_VOL_UP:
                    codec.setVolume(min(100, (int)codec.getVolume() + 10));
                    break;
                case VOICE_CMD_VOL_DOWN:
                    codec.setVolume(max(0, (int)codec.getVolume() - 10));
                    break;
                default:
                    break;
            }
        }
    }

    // ── Deferred EPD full refresh (runs after audio.loop() had a chance) ──
    if (g_full_refresh_pending) {
        g_full_refresh_pending = false;
        last_ui_tick = millis();                 // reset periodic timer so it doesn't fire right after
        epd_full_refresh();
        epd_save_frame();                        // persist frame for boot-skip on next power-on
        audio.loop();                            // pump audio events that queued during the 2s EPD block
        return;                                  // skip rest of loop this iteration
    }

    // ── Restore position from NVS: seek as soon as audio is running ──
    if (g_restore_pos_sec > 0 && audio.isRunning()) {
        Serial.printf("Restoring position to %us\n", g_restore_pos_sec);
        audio.setAudioPlayTime(g_restore_pos_sec);
        g_restore_pos_sec = 0;
    }

    // ── Periodic NVS position save (every 3 s while playing) ──
    {
        uint32_t now = millis();
        if (is_playing && audio.isRunning() && now - g_last_nvs_save_ms > 3000) {
            g_last_nvs_save_ms = now;
            prefs.putUInt("pos", audio.getAudioCurrentTime());
        }
    }

    // ── Auto-advance on EOF ──
    if (is_playing && !tracks.empty() && !audio.isRunning()) {
        if (g_eof_debounce == 0) g_eof_debounce = millis();
        if (millis() - g_eof_debounce > 800) {
            g_eof_debounce = 0;
            play_track(cur_track + 1);
        }
    } else {
        g_eof_debounce = 0;  // reset whenever audio is running or paused normally
    }

    // ── Cover art decode — signal Core 0 task (deferred 200ms after callback) ──
    if (!art_pending && g_art_len > 0 && g_cur_file[0]) {
        art_pending    = true;
        last_art_check = millis();
    }
    if (art_pending && (millis() - last_art_check > 200)) {
        art_pending = false;
        uint32_t pos = g_art_pos, len = g_art_len;
        g_art_pos = g_art_len = 0;
        if (len > 0 && !g_cover_busy) {
            // Copy request then release Core 0 task
            strlcpy(g_cover_req.path, g_cur_file, sizeof(g_cover_req.path));
            g_cover_req.file_pos = pos;
            g_cover_req.file_len = len;
            xSemaphoreGive(g_cover_sem);
        }
    }

    // ── Save metadata cache for no-cover tracks (after 8s, ID3 settled) ──
    if (g_meta_needs_save && g_track_start_ms > 0 &&
        millis() - g_track_start_ms > 8000) {
        g_meta_needs_save = false;
        save_meta_cache(g_cur_file, false);
    }

    // ── Periodic UI refresh (every second) ──
    uint32_t now = millis();
    if (now - last_ui_tick >= 1000) {
        last_ui_tick = now;
        if (xSemaphoreTake(lvgl_mtx, 0) == pdTRUE) {
            refresh_ui();
            xSemaphoreGive(lvgl_mtx);
        }
    }

    // ── LVGL handler ──
    if (xSemaphoreTake(lvgl_mtx, pdMS_TO_TICKS(20)) == pdTRUE) {
        lv_timer_handler();
        xSemaphoreGive(lvgl_mtx);
    }
}
