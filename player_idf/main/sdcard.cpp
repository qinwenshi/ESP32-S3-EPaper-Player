// sdcard.cpp — SD card via SDMMC (1-bit) + VFS FAT (ESP-IDF v5.x)
#include "sdcard.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <dirent.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

static const char *TAG = "sdcard";

// Pointer to the mounted card — kept so we can unmount cleanly.
static sdmmc_card_t *s_card = nullptr;

// ── Mount / Unmount ───────────────────────────────────────────────────────────

bool sdcard_mount(int clk, int cmd, int d0)
{
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED; // 40 MHz

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk   = (gpio_num_t)clk;
    slot.cmd   = (gpio_num_t)cmd;
    slot.d0    = (gpio_num_t)d0;
    slot.width = 1; // 1-bit mode
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 8,
        .allocation_unit_size   = 16 * 1024,
    };

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot,
                                             &mount_cfg, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(ret));
        return false;
    }

    sdmmc_card_print_info(stdout, s_card);
    return true;
}

void sdcard_unmount(void)
{
    if (s_card) {
        esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
        s_card = nullptr;
    }
}

// ── Track scan ────────────────────────────────────────────────────────────────

// Returns true if the filename ends with ".mp3" (case-insensitive).
static bool is_mp3(const char *name)
{
    size_t len = strlen(name);
    if (len < 4) return false;
    const char *ext = name + len - 4;
    return (strcasecmp(ext, ".mp3") == 0);
}

// Recursively collect .mp3 paths under `dir_path` into `out`.
static void scan_dir(const char *dir_path, std::vector<std::string> &out)
{
    DIR *dir = opendir(dir_path);
    if (!dir) return;

    struct dirent *ent;
    while ((ent = readdir(dir)) != nullptr) {
        // Skip hidden / macOS resource-fork files
        if (ent->d_name[0] == '.') continue;
        if (strncmp(ent->d_name, "._", 2) == 0) continue;

        char full[512];
        snprintf(full, sizeof(full), "%s/%s", dir_path, ent->d_name);

        if (ent->d_type == DT_DIR) {
            scan_dir(full, out);
        } else if (ent->d_type == DT_REG || ent->d_type == DT_UNKNOWN) {
            // DT_UNKNOWN: fall back to stat to check if regular file + .mp3
            if (is_mp3(ent->d_name)) {
                if (ent->d_type == DT_UNKNOWN) {
                    struct stat st;
                    if (stat(full, &st) != 0 || !S_ISREG(st.st_mode)) continue;
                }
                out.push_back(std::string(full));
            }
        }
    }
    closedir(dir);
}

void sdcard_scan_tracks(std::vector<std::string> &out)
{
    out.clear();

    // Prefer /sdcard/music; fall back to /sdcard root.
    scan_dir("/sdcard/music", out);
    if (out.empty()) scan_dir("/sdcard", out);

    std::sort(out.begin(), out.end());
    ESP_LOGI(TAG, "Found %zu tracks", out.size());
}

// ── Meta cache ────────────────────────────────────────────────────────────────

// Binary layout:
//   [0..3]   magic "META"
//   [4..131] title[128]
//   [132..195] artist[64]
//   [196]    has_cover (0 or 1)
//   [197..]  cover_data[8 + COVER_STRIDE*COVER_H] — only when has_cover==1
static const char  META_MAGIC[4]     = {'M','E','T','A'};
static const size_t META_HEADER_SIZE  = 4 + 128 + 64 + 1;          // 197
static const size_t META_COVER_SIZE   = 8 + (10 * 80);             // 808
static const size_t META_FULL_SIZE    = META_HEADER_SIZE + META_COVER_SIZE; // 1005

// Build .meta path from .mp3 path.
static void meta_path(const char *mp3_path, char *out, size_t out_sz)
{
    strncpy(out, mp3_path, out_sz - 1);
    out[out_sz - 1] = '\0';

    // Replace final extension with ".meta"
    char *dot = strrchr(out, '.');
    if (dot) {
        strncpy(dot, ".meta", out_sz - (size_t)(dot - out) - 1);
    } else {
        strncat(out, ".meta", out_sz - strlen(out) - 1);
    }
}

bool sdcard_load_meta(const char *mp3_path, TrackMeta &out)
{
    char path[512];
    meta_path(mp3_path, path, sizeof(path));

    FILE *f = fopen(path, "rb");
    if (!f) return false;

    uint8_t magic[4];
    if (fread(magic, 1, 4, f) != 4 ||
        memcmp(magic, META_MAGIC, 4) != 0) {
        fclose(f);
        return false;
    }

    if (fread(out.title,  1, 128, f) != 128 ||
        fread(out.artist, 1, 64,  f) != 64) {
        fclose(f);
        return false;
    }

    uint8_t hc = 0;
    if (fread(&hc, 1, 1, f) != 1) { fclose(f); return false; }
    out.has_cover = (hc != 0);

    if (out.has_cover) {
        if (fread(out.cover_data, 1, META_COVER_SIZE, f) != META_COVER_SIZE) {
            out.has_cover = false;
        }
    }

    fclose(f);
    return true;
}

void sdcard_save_meta(const char *mp3_path, const TrackMeta &meta)
{
    char path[512];
    meta_path(mp3_path, path, sizeof(path));

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot write meta: %s", path);
        return;
    }

    fwrite(META_MAGIC,    1, 4,   f);
    fwrite(meta.title,    1, 128, f);
    fwrite(meta.artist,   1, 64,  f);
    uint8_t hc = meta.has_cover ? 1 : 0;
    fwrite(&hc,           1, 1,   f);
    if (meta.has_cover) {
        fwrite(meta.cover_data, 1, META_COVER_SIZE, f);
    }

    fclose(f);
}

// ── Raw file I/O ──────────────────────────────────────────────────────────────

bool sdcard_write_file(const char *path, const uint8_t *data, size_t len)
{
    FILE *f = fopen(path, "wb");
    if (!f) return false;
    size_t written = fwrite(data, 1, len, f);
    fclose(f);
    return (written == len);
}

bool sdcard_read_file(const char *path, uint8_t *buf, size_t len)
{
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    bool ok = (fread(buf, 1, len, f) == len);
    fclose(f);
    return ok;
}
