// sdcard.h — SD card via SDMMC + VFS FAT
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

// Mount SD card (1-bit SDMMC mode).  Returns true on success.
// After mount, files accessible under /sdcard/
bool sdcard_mount(int clk, int cmd, int d0);
void sdcard_unmount(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// C++ helpers (scan, meta cache)

// Recursively scan /sdcard/music for .mp3 files
void sdcard_scan_tracks(std::vector<std::string> &out);

struct TrackMeta {
    char title[128];
    char artist[64];
    bool has_cover;
    // cover pixel data (1-bit LVGL I1 format): 8 palette bytes + stride*height bits
    // Only valid if has_cover == true
    uint8_t cover_data[8 + (10 * 80)]; // worst case 80×80
};
// Load .meta sidecar file.  Returns true if found and valid.
bool sdcard_load_meta(const char *mp3_path, TrackMeta &out);
// Save .meta sidecar file.
void sdcard_save_meta(const char *mp3_path, const TrackMeta &meta);

// Save/load raw bytes to a path
bool sdcard_write_file(const char *path, const uint8_t *data, size_t len);
bool sdcard_read_file(const char *path, uint8_t *buf, size_t len);

#endif
