// audio.h — MP3 player using minimp3 + ESP-IDF I2S driver
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/i2s_std.h"

#ifdef __cplusplus
extern "C" {
#endif

// Callbacks set by app_main
typedef void (*audio_meta_cb_t)(const char *key, const char *value);
typedef void (*audio_image_cb_t)(uint32_t file_offset, uint32_t byte_len);
typedef void (*audio_eof_cb_t)(void);

void audio_init(int bclk, int ws, int dout, int mclk);
// Returns the I2S0 RX handle (duplex mic channel, 32-bit Philips, 44100 Hz).
// Valid only after audio_init(). Used by voice_sr to read mic data.
i2s_chan_handle_t audio_get_i2s_rx(void);
void audio_set_callbacks(audio_meta_cb_t meta_cb,
                          audio_image_cb_t image_cb,
                          audio_eof_cb_t eof_cb);

// Play a file from the VFS-mounted SD card (e.g. "/sdcard/music/track.mp3")
bool audio_play(const char *vfs_path);

void audio_stop(void);
void audio_pause_resume(void);
bool audio_is_running(void);

// Returns current playback position in seconds (based on decoded frames)
uint32_t audio_get_current_time(void);
// Returns total duration in seconds (from VBR/CBR header scan)
uint32_t audio_get_duration(void);
// Seek to position in seconds (crude: byte-offset estimate)
void audio_seek(uint32_t seconds);

// Volume: 0-100 (applied to I2S output scale)
// (primary volume control is codec; this is software trim)

#ifdef __cplusplus
}
#endif
