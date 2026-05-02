// codec.h — ES8311 codec driver over IDF i2c_master
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool    codec_init(int sda, int scl, uint32_t freq_hz);
void    codec_set_volume(uint8_t vol);   // 0-100
uint8_t codec_get_volume(void);
void    codec_set_sample_rate(uint32_t hz);
void    codec_enable_mic(bool en);
void    codec_set_mic_gain(uint8_t gain); // 0-7
void    codec_dac_mute(bool mute);
void    codec_read_all(void);             // debug register dump

#ifdef __cplusplus
}
#endif
