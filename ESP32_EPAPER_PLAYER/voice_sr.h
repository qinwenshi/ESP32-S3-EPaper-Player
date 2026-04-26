#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Voice command IDs returned by voice_sr_get_cmd()
#define VOICE_CMD_NONE      -1
#define VOICE_CMD_NEXT       0   // 下一首 (xia yi shou)
#define VOICE_CMD_PREV       1   // 上一首 (shang yi shou)
#define VOICE_CMD_PAUSE      2   // 暂停   (zan ting)
#define VOICE_CMD_RESUME     3   // 继续   (ji xu)
#define VOICE_CMD_VOL_UP     4   // 声音大一点 (sheng yin da yi dian)
#define VOICE_CMD_VOL_DOWN   5   // 声音小一点 (sheng yin xiao yi dian)

/**
 * @brief Initialize I2S1 mic + AFE + WakeNet + MultiNet pipeline.
 *        Must be called after audio.setPinout() so I2S0 clocks are running.
 *        Starts a FreeRTOS task on Core 0.
 */
void voice_sr_init(void);

/**
 * @brief Non-blocking: returns the latest detected command and clears it.
 * @return VOICE_CMD_* or VOICE_CMD_NONE (-1) if none pending.
 */
int voice_sr_get_cmd(void);

/**
 * @brief Notify the voice engine of the current audio sample rate.
 *        Call this from audio_info() whenever sample rate changes.
 */
void voice_sr_set_input_rate(uint32_t hz);

#ifdef __cplusplus
}
#endif
