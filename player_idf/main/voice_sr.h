#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Voice command IDs returned by voice_sr_get_cmd()
#define VOICE_CMD_NONE      -1
#define VOICE_CMD_NEXT       0   // next song
#define VOICE_CMD_PREV       1   // previous song
#define VOICE_CMD_PAUSE      2   // pause
#define VOICE_CMD_RESUME     3   // play / resume
#define VOICE_CMD_VOL_UP     4   // volume up
#define VOICE_CMD_VOL_DOWN   5   // volume down

/**
 * @brief Initialize I2S1 mic + AFE + WakeNet + MultiNet pipeline.
 *        Must be called after audio.setPinout() so I2S0 clocks are running.
 */
void voice_sr_init(void);

/**
 * @brief Non-blocking: returns the latest detected command and clears it.
 * @return VOICE_CMD_* or VOICE_CMD_NONE (-1) if none pending.
 */
int voice_sr_get_cmd(void);

/**
 * @brief Notify the voice engine of the current audio sample rate.
 *        Call from audio_info() whenever sample rate changes.
 */
void voice_sr_set_input_rate(uint32_t hz);

/**
 * @brief Manually activate command listening (skips wake word).
 *        Use this when the speaker is muted so SNR is good.
 *        Voice engine will listen for a command for up to 6 seconds.
 */
void voice_sr_start_listen(void);

/**
 * @brief Returns true while a manual-listen or post-wake-word command
 *        window is open.  The caller should mute the speaker during this
 *        window and restore volume when it returns false.
 */
bool voice_sr_is_listening(void);

#ifdef __cplusplus
}
#endif
