// sprite_anim.h — Mimi sprite animation state machine for ESP32-S3 e-paper player
#pragma once
#include "lvgl.h"

typedef enum {
    SPRITE_STATE_IDLE,           // playing normally — breathing/blinking loop
    SPRITE_STATE_WAITING,        // paused — patient waiting loop
    SPRITE_STATE_RUNNING,        // auto-advance to next track (EOF)
    SPRITE_STATE_RUNNING_RIGHT,  // user pressed next track
    SPRITE_STATE_RUNNING_LEFT,   // user pressed prev track
    SPRITE_STATE_JUMPING,        // volume up command
    SPRITE_STATE_WAVING,         // wake word / voice command mode
    SPRITE_STATE_FAILED,         // error: no SD card / no tracks
    SPRITE_STATE_REVIEW,         // loading / scanning SD
} sprite_state_t;

// Initialise the sprite widget. Must be called once after LVGL screen is built.
// img_widget must be a pre-created lv_image object of size SPRITE_W x SPRITE_H.
void sprite_anim_init(lv_obj_t *img_widget);

// Set a new state. If one_shot is true the animation plays once then reverts
// to the state that was active before this call (IDLE or WAITING).
void sprite_anim_set_state(sprite_state_t state, bool one_shot);

// Advance one animation frame. Call every ~300 ms from the main task
// (under lvgl_mtx).
void sprite_anim_tick(void);
