// sprite_anim.cpp — Mimi sprite animation state machine
#include "sprite_anim.h"
#include "mimi_sprites.h"
#include <string.h>

// ── Frame table ───────────────────────────────────────────────────────────────
struct StateInfo {
    const uint8_t **frames;
    int             count;
};

static const StateInfo STATE_TABLE[] = {
    { mimi_idle_frames,          MIMI_IDLE_FRAMES          },  // SPRITE_STATE_IDLE
    { mimi_waiting_frames,       MIMI_WAITING_FRAMES       },  // SPRITE_STATE_WAITING
    { mimi_running_frames,       MIMI_RUNNING_FRAMES       },  // SPRITE_STATE_RUNNING
    { mimi_running_right_frames, MIMI_RUNNING_RIGHT_FRAMES },  // SPRITE_STATE_RUNNING_RIGHT
    { mimi_running_left_frames,  MIMI_RUNNING_LEFT_FRAMES  },  // SPRITE_STATE_RUNNING_LEFT
    { mimi_jumping_frames,       MIMI_JUMPING_FRAMES       },  // SPRITE_STATE_JUMPING
    { mimi_waving_frames,        MIMI_WAVING_FRAMES        },  // SPRITE_STATE_WAVING
    { mimi_failed_frames,        MIMI_FAILED_FRAMES        },  // SPRITE_STATE_FAILED
    { mimi_review_frames,        MIMI_REVIEW_FRAMES        },  // SPRITE_STATE_REVIEW
};

// ── Runtime state ─────────────────────────────────────────────────────────────
static lv_obj_t      *s_img = nullptr;
static lv_image_dsc_t s_dsc = {};

static sprite_state_t s_state       = SPRITE_STATE_IDLE;
static sprite_state_t s_base_state  = SPRITE_STATE_IDLE;  // where to return after one-shot
static bool           s_one_shot    = false;
static int            s_frame       = 0;

// ── Helpers ───────────────────────────────────────────────────────────────────
static void apply_frame(void)
{
    if (!s_img) return;
    const StateInfo &si = STATE_TABLE[s_state];
    s_dsc.data = si.frames[s_frame];
    lv_image_set_src(s_img, &s_dsc);
}

// ── Public API ────────────────────────────────────────────────────────────────
void sprite_anim_init(lv_obj_t *img_widget)
{
    s_img = img_widget;

    s_dsc.header.magic  = LV_IMAGE_HEADER_MAGIC;
    s_dsc.header.cf     = LV_COLOR_FORMAT_I1;
    s_dsc.header.flags  = 0;
    s_dsc.header.w      = SPRITE_W;
    s_dsc.header.h      = SPRITE_H;
    s_dsc.header.stride = SPRITE_STRIDE;
    s_dsc.data_size     = SPRITE_DATA_SZ;
    s_dsc.data          = mimi_idle_frames[0];

    s_state      = SPRITE_STATE_IDLE;
    s_base_state = SPRITE_STATE_IDLE;
    s_one_shot   = false;
    s_frame      = 0;

    lv_image_set_src(s_img, &s_dsc);
}

void sprite_anim_set_state(sprite_state_t state, bool one_shot)
{
    if (s_state == state && !one_shot) return;

    // When starting a one-shot, remember the current looping state to restore
    if (one_shot) {
        if (!s_one_shot) {
            // Save the current persistent (non-one-shot) state
            s_base_state = s_state;
        }
        s_one_shot = true;
    } else {
        s_one_shot   = false;
        s_base_state = state;
    }

    s_state = state;
    s_frame = 0;
    apply_frame();
}

void sprite_anim_tick(void)
{
    const StateInfo &si = STATE_TABLE[s_state];
    s_frame++;

    if (s_frame >= si.count) {
        if (s_one_shot) {
            // Revert to base (IDLE or WAITING) after one-shot completes
            s_one_shot = false;
            s_state    = s_base_state;
            s_frame    = 0;
        } else {
            s_frame = 0;  // loop
        }
    }

    apply_frame();
}
