// buttons.h — GPIO button ISRs
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void buttons_init(int boot_pin, int pwr_pin);

// Single-click: short press (20-1200 ms) not followed by a second press within 400 ms.
// held_ms = press duration.
bool buttons_boot_fired(uint32_t *held_ms);

// Double-click: two short presses with release-to-release gap < 400 ms.
bool buttons_boot_double_fired(void);

bool buttons_pwr_fired(uint32_t *held_ms);

#ifdef __cplusplus
}
#endif
