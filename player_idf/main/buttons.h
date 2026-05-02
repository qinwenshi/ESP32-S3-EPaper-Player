// buttons.h — GPIO button ISRs
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void buttons_init(int boot_pin, int pwr_pin);

// Returns true once (clears flag).  held_ms = how long the button was held.
bool buttons_boot_fired(uint32_t *held_ms);
bool buttons_pwr_fired(uint32_t *held_ms);

#ifdef __cplusplus
}
#endif
