// nvs_state.h — NVS wrappers replacing Arduino Preferences
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void nvs_state_init(void);

int32_t  nvs_state_get_int(const char *key, int32_t def);
void     nvs_state_set_int(const char *key, int32_t val);

uint32_t nvs_state_get_uint(const char *key, uint32_t def);
void     nvs_state_set_uint(const char *key, uint32_t val);

#ifdef __cplusplus
}
#endif
