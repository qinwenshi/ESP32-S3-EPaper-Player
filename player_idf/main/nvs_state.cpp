// nvs_state.cpp — thin NVS wrapper replacing Arduino Preferences
#include "nvs_state.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_state";
static const char *NVS_NS = "player";

void nvs_state_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

static nvs_handle_t open_rw(void)
{
    nvs_handle_t h = 0;
    nvs_open(NVS_NS, NVS_READWRITE, &h);
    return h;
}

static nvs_handle_t open_ro(void)
{
    nvs_handle_t h = 0;
    nvs_open(NVS_NS, NVS_READONLY, &h);
    return h;
}

int32_t nvs_state_get_int(const char *key, int32_t def)
{
    nvs_handle_t h = open_ro();
    if (!h) return def;
    int32_t v = def;
    nvs_get_i32(h, key, &v);
    nvs_close(h);
    return v;
}

void nvs_state_set_int(const char *key, int32_t val)
{
    nvs_handle_t h = open_rw();
    if (!h) return;
    nvs_set_i32(h, key, val);
    nvs_commit(h);
    nvs_close(h);
}

uint32_t nvs_state_get_uint(const char *key, uint32_t def)
{
    nvs_handle_t h = open_ro();
    if (!h) return def;
    uint32_t v = def;
    nvs_get_u32(h, key, &v);
    nvs_close(h);
    return v;
}

void nvs_state_set_uint(const char *key, uint32_t val)
{
    nvs_handle_t h = open_rw();
    if (!h) return;
    nvs_set_u32(h, key, val);
    nvs_commit(h);
    nvs_close(h);
}
