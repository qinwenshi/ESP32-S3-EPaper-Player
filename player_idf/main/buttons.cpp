// buttons.cpp — GPIO button ISR handler
#include "buttons.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static volatile bool     s_boot_flag = false;
static volatile bool     s_pwr_flag  = false;
static volatile uint32_t s_boot_down = 0;
static volatile uint32_t s_boot_held = 0;
static volatile uint32_t s_pwr_down  = 0;
static volatile uint32_t s_pwr_held  = 0;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void IRAM_ATTR isr_boot(void *arg)
{
    int level = gpio_get_level((gpio_num_t)(intptr_t)arg);
    if (level == 0) {
        s_boot_down = now_ms();
    } else {
        s_boot_held = now_ms() - s_boot_down;
        s_boot_flag = true;
    }
}

static void IRAM_ATTR isr_pwr(void *arg)
{
    int level = gpio_get_level((gpio_num_t)(intptr_t)arg);
    if (level == 0) {
        s_pwr_down = now_ms();
    } else {
        s_pwr_held = now_ms() - s_pwr_down;
        s_pwr_flag = true;
    }
}

void buttons_init(int boot_pin, int pwr_pin)
{
    gpio_config_t cfg = {};
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_ANYEDGE;
    cfg.pin_bit_mask = (1ULL << boot_pin) | (1ULL << pwr_pin);
    gpio_config(&cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)boot_pin, isr_boot, (void*)(intptr_t)boot_pin);
    gpio_isr_handler_add((gpio_num_t)pwr_pin,  isr_pwr,  (void*)(intptr_t)pwr_pin);
}

bool buttons_boot_fired(uint32_t *held_ms)
{
    if (!s_boot_flag) return false;
    s_boot_flag = false;
    if (held_ms) *held_ms = s_boot_held;
    return true;
}

bool buttons_pwr_fired(uint32_t *held_ms)
{
    if (!s_pwr_flag) return false;
    s_pwr_flag = false;
    if (held_ms) *held_ms = s_pwr_held;
    return true;
}
