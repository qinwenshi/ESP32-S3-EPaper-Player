// buttons.cpp — GPIO button ISR handler
#include "buttons.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

// ── BOOT button state ────────────────────────────────────────────────────────
static volatile bool     s_boot_single = false;  // single-click pending
static volatile bool     s_boot_double = false;  // double-click pending
static volatile uint32_t s_boot_down         = 0;
static volatile uint32_t s_boot_held         = 0;
static volatile uint32_t s_boot_last_release = 0; // time of previous release

// ── PWR button state ─────────────────────────────────────────────────────────
static volatile bool     s_pwr_flag = false;
static volatile uint32_t s_pwr_down = 0;
static volatile uint32_t s_pwr_held = 0;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Double-click window: two releases within 400 ms, each press 20-1200 ms
#define DCLICK_WINDOW_MS  400
#define PRESS_MIN_MS       20
#define PRESS_MAX_MS     1200

static void IRAM_ATTR isr_boot(void *arg)
{
    uint32_t now = now_ms();
    int level = gpio_get_level((gpio_num_t)(intptr_t)arg);
    if (level == 0) {
        // Falling edge — button pressed
        s_boot_down = now;
    } else {
        // Rising edge — button released
        uint32_t held = now - s_boot_down;
        if (held < PRESS_MIN_MS || held > PRESS_MAX_MS) {
            // Too short (noise) or too long — ignore for click logic
            s_boot_last_release = 0;
            return;
        }
        s_boot_held = held;

        uint32_t gap = now - s_boot_last_release;
        if (s_boot_last_release != 0 && gap < DCLICK_WINDOW_MS) {
            // Second release within window → double-click
            s_boot_double       = true;
            s_boot_single       = false; // cancel any pending single
            s_boot_last_release = 0;
        } else {
            // First release (or gap too long) → potential single-click
            s_boot_single       = true;
            s_boot_last_release = now;
        }
    }
}

static void IRAM_ATTR isr_pwr(void *arg)
{
    uint32_t now = now_ms();
    int level = gpio_get_level((gpio_num_t)(intptr_t)arg);
    if (level == 0) {
        s_pwr_down = now;
    } else {
        s_pwr_held = now - s_pwr_down;
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
    if (!s_boot_single) return false;
    s_boot_single = false;
    if (held_ms) *held_ms = s_boot_held;
    return true;
}

bool buttons_boot_double_fired(void)
{
    if (!s_boot_double) return false;
    s_boot_double = false;
    return true;
}

bool buttons_pwr_fired(uint32_t *held_ms)
{
    if (!s_pwr_flag) return false;
    s_pwr_flag = false;
    if (held_ms) *held_ms = s_pwr_held;
    return true;
}
