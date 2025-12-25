#include "button.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool is_pressed(const button_t *btn)
{
    int lvl = gpio_get_level(btn->gpio);
    return btn->active_low ? (lvl == 0) : (lvl != 0);
}

esp_err_t button_init(button_t *btn, gpio_num_t gpio, bool active_low, uint32_t long_press_ms)
{
    if (!btn) {
        return ESP_ERR_INVALID_ARG;
    }

    btn->gpio = gpio;
    btn->active_low = active_low;
    btn->long_press_ms = long_press_ms;
    btn->last_pressed = false;
    btn->press_start_us = 0;
    btn->long_reported = false;

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&cfg);
}

uint32_t button_measure_press_ms(button_t *btn, uint32_t max_ms)
{
    if (!btn) {
        return 0;
    }

    if (!is_pressed(btn)) {
        return 0;
    }

    int64_t start = esp_timer_get_time();
    while (is_pressed(btn)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        int64_t now = esp_timer_get_time();
        if ((uint32_t)((now - start) / 1000) >= max_ms) {
            break;
        }
    }
    int64_t end = esp_timer_get_time();
    return (uint32_t)((end - start) / 1000);
}

button_event_t button_poll(button_t *btn)
{
    if (!btn) {
        return BUTTON_EVENT_NONE;
    }

    bool pressed = is_pressed(btn);
    int64_t now_us = esp_timer_get_time();

    if (pressed && !btn->last_pressed) {
        btn->press_start_us = now_us;
        btn->last_pressed = true;
        btn->long_reported = false;
        return BUTTON_EVENT_NONE;
    }

    // While still pressed, emit LONG once when the threshold is reached.
    if (pressed && btn->last_pressed && !btn->long_reported) {
        uint32_t dur_ms = (uint32_t)((now_us - btn->press_start_us) / 1000);
        if (dur_ms >= btn->long_press_ms) {
            btn->long_reported = true;
            return BUTTON_EVENT_LONG;
        }
    }

    if (!pressed && btn->last_pressed) {
        uint32_t dur_ms = (uint32_t)((now_us - btn->press_start_us) / 1000);
        btn->last_pressed = false;
        btn->press_start_us = 0;
        if (btn->long_reported || dur_ms >= btn->long_press_ms) {
            btn->long_reported = false;
            return BUTTON_EVENT_NONE;
        }
        return BUTTON_EVENT_SHORT;
    }

    return BUTTON_EVENT_NONE;
}
