#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool inited;
    gpio_num_t warm_gpio;
    gpio_num_t cool_gpio;
    uint32_t freq_hz;
    uint32_t duty_max;
} pwm_led_t;

esp_err_t pwm_led_init(pwm_led_t *led, gpio_num_t warm_gpio, gpio_num_t cool_gpio);

// percent 0..100
esp_err_t pwm_led_set_percent(pwm_led_t *led, uint8_t warm_percent, uint8_t cool_percent);

esp_err_t pwm_led_off(pwm_led_t *led);

#ifdef __cplusplus
}
#endif
