#include "pwm_led.h"

#include "driver/ledc.h"

static uint32_t percent_to_duty(uint8_t percent, uint32_t duty_max)
{
    if (percent >= 100) {
        return duty_max;
    }
    return (uint32_t)((uint64_t)percent * duty_max / 100ULL);
}

esp_err_t pwm_led_init(pwm_led_t *led, gpio_num_t warm_gpio, gpio_num_t cool_gpio)
{
    if (!led) {
        return ESP_ERR_INVALID_ARG;
    }

    led->warm_gpio = warm_gpio;
    led->cool_gpio = cool_gpio;
    led->freq_hz = 24000;

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_9_BIT, // 0..511 (<=500-ish requirement)
        .timer_num = LEDC_TIMER_0,
        .freq_hz = led->freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }

    led->duty_max = (1u << 9) - 1;

    ledc_channel_config_t ch0 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = warm_gpio,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&ch0);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t ch1 = ch0;
    ch1.channel = LEDC_CHANNEL_1;
    ch1.gpio_num = cool_gpio;
    ch1.duty = 0;
    err = ledc_channel_config(&ch1);
    if (err != ESP_OK) {
        return err;
    }

    led->inited = true;
    return ESP_OK;
}

esp_err_t pwm_led_set_percent(pwm_led_t *led, uint8_t warm_percent, uint8_t cool_percent)
{
    if (!led || !led->inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t warm_duty = percent_to_duty(warm_percent, led->duty_max);
    uint32_t cool_duty = percent_to_duty(cool_percent, led->duty_max);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, warm_duty);
    if (err != ESP_OK) return err;
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (err != ESP_OK) return err;

    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, cool_duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

esp_err_t pwm_led_off(pwm_led_t *led)
{
    if (!led || !led->inited) {
        return ESP_OK;
    }
    return pwm_led_set_percent(led, 0, 0);
}
