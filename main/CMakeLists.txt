idf_component_register(
    SRCS
        "app_main.c"
        "ble_alarm.c"
        "device_config.c"
        "timekeeper.c"
        "battery.c"
        "ch455g.c"
        "pwm_led.c"
        "button.c"
    PRIV_REQUIRES bt nvs_flash driver esp_adc
    INCLUDE_DIRS ".")
