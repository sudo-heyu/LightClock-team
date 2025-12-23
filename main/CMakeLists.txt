idf_component_register(
    SRCS
        "app_main.c"
        "ble_alarm.c"
        "device_config.c"
        "timekeeper.c"
        "ch455g.c"
        "pwm_led.c"
        "button.c"
    PRIV_REQUIRES bt nvs_flash driver
    INCLUDE_DIRS ".")
