idf_component_register(
    SRCS
        "app_main.c"
        "control.c"
        "canopen.c"
        "motor.c"
        "encoder.c"
        "ir_safety.c"
        "pid.c"
        "jitter.c"
        "loadgen.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_twai
)
