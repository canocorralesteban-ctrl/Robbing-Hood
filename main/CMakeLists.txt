idf_component_register(SRCS "main.c" "funciones.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_ledc esp_driver_gpio esp_adc esp_driver_rmt esp_timer)
