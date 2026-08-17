idf_component_register(SRCS "safety_band_main.c" "gl868_modem.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_uart esp_modem esp_pm)
