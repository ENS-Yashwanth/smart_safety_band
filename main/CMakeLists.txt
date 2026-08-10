idf_component_register(SRCS "safety_band_main.c" "communication_task.c" "safety_manager_task.c" "manual_sos_task.c" "sos_panic.c" "motion_tracker.c" "gnss_task.c" "voice_call_task.c" "sms_alert_task.c" "emergency_dispatch_task.c" "live_location_task.c" "gl868_modem.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_uart esp_driver_i2s esp_modem nvs_flash)
