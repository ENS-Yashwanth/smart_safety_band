#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "gl868_modem.h"

#ifndef MOTION_THRESHOLD_MG
#define MOTION_THRESHOLD_MG 300
#endif

#ifndef MOTION_POLL_MS
#define MOTION_POLL_MS 200
#endif

#ifndef MOTION_UPDATE_MS
#define MOTION_UPDATE_MS 300000
#endif

static const char *TAG = "MOTION_TRACKER";

// forward to C function in safety_band_main.c
extern int32_t get_motion_magnitude_mg(void);

static void motion_tracker_task(void *arg)
{
    bool in_motion = false;
    TickType_t last_motion_tick = 0;

    for (;;) {
        int32_t mag = get_motion_magnitude_mg();
        if (mag >= MOTION_THRESHOLD_MG) {
            if (!in_motion) {
                in_motion = true;
                ESP_LOGI(TAG, "Motion started (mg=%d)", mag);
                // send immediate location
                char gpsbuf[256] = {0};
                if (gl868_modem_get_gps_now(gpsbuf, sizeof(gpsbuf))) {
                    const char *recip = gl868_modem_get_emergency_sms_number();
                    if (recip) {
                        char msg[512];
                        snprintf(msg, sizeof(msg), "MOTION START: GPS: %s", gpsbuf);
                        gl868_modem_send_sms_to(recip, msg);
                        ESP_LOGI(TAG, "Sent immediate location to %s", recip);
                    }
                } else {
                    ESP_LOGW(TAG, "Could not get GPS at motion start");
                }
                last_motion_tick = xTaskGetTickCount();
            } else {
                // still in motion; if MOTION_UPDATE_MS passed since last update, send update
                if ((xTaskGetTickCount() - last_motion_tick) >= pdMS_TO_TICKS(MOTION_UPDATE_MS)) {
                    char gpsbuf[256] = {0};
                    if (gl868_modem_get_gps_now(gpsbuf, sizeof(gpsbuf))) {
                        const char *recip = gl868_modem_get_emergency_sms_number();
                        if (recip) {
                            char msg[512];
                            snprintf(msg, sizeof(msg), "MOTION UPDATE: GPS: %s", gpsbuf);
                            gl868_modem_send_sms_to(recip, msg);
                            ESP_LOGI(TAG, "Sent periodic location to %s", recip);
                        }
                    } else {
                        ESP_LOGW(TAG, "Could not get GPS for periodic update");
                    }
                    last_motion_tick = xTaskGetTickCount();
                }
            }
        } else {
            if (in_motion) {
                // require a small quiet window before declaring stopped
                static int quiet_count = 0;
                if (++quiet_count * MOTION_POLL_MS >= 3000) {
                    in_motion = false;
                    quiet_count = 0;
                    ESP_LOGI(TAG, "Motion stopped (mg=%d)", mag);
                    const char *recip = gl868_modem_get_emergency_sms_number();
                    if (recip) {
                        gl868_modem_send_sms_to(recip, "MOTION STOPPED");
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MOTION_POLL_MS));
    }
}

void motion_tracker_start(void)
{
    xTaskCreate(motion_tracker_task, "motion_tracker", 4096, NULL, 6, NULL);
}