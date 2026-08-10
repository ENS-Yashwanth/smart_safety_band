#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "gl868_modem.h"
#include "gnss_task.h"
#include "sms_alert_task.h"
#include "safety_shared.h"

static const char *TAG = "LIVE_LOCATION";

#define LIVE_GNSS_SAMPLE_MS 1000
#define LIVE_SMS_FALLBACK_MS 60000

typedef struct {
    bool initialized;
    double latitude;
    double longitude;
    double variance;
} location_kalman_t;

static void kalman_update(location_kalman_t *filter, double measurement, double *estimate)
{
    const double process_noise = 0.00000001;
    const double measurement_noise = 0.00001;
    if (!filter->initialized) {
        *estimate = measurement;
        return;
    }
    filter->variance += process_noise;
    double gain = filter->variance / (filter->variance + measurement_noise);
    *estimate += gain * (measurement - *estimate);
    filter->variance *= (1.0 - gain);
}

static bool snapshot_fix(gnss_fix_t *fix)
{
    if (xSemaphoreTake(s_gnss_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    *fix = s_gnss_fix;
    xSemaphoreGive(s_gnss_mutex);
    return fix->valid;
}

static void live_location_task(void *arg)
{
    (void)arg;
    location_kalman_t filter = {.variance = 1.0};
    ESP_LOGI(TAG, "Live location task started");

    for (;;) {
        xEventGroupWaitBits(s_system_events, BIT_EMERGENCY, pdFALSE, pdTRUE, portMAX_DELAY);
        ESP_LOGW(TAG, "Emergency mode entered; starting 1 Hz GNSS tracking");
        (void)gl868_modem_enable_gnss();
        filter.initialized = false;
        TickType_t last_sms_fallback = 0;

        while (xEventGroupGetBits(s_system_events) & BIT_EMERGENCY) {
            char raw[256] = {0};
            if (gl868_modem_get_gps_now(raw, sizeof(raw))) gnss_task_process_response(raw);

            gnss_fix_t fix = {0};
            if (snapshot_fix(&fix)) {
                if (!filter.initialized) {
                    filter.latitude = fix.latitude;
                    filter.longitude = fix.longitude;
                    filter.variance = 1.0;
                    filter.initialized = true;
                } else {
                    kalman_update(&filter, fix.latitude, &filter.latitude);
                    kalman_update(&filter, fix.longitude, &filter.longitude);
                }
                gnss_task_apply_filtered_location(filter.latitude, filter.longitude);

                TickType_t now = xTaskGetTickCount();
                if (last_sms_fallback == 0 || now - last_sms_fallback >= pdMS_TO_TICKS(LIVE_SMS_FALLBACK_MS)) {
                    sms_alert_task_request_live_location();
                    last_sms_fallback = now;
                    ESP_LOGW(TAG, "Queued SMS live-location fallback");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(LIVE_GNSS_SAMPLE_MS));
        }
        ESP_LOGI(TAG, "Emergency mode exited; live tracking paused");
    }
}

void live_location_task_start(void)
{
    xTaskCreate(live_location_task, "live_location", 6144, NULL, 9, NULL);
}
