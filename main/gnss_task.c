#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs.h"

#include "gl868_modem.h"
#include "safety_shared.h"

static const char *TAG = "GNSS_TASK";

#ifndef GNSS_FIX_TIMEOUT_MS
#define GNSS_FIX_TIMEOUT_MS 15000
#endif

#ifndef GNSS_MIN_SATELLITES
#define GNSS_MIN_SATELLITES 4
#endif

#ifndef GNSS_MAX_HDOP
#define GNSS_MAX_HDOP 2.5f
#endif
#ifndef GNSS_NORMAL_INTERVAL_SECONDS
#define GNSS_NORMAL_INTERVAL_SECONDS 900
#endif
#define GNSS_TRACKING_SAMPLE_MS 1000

static uint32_t normal_interval_seconds(void)
{
    uint32_t seconds = GNSS_NORMAL_INTERVAL_SECONDS;
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READONLY, &handle) == ESP_OK) {
        (void)nvs_get_u32(handle, "gnss_interval_s", &seconds);
        nvs_close(handle);
    }
    if (seconds < 60) seconds = 60;
    if (seconds > 1800) seconds = 1800;
    return seconds;
}

bool gnss_task_set_normal_reporting_interval(uint32_t seconds)
{
    if (seconds < 60 || seconds > 1800) return false;
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t err = nvs_set_u32(handle, "gnss_interval_s", seconds);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err == ESP_OK;
}

static bool parse_cgnsinf_info(const char *response, bool *has_fix, double *out_lat, double *out_lon, char *out_utc, size_t utc_len,
                               float *out_altitude, float *out_speed, float *out_bearing,
                               int *out_fix_mode, int *out_sat_used, int *out_sat_view, float *out_hdop)
{
    if (!response) return false;
    const char *prefix = "+CGNSINF:";
    const char *pos = strstr(response, prefix);
    if (!pos) return false;
    pos += strlen(prefix);
    char buf[256];
    strncpy(buf, pos, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *saveptr = NULL;
    char *token = strtok_r(buf, ",", &saveptr);
    int field = 1;
    bool fix = false;
    double lat = 0.0;
    double lon = 0.0;
    int fix_mode = 0;
    int sat_used = 0;
    int sat_view = 0;
    float hdop = 99.9f;
    float altitude = 0.0f, speed = 0.0f, bearing = 0.0f;
    if (out_utc && utc_len) out_utc[0] = '\0';

    while (token) {
        switch (field) {
        case 1: // GNSS run status
            break;
        case 2: // Fix status
            fix = (token[0] == '1');
            break;
        case 3: // UTC time
            if (out_utc && utc_len) snprintf(out_utc, utc_len, "%s", token);
            break;
        case 4: // latitude
            lat = atof(token);
            break;
        case 5: // longitude
            lon = atof(token);
            break;
        case 6: // MSL altitude
            altitude = (float)atof(token);
            break;
        case 7: // speed over ground
            speed = (float)atof(token);
            break;
        case 8: // course over ground
            bearing = (float)atof(token);
            break;
        case 9: // fix mode
            fix_mode = atoi(token);
            break;
        case 10: // reserved
            break;
        case 11: // HDOP
            hdop = (float)atof(token);
            break;
        case 12: // PDOP
            break;
        case 13: // VDOP
            break;
        case 14: // satellites in use
            sat_used = atoi(token);
            break;
        case 15: // satellites in view
            sat_view = atoi(token);
            break;
        default:
            break;
        }
        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }

    if (out_lat) *out_lat = lat;
    if (out_lon) *out_lon = lon;
    if (out_fix_mode) *out_fix_mode = fix_mode;
    if (out_sat_used) *out_sat_used = sat_used;
    if (out_sat_view) *out_sat_view = sat_view;
    if (out_hdop) *out_hdop = hdop;
    if (out_altitude) *out_altitude = altitude;
    if (out_speed) *out_speed = speed;
    if (out_bearing) *out_bearing = bearing;
    if (has_fix) *has_fix = fix;
    return true;
}

static void update_gnss_shared_state(const char *raw, const char *utc, bool valid, double lat, double lon,
                                     int fix_mode, int sat_used, int sat_view, float hdop, float altitude, float speed, float bearing)
{
    if (xSemaphoreTake(s_gnss_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not take GNSS mutex");
        return;
    }

    s_gnss_fix.valid = valid;
    s_gnss_fix.powered = true;
    s_gnss_fix.latitude = lat;
    s_gnss_fix.longitude = lon;
    s_gnss_fix.hdop = hdop;
    s_gnss_fix.altitude_m = altitude;
    s_gnss_fix.speed_kph = speed;
    s_gnss_fix.bearing_degrees = bearing;
    s_gnss_fix.satellites_used = sat_used;
    s_gnss_fix.satellites_in_view = sat_view;
    s_gnss_fix.fix_mode = fix_mode;
    s_gnss_fix.timestamp = xTaskGetTickCount();
    snprintf(s_gnss_fix.utc, sizeof(s_gnss_fix.utc), "%s", utc ? utc : "");
    strncpy(s_gnss_fix.raw_info, raw ? raw : "", sizeof(s_gnss_fix.raw_info) - 1);
    s_gnss_fix.raw_info[sizeof(s_gnss_fix.raw_info) - 1] = '\0';

    xSemaphoreGive(s_gnss_mutex);
}

void gnss_task_process_response(const char *response)
{
    bool has_fix = false;
    double lat = 0.0, lon = 0.0;
    char utc[24] = {0};
    int fix_mode = 0, sat_used = 0, sat_view = 0;
    float hdop = 99.9f;
    float altitude = 0.0f, speed = 0.0f, bearing = 0.0f;
    if (parse_cgnsinf_info(response, &has_fix, &lat, &lon, utc, sizeof(utc),
                            &altitude, &speed, &bearing, &fix_mode, &sat_used, &sat_view, &hdop)) {
        bool valid = has_fix && sat_view >= GNSS_MIN_SATELLITES && hdop > 0.0f && hdop <= GNSS_MAX_HDOP;
        update_gnss_shared_state(response, utc, valid, lat, lon, fix_mode, sat_used, sat_view, hdop, altitude, speed, bearing);
    } else {
        update_gnss_shared_state(response, "", false, 0.0, 0.0, 0, 0, 0, 99.9f, 0.0f, 0.0f, 0.0f);
    }
}

void gnss_task_apply_filtered_location(double latitude, double longitude)
{
    if (xSemaphoreTake(s_gnss_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_gnss_fix.latitude = latitude;
        s_gnss_fix.longitude = longitude;
        xSemaphoreGive(s_gnss_mutex);
    }
}

static void gnss_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "GNSS task started");
    TickType_t last_normal_cycle = xTaskGetTickCount() - pdMS_TO_TICKS(normal_interval_seconds() * 1000U);
    for (;;) {
        if (!gl868_modem_init()) {
            ESP_LOGW(TAG, "GNSS task restarting modem initialization");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        /* The live-location worker owns 1 Hz acquisition in emergency mode.
         * Normal mode powers GNSS only while acquiring a scheduled fix. */
        if (xEventGroupGetBits(s_system_events) & BIT_EMERGENCY) {
            vTaskDelay(pdMS_TO_TICKS(GNSS_TRACKING_SAMPLE_MS));
            continue;
        }

        uint32_t interval_seconds = normal_interval_seconds();
        if ((xTaskGetTickCount() - last_normal_cycle) < pdMS_TO_TICKS(interval_seconds * 1000U)) {
            vTaskDelay(pdMS_TO_TICKS(GNSS_TRACKING_SAMPLE_MS));
            continue;
        }

        if (!gl868_modem_enable_gnss()) {
            ESP_LOGW(TAG, "Failed to power GNSS for normal acquisition");
            last_normal_cycle = xTaskGetTickCount();
            continue;
        }

        TickType_t started = xTaskGetTickCount();
        bool valid_fix = false;
        do {
            char response[256] = {0};
            if (gl868_modem_get_gps_now(response, sizeof(response))) {
                gnss_task_process_response(response);
                if (xSemaphoreTake(s_gnss_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    valid_fix = s_gnss_fix.valid;
                    xSemaphoreGive(s_gnss_mutex);
                }
            }
            if (!valid_fix) vTaskDelay(pdMS_TO_TICKS(GNSS_TRACKING_SAMPLE_MS));
        } while (!valid_fix &&
                 (xTaskGetTickCount() - started) < pdMS_TO_TICKS(GNSS_FIX_TIMEOUT_MS) &&
                 !(xEventGroupGetBits(s_system_events) & BIT_EMERGENCY));

        if (!(xEventGroupGetBits(s_system_events) & BIT_EMERGENCY)) {
            (void)gl868_modem_disable_gnss();
            gl868_modem_shutdown();
        }
        last_normal_cycle = xTaskGetTickCount();
    }
}

void gnss_task_start(void)
{
    xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 5, NULL);
}
