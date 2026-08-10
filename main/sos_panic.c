#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "gl868_modem.h"

static const char *TAG = "sos_panic";

#ifndef BUTTON_PIN
#define BUTTON_PIN CONFIG_SAFETY_BAND_SOS_GPIO
#endif

#define LONG_PRESS_MS 3000
#define DEBOUNCE_MS 50
#define SOS_UPDATE_INTERVAL_MS 60000

static bool isArmed = true;
static bool sosActive = false;

static bool parse_cgnsinf_latlon(const char *gps, double *out_lat, double *out_lon)
{
    if (!gps) return false;
    const char *p = strstr(gps, "+CGNSINF:");
    if (!p) {
        return false;
    }
    // skip prefix
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    // tokenize by commas; fields: 1 run status,2 fix status,3 UTC,4 lat,5 lon,...
    char buf[256];
    strncpy(buf, p, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *save = NULL;
    char *tok = strtok_r(buf, ",", &save); // field1
    int field = 1;
    double lat = 0.0, lon = 0.0;
    while (tok) {
        if (field == 4) lat = atof(tok);
        if (field == 5) { lon = atof(tok); break; }
        tok = strtok_r(NULL, ",", &save);
        field++;
    }
    if (lat == 0.0 && lon == 0.0) return false;
    *out_lat = lat;
    *out_lon = lon;
    return true;
}

static void send_sos_message_internal(const char *gps, bool hasLocation, bool isInitial)
{
    char message[512];
    int batt = gl868_modem_get_battery_percent();
    char batt_str[32] = "";
    if (batt >= 0) snprintf(batt_str, sizeof(batt_str), "Battery: %d%%\n", batt);

    if (hasLocation && gps && gps[0] != '\0') {
        double lat = 0.0, lon = 0.0;
        bool parsed = parse_cgnsinf_latlon(gps, &lat, &lon);
        if (parsed) {
            char maps[128];
            snprintf(maps, sizeof(maps), "https://maps.google.com/?q=%.6f,%.6f", lat, lon);
            snprintf(message, sizeof(message),
                     "%s\n%s\n\nLocation:\nLat: %.6f\nLon: %.6f\n%s\nTrack: %s",
                     isInitial ? "*** SOS EMERGENCY ***" : "*** SOS UPDATE ***",
                     isInitial ? "Panic button pressed!" : "Continuous tracking active",
                     lat, lon, batt_str, maps);
        } else {
            snprintf(message, sizeof(message),
                     "%s\n%s\n\nLocation: %s\n%s",
                     isInitial ? "*** SOS EMERGENCY ***" : "*** SOS UPDATE ***",
                     isInitial ? "Panic button pressed!" : "Continuous tracking active",
                     "GPS data available but parsing failed",
                     batt_str);
        }
    } else {
        snprintf(message, sizeof(message),
                 "%s\n%s\n\nLocation: No GPS fix (retrying)\n%s",
                 isInitial ? "*** SOS EMERGENCY ***" : "*** SOS UPDATE ***",
                 isInitial ? "Panic button pressed!" : "Continuous tracking active",
                 batt_str);
    }

    const char *sms_number = gl868_modem_get_emergency_sms_number();
    if (sms_number) {
        ESP_LOGI(TAG, "Sending SOS SMS to %s", sms_number);
        gl868_modem_send_sms_to(sms_number, message);
    }
}

extern SemaphoreHandle_t s_sos_sem; // provided by safety_band_main.c

static void sos_task(void *arg)
{
    // Configure input with interrupt on any edge and a queue for events
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // The global semaphore `s_sos_sem` is given from the ISR in init_io()
    // configured in `safety_band_main.c`. We wait on that semaphore for presses.

    TickType_t press_start = 0;
    bool longHandled = false;
    TickType_t lastUpdate = 0;

    ESP_LOGI(TAG, "SOS task started, button pin=%d", BUTTON_PIN);


    for (;;) {
        // wait indefinitely for ISR to give semaphore on either edge
        if (!(s_sos_sem && xSemaphoreTake(s_sos_sem, portMAX_DELAY) == pdTRUE)) {
            continue;
        }

        // determine current level: 0 = pressed, 1 = released
        int level = gpio_get_level(BUTTON_PIN);
        if (level == 0) {
            // press detected - wait for release or timeout for long-press
            press_start = xTaskGetTickCount();
            longHandled = false;
            if (xSemaphoreTake(s_sos_sem, pdMS_TO_TICKS(LONG_PRESS_MS)) == pdTRUE) {
                // release happened before long-press timeout
                int level2 = gpio_get_level(BUTTON_PIN);
                TickType_t held = xTaskGetTickCount() - press_start;
                if (level2 == 1 && held >= pdMS_TO_TICKS(DEBOUNCE_MS) && !longHandled) {
                    // short press
                    if (isArmed) {
                        ESP_LOGI(TAG, "Short press detected: triggering SOS");
                        sosActive = true;
                        char gpsbuf[256] = {0};
                        bool hasGps = gl868_modem_get_gps_now(gpsbuf, sizeof(gpsbuf));
                        send_sos_message_internal(gpsbuf, hasGps, true);
                        const char *call_number = gl868_modem_get_emergency_call_number();
                        if (call_number) {
                            ESP_LOGI(TAG, "Calling emergency number: %s", call_number);
                            gl868_modem_make_call_to(call_number);
                        }
                        lastUpdate = xTaskGetTickCount();
                    } else {
                        ESP_LOGI(TAG, "Short press ignored - system disarmed");
                    }
                }
            } else {
                // timeout -> consider long press
                longHandled = true;
                isArmed = !isArmed;
                ESP_LOGI(TAG, "System %s", isArmed ? "ARMED" : "DISARMED");
                if (!isArmed) {
                    sosActive = false;
                    const char *sms_number = gl868_modem_get_emergency_sms_number();
                    if (sms_number) gl868_modem_send_sms_to(sms_number, "SOS System DISARMED - Emergency mode ended");
                } else {
                    const char *sms_number = gl868_modem_get_emergency_sms_number();
                    if (sms_number) gl868_modem_send_sms_to(sms_number, "SOS System ARMED");
                }
                // wait for release event to consume it
                xSemaphoreTake(s_sos_sem, portMAX_DELAY);
            }
        } else {
            // release without prior press (spurious) - ignore
        }

        // Periodic updates while SOS active
        if (sosActive && isArmed) {
            if (lastUpdate == 0) lastUpdate = xTaskGetTickCount();
            if ((xTaskGetTickCount() - lastUpdate) >= pdMS_TO_TICKS(SOS_UPDATE_INTERVAL_MS)) {
                char gpsbuf[256] = {0};
                bool hasGps = gl868_modem_get_gps_now(gpsbuf, sizeof(gpsbuf));
                ESP_LOGI(TAG, "Sending periodic SOS update (hasGps=%d)", hasGps);
                send_sos_message_internal(gpsbuf, hasGps, false);
                lastUpdate = xTaskGetTickCount();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void sos_panic_start(void)
{
    xTaskCreate(sos_task, "sos_task", 4096, NULL, 7, NULL);
}