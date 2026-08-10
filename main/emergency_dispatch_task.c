#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"

#include "gl868_modem.h"
#include "gnss_task.h"
#include "safety_shared.h"

static const char *TAG = "EMERGENCY_DISPATCH";

#define DISPATCH_SMS_BIT   BIT0
#define DISPATCH_VOICE_BIT BIT1
#define DISPATCH_DEFAULT_CHANNELS (DISPATCH_SMS_BIT | DISPATCH_VOICE_BIT)
#define DISPATCH_LOG_SLOTS 8

typedef struct {
    uint32_t uptime_seconds;
    uint8_t event_type;
    uint8_t channels;
    bool gnss_valid;
    float hdop;
} dispatch_log_record_t;

static uint8_t read_dispatch_rules(void)
{
    nvs_handle_t handle;
    uint8_t channels = DISPATCH_DEFAULT_CHANNELS;
    if (nvs_open("safety", NVS_READONLY, &handle) == ESP_OK) {
        (void)nvs_get_u8(handle, "dispatch_channels", &channels);
        nvs_close(handle);
    }
    return channels & (DISPATCH_SMS_BIT | DISPATCH_VOICE_BIT);
}

static void log_dispatch(const safety_event_t *event, uint8_t channels, const gnss_fix_t *fix)
{
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READWRITE, &handle) != ESP_OK) return;
    uint8_t next = 0;
    (void)nvs_get_u8(handle, "dispatch_log_next", &next);
    char key[12];
    snprintf(key, sizeof(key), "dsp_log%u", next % DISPATCH_LOG_SLOTS);
    dispatch_log_record_t record = {
        .uptime_seconds = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ),
        .event_type = (uint8_t)event->type,
        .channels = channels,
        .gnss_valid = fix->valid,
        .hdop = fix->hdop,
    };
    (void)nvs_set_blob(handle, key, &record, sizeof(record));
    (void)nvs_set_u8(handle, "dispatch_log_next", (uint8_t)((next + 1) % DISPATCH_LOG_SLOTS));
    (void)nvs_commit(handle);
    nvs_close(handle);
}

static gnss_fix_t snapshot_gnss(void)
{
    gnss_fix_t fix = {0};
    if (xSemaphoreTake(s_gnss_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        fix = s_gnss_fix;
        xSemaphoreGive(s_gnss_mutex);
    }
    return fix;
}

static void emergency_dispatch_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Emergency dispatch task started");
    for (;;) {
        safety_event_t event;
        if (xQueueReceive(s_dispatch_events, &event, portMAX_DELAY) != pdTRUE) continue;

        /* This is the central NORMAL -> EMERGENCY_ACTIVE transition. */
        s_emergency_latched = true;
        xEventGroupSetBits(s_system_events, BIT_EMERGENCY);

        /* Request a fresh GNSS acquisition. The GNSS worker owns normal
         * polling; this direct query wakes the modem path immediately. */
        if (!gl868_modem_enable_gnss()) ESP_LOGW(TAG, "Unable to wake GNSS");
        char raw_gnss[256] = {0};
        if (gl868_modem_get_gps_now(raw_gnss, sizeof(raw_gnss))) {
            gnss_task_process_response(raw_gnss);
        }
        gnss_fix_t fix = snapshot_gnss();
        ESP_LOGI(TAG, "Dispatch event %d, GNSS valid=%d HDOP=%.1f UTC=%s",
                 event.type, fix.valid, fix.hdop, fix.utc);

        uint8_t channels = read_dispatch_rules();
        log_dispatch(&event, channels, &fix);
        if ((channels & DISPATCH_SMS_BIT) && xQueueSend(s_sms_events, &event, 0) != pdPASS) {
            ESP_LOGW(TAG, "SMS dispatch queue full");
        }
        if ((channels & DISPATCH_VOICE_BIT) && xQueueSend(s_voice_events, &event, 0) != pdPASS) {
            ESP_LOGW(TAG, "Voice dispatch queue full");
        }
    }
}

void emergency_dispatch_task_start(void)
{
    xTaskCreate(emergency_dispatch_task, "emergency_dispatch", 6144, NULL, 11, NULL);
}
