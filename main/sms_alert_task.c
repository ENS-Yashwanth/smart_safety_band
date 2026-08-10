#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"

#include "gl868_modem.h"
#include "safety_shared.h"

static const char *TAG = "SMS_ALERT";

#define SMS_RECIPIENT_MAX       48
#define SMS_PAYLOAD_MAX         256
#define SMS_RETRY_SLOTS         8
#define SMS_MAX_ATTEMPTS        5
#define SMS_RETRY_DELAY_MS      30000
#define SMS_LOG_SLOTS           8

typedef struct {
    uint8_t attempts;
    char recipient[SMS_RECIPIENT_MAX];
    char payload[SMS_PAYLOAD_MAX];
} sms_retry_record_t;

typedef struct {
    uint32_t uptime_seconds;
    uint8_t attempts;
    bool success;
    char recipient[SMS_RECIPIENT_MAX];
} sms_delivery_log_t;

static TickType_t s_retry_due[SMS_RETRY_SLOTS];

static bool valid_number(const char *number)
{
    size_t digits = 0;
    if (number == NULL || *number == '\0') return false;
    for (const char *p = number; *p; ++p) {
        if ((*p == '+' && p == number) || (*p >= '0' && *p <= '9')) {
            if (*p != '+') ++digits;
        } else {
            return false;
        }
    }
    return digits >= 3 && strlen(number) < SMS_RECIPIENT_MAX;
}

static size_t parse_recipients(const char *list, char recipients[][SMS_RECIPIENT_MAX], size_t capacity)
{
    size_t count = 0;
    while (list && *list && count < capacity) {
        while (*list == ' ' || *list == ',' || *list == ';') ++list;
        const char *end = list;
        while (*end && *end != ',' && *end != ';') ++end;
        size_t len = (size_t)(end - list);
        while (len && list[len - 1] == ' ') --len;
        if (len && len < SMS_RECIPIENT_MAX) {
            memcpy(recipients[count], list, len);
            recipients[count][len] = '\0';
            if (valid_number(recipients[count])) ++count;
            else ESP_LOGW(TAG, "Ignoring invalid SMS recipient");
        }
        list = end;
    }
    return count;
}

static size_t load_recipients(char recipients[][SMS_RECIPIENT_MAX], size_t capacity)
{
    char stored[SMS_RECIPIENT_MAX * SMS_RETRY_SLOTS] = {0};
    size_t length = sizeof(stored);
    nvs_handle_t handle;
    esp_err_t err = nvs_open("safety", NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_str(handle, "emergency_sms_numbers", stored, &length);
        if (err != ESP_OK) {
            length = sizeof(stored);
            err = nvs_get_str(handle, "emergency_sms_number", stored, &length);
        }
        nvs_close(handle);
    }
    if (err == ESP_OK) return parse_recipients(stored, recipients, capacity);
    return parse_recipients(gl868_modem_get_emergency_sms_number(), recipients, capacity);
}

static void timestamp_utc(const gnss_fix_t *fix, char *out, size_t out_len)
{
    if (fix->utc[0] != '\0') {
        snprintf(out, out_len, "%s", fix->utc);
        return;
    }
    time_t now = time(NULL);
    struct tm utc;
    if (now > 1577836800 && gmtime_r(&now, &utc) != NULL) {
        strftime(out, out_len, "%Y-%m-%dT%H:%M:%SZ", &utc);
        return;
    }
    snprintf(out, out_len, "uptime-%lus", (unsigned long)(xTaskGetTickCount() / configTICK_RATE_HZ));
}

static const char *event_name(safety_event_type_t type)
{
    switch (type) {
    case EVENT_MANUAL_SOS: return "SOS Triggered";
    case EVENT_FALL: return "Fall Detected";
    case EVENT_MAN_DOWN: return "Man Down Detected";
    case EVENT_GEOFENCE_BREACH: return "Geofence Breach";
    case EVENT_LIVE_LOCATION: return "Live Location Update";
    default: return "Emergency Alert";
    }
}

static void build_payload(const safety_event_t *event, char *payload, size_t payload_len)
{
    gnss_fix_t fix = {0};
    if (xSemaphoreTake(s_gnss_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        fix = s_gnss_fix;
        xSemaphoreGive(s_gnss_mutex);
    }
    char timestamp[32];
    timestamp_utc(&fix, timestamp, sizeof(timestamp));
    int battery = gl868_modem_get_battery_percent();
    if (fix.valid) {
        snprintf(payload, payload_len,
                 "ALERT: %s! UTC:%s Loc: %.6f,%.6f Map: https://maps.google.com/?q=%.6f,%.6f Batt: %d%%",
                 event_name(event->type), timestamp, fix.latitude, fix.longitude,
                 fix.latitude, fix.longitude, battery);
    } else {
        snprintf(payload, payload_len, "ALERT: %s! UTC:%s Loc: unavailable Batt: %d%%",
                 event_name(event->type), timestamp, battery);
    }
}

static void retry_key(char *key, size_t key_len, unsigned slot) { snprintf(key, key_len, "smsr%u", slot); }
static void log_key(char *key, size_t key_len, unsigned slot) { snprintf(key, key_len, "smsl%u", slot); }

static void log_outcome(const char *recipient, uint8_t attempts, bool success)
{
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READWRITE, &handle) != ESP_OK) return;
    uint8_t next = 0;
    (void)nvs_get_u8(handle, "sms_log_next", &next);
    sms_delivery_log_t record = {
        .uptime_seconds = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ),
        .attempts = attempts,
        .success = success,
    };
    snprintf(record.recipient, sizeof(record.recipient), "%s", recipient);
    char key[8];
    log_key(key, sizeof(key), next % SMS_LOG_SLOTS);
    (void)nvs_set_blob(handle, key, &record, sizeof(record));
    (void)nvs_set_u8(handle, "sms_log_next", (uint8_t)((next + 1) % SMS_LOG_SLOTS));
    (void)nvs_commit(handle);
    nvs_close(handle);
}

static void save_retry(const char *recipient, const char *payload, uint8_t attempts)
{
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READWRITE, &handle) != ESP_OK) return;
    uint8_t next = 0;
    (void)nvs_get_u8(handle, "sms_retry_next", &next);
    unsigned slot = next % SMS_RETRY_SLOTS;
    sms_retry_record_t record = {.attempts = attempts};
    snprintf(record.recipient, sizeof(record.recipient), "%s", recipient);
    snprintf(record.payload, sizeof(record.payload), "%s", payload);
    char key[8];
    retry_key(key, sizeof(key), slot);
    if (nvs_set_blob(handle, key, &record, sizeof(record)) == ESP_OK) {
        (void)nvs_set_u8(handle, "sms_retry_next", (uint8_t)((next + 1) % SMS_RETRY_SLOTS));
        (void)nvs_commit(handle);
        s_retry_due[slot] = xTaskGetTickCount() + pdMS_TO_TICKS(SMS_RETRY_DELAY_MS);
    }
    nvs_close(handle);
}

static void clear_retry(unsigned slot)
{
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READWRITE, &handle) == ESP_OK) {
        char key[8];
        retry_key(key, sizeof(key), slot);
        (void)nvs_erase_key(handle, key);
        (void)nvs_commit(handle);
        nvs_close(handle);
    }
    s_retry_due[slot] = 0;
}

static void send_or_store(const char *recipient, const char *payload, uint8_t attempts)
{
    bool sent = gl868_modem_send_sms_to(recipient, payload);
    ESP_LOGI(TAG, "SMS %s for %s (attempt %u)", sent ? "delivered to modem" : "failed", recipient, (unsigned)attempts);
    log_outcome(recipient, attempts, sent);
    if (!sent && attempts < SMS_MAX_ATTEMPTS) save_retry(recipient, payload, attempts);
}

static void retry_pending(void)
{
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READONLY, &handle) != ESP_OK) return;
    for (unsigned slot = 0; slot < SMS_RETRY_SLOTS; ++slot) {
        if (s_retry_due[slot] != 0 && (int32_t)(xTaskGetTickCount() - s_retry_due[slot]) < 0) continue;
        char key[8];
        retry_key(key, sizeof(key), slot);
        sms_retry_record_t record = {0};
        size_t length = sizeof(record);
        if (nvs_get_blob(handle, key, &record, &length) != ESP_OK || length != sizeof(record)) continue;
        s_retry_due[slot] = 0;
        if (record.attempts >= SMS_MAX_ATTEMPTS) {
            log_outcome(record.recipient, record.attempts, false);
            nvs_close(handle);
            clear_retry(slot);
            if (nvs_open("safety", NVS_READONLY, &handle) != ESP_OK) return;
            continue;
        }
        nvs_close(handle);
        bool sent = gl868_modem_send_sms_to(record.recipient, record.payload);
        uint8_t next_attempt = record.attempts + 1;
        log_outcome(record.recipient, next_attempt, sent);
        if (sent) clear_retry(slot);
        else {
            nvs_handle_t write_handle;
            if (nvs_open("safety", NVS_READWRITE, &write_handle) == ESP_OK) {
                record.attempts = next_attempt;
                char write_key[8];
                retry_key(write_key, sizeof(write_key), slot);
                (void)nvs_set_blob(write_handle, write_key, &record, sizeof(record));
                (void)nvs_commit(write_handle);
                nvs_close(write_handle);
                s_retry_due[slot] = xTaskGetTickCount() + pdMS_TO_TICKS(SMS_RETRY_DELAY_MS);
            }
        }
        if (nvs_open("safety", NVS_READONLY, &handle) != ESP_OK) return;
    }
    nvs_close(handle);
}

static void sms_alert_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "SMS alert task started");
    for (;;) {
        safety_event_t event;
        if (xQueueReceive(s_sms_events, &event, pdMS_TO_TICKS(1000)) == pdTRUE) {
            char payload[SMS_PAYLOAD_MAX] = {0};
            char recipients[SMS_RETRY_SLOTS][SMS_RECIPIENT_MAX] = {{0}};
            build_payload(&event, payload, sizeof(payload));
            size_t count = load_recipients(recipients, SMS_RETRY_SLOTS);
            if (count == 0) {
                ESP_LOGE(TAG, "No valid emergency SMS recipients configured");
            }
            for (size_t i = 0; i < count; ++i) send_or_store(recipients[i], payload, 1);
        }
        retry_pending();
    }
}

void sms_alert_task_start(void)
{
    xTaskCreate(sms_alert_task, "sms_alert", 6144, NULL, 8, NULL);
}

void sms_alert_task_request_live_location(void)
{
    safety_event_t event = {.type = EVENT_LIVE_LOCATION, .value = 0, .source = "live location fallback"};
    if (xQueueSend(s_sms_events, &event, 0) != pdPASS) ESP_LOGW(TAG, "SMS queue full; live location update dropped");
}
