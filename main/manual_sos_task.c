#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"
#include "driver/gpio.h"

#include "safety_shared.h"

static const char *TAG = "manual_sos";

#ifndef CONFIG_SAFETY_BAND_SOS_GPIO
#define CONFIG_SAFETY_BAND_SOS_GPIO 4
#endif
#ifndef SOS_STATUS_LED_GPIO
#define SOS_STATUS_LED_GPIO GPIO_NUM_47
#endif
/* Set these board-specific GPIOs when the haptic driver and buzzer are fitted. */
#ifndef SOS_HAPTIC_GPIO
#define SOS_HAPTIC_GPIO GPIO_NUM_NC
#endif
#ifndef SOS_BUZZER_GPIO
#define SOS_BUZZER_GPIO GPIO_NUM_NC
#endif

#define SOS_DEBOUNCE_MS 50
#define SOS_VALID_HOLD_MS 3000
#define SOS_SAMPLE_MS 20
#define SOS_FEEDBACK_MS 200
#define SOS_LOG_SLOTS 8

typedef struct {
    uint32_t uptime_seconds;
    uint32_t hold_ms;
    uint8_t event_type;
} sos_log_record_t;

static void configure_optional_output(gpio_num_t pin)
{
    if (pin == GPIO_NUM_NC) return;
    gpio_config_t cfg = {.pin_bit_mask = (1ULL << pin), .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(pin, 0);
}

static void sos_feedback_task(void *arg)
{
    (void)arg;
    gpio_set_level(SOS_STATUS_LED_GPIO, 1);
    if (SOS_HAPTIC_GPIO != GPIO_NUM_NC) gpio_set_level(SOS_HAPTIC_GPIO, 1);
    /* Generate an approximately 500 Hz emergency tone for 200 ms. */
    for (unsigned i = 0; i < SOS_FEEDBACK_MS / 2; ++i) {
        if (SOS_BUZZER_GPIO != GPIO_NUM_NC) gpio_set_level(SOS_BUZZER_GPIO, i & 1U);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (SOS_HAPTIC_GPIO != GPIO_NUM_NC) gpio_set_level(SOS_HAPTIC_GPIO, 0);
    if (SOS_BUZZER_GPIO != GPIO_NUM_NC) gpio_set_level(SOS_BUZZER_GPIO, 0);
    gpio_set_level(SOS_STATUS_LED_GPIO, 0);
    vTaskDelete(NULL);
}

static void log_sos_activation(uint32_t hold_ms)
{
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READWRITE, &handle) != ESP_OK) return;
    uint8_t next = 0;
    (void)nvs_get_u8(handle, "sos_log_next", &next);
    char key[12];
    snprintf(key, sizeof(key), "soslog%u", next % SOS_LOG_SLOTS);
    sos_log_record_t record = {
        .uptime_seconds = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ),
        .hold_ms = hold_ms,
        .event_type = EVENT_MANUAL_SOS,
    };
    (void)nvs_set_blob(handle, key, &record, sizeof(record));
    (void)nvs_set_u8(handle, "sos_log_next", (uint8_t)((next + 1) % SOS_LOG_SLOTS));
    (void)nvs_commit(handle);
    nvs_close(handle);
}

static bool validate_sos_hold(uint32_t *hold_ms)
{
    /* The ISR fires on both edges. Confirm the active-low level after 50 ms,
     * then sample it with a timer-driven duration recognizer for three seconds. */
    vTaskDelay(pdMS_TO_TICKS(SOS_DEBOUNCE_MS));
    if (gpio_get_level(CONFIG_SAFETY_BAND_SOS_GPIO) != 0) return false;

    TickType_t pressed_at = xTaskGetTickCount();
    while (gpio_get_level(CONFIG_SAFETY_BAND_SOS_GPIO) == 0) {
        TickType_t elapsed = xTaskGetTickCount() - pressed_at;
        if (elapsed >= pdMS_TO_TICKS(SOS_VALID_HOLD_MS)) {
            *hold_ms = (uint32_t)(elapsed * portTICK_PERIOD_MS);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(SOS_SAMPLE_MS));
    }
    return false;
}

void manual_sos_task(void *argument)
{
    (void)argument;
    configure_optional_output(SOS_HAPTIC_GPIO);
    configure_optional_output(SOS_BUZZER_GPIO);
    ESP_LOGI(TAG, "SOS task ready: GPIO %d, valid hold %d ms", CONFIG_SAFETY_BAND_SOS_GPIO, SOS_VALID_HOLD_MS);

    while (true) {
        if (xSemaphoreTake(s_sos_sem, portMAX_DELAY) != pdTRUE) continue;
        if (gpio_get_level(CONFIG_SAFETY_BAND_SOS_GPIO) != 0) continue;

        uint32_t hold_ms = 0;
        if (!validate_sos_hold(&hold_ms)) {
            ESP_LOGI(TAG, "Ignored SOS press: held less than %d ms", SOS_VALID_HOLD_MS);
            continue;
        }

        ESP_LOGW(TAG, "Validated SOS hold (%lu ms); entering emergency state", (unsigned long)hold_ms);
        log_sos_activation(hold_ms);
        (void)xTaskCreate(sos_feedback_task, "sos_feedback", 2048, NULL, 10, NULL);
        publish_event(EVENT_MANUAL_SOS, (int32_t)hold_ms, "SOS button");

        /* Do not retrigger until the user releases the button; discard its
         * release-edge semaphore so the next press starts a fresh sequence. */
        while (gpio_get_level(CONFIG_SAFETY_BAND_SOS_GPIO) == 0) vTaskDelay(pdMS_TO_TICKS(SOS_SAMPLE_MS));
        while (xSemaphoreTake(s_sos_sem, 0) == pdTRUE) {}
    }
}

void manual_sos_start(TaskHandle_t comm)
{
    (void)comm;
    xTaskCreate(manual_sos_task, "manual_sos", 4096, NULL, 8, NULL);
}
