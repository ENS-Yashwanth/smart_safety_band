#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gl868_modem.h"

#ifndef CONFIG_SAFETY_BAND_SOS_GPIO
#define CONFIG_SAFETY_BAND_SOS_GPIO 4
#endif
#ifndef CONFIG_SAFETY_BAND_MOTION_INT_GPIO
#define CONFIG_SAFETY_BAND_MOTION_INT_GPIO 2
#endif
#ifndef CONFIG_SAFETY_BAND_SIMULATION
#define CONFIG_SAFETY_BAND_SIMULATION 0
#endif

/* GL868 reference pins. SOS and motion interrupt are menuconfig options. */
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_8
#define I2C_SCL_GPIO GPIO_NUM_9
#define STATUS_LED_GPIO GPIO_NUM_47
#define MODEM_UART UART_NUM_1
#define MODEM_TX_GPIO GPIO_NUM_17
#define MODEM_RX_GPIO GPIO_NUM_18
#define MODEM_POWER_GPIO GPIO_NUM_42
#define SOS_BUTTON_GPIO ((gpio_num_t)CONFIG_SAFETY_BAND_SOS_GPIO)
#define MOTION_INT_GPIO ((gpio_num_t)CONFIG_SAFETY_BAND_MOTION_INT_GPIO)
#define I2C_TIMEOUT_MS 100
#define COMMUNICATION_QUEUE_DEPTH 32
#define GPS_UPDATE_INTERVAL_MS (2 * 60 * 1000)
#define SOS_LOCATION_UPDATE_INTERVAL_MS (10 * 60 * 1000)
#define SOS_DEBOUNCE_MS 60

#define BIT_MODEM_READY BIT0
#define BIT_EMERGENCY BIT1

static const char *TAG = "SMART_SAFETY_BAND_001";

typedef enum { COMM_EVENT_EMERGENCY, COMM_EVENT_GPS_UPLOAD, COMM_EVENT_LIVE_TRACKING } communication_event_type_t;
typedef struct { communication_event_type_t type; const char *source; } communication_event_t;

static SemaphoreHandle_t s_sos_sem;
static QueueHandle_t s_communication_events;
static EventGroupHandle_t s_system_events;
static const int s_sos_button_idle_level = 1;
static const int s_sos_button_active_level = 0;
static int s_sos_button_last_level = 1;
static volatile bool s_sos_active = false;

static bool queue_communication_event(communication_event_type_t type, const char *source)
{
    if (type == COMM_EVENT_GPS_UPLOAD && uxQueueMessagesWaiting(s_communication_events) > 0) {
        return false;
    }

    const communication_event_t event = {.type = type, .source = source};
    if (xQueueSend(s_communication_events, &event, 0) != pdPASS) {
        ESP_LOGW(TAG, "Communication queue full; dropped %s", source);
        return false;
    }
    return true;
}

void gl868_modem_request_deferred_gps_upload(void)
{
    queue_communication_event(COMM_EVENT_GPS_UPLOAD, "deferred gps retry");
}

static void IRAM_ATTR sos_isr(void *argument)
{
    BaseType_t woke = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)argument, &woke);
    if (woke) portYIELD_FROM_ISR();
}

static void init_io(void)
{
    gpio_config_t output = {.pin_bit_mask = (1ULL << STATUS_LED_GPIO) | (1ULL << MODEM_POWER_GPIO), .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&output));
    gpio_set_level(STATUS_LED_GPIO, 0); gpio_set_level(MODEM_POWER_GPIO, 1);
    gpio_config_t input = {.pin_bit_mask = (1ULL << SOS_BUTTON_GPIO), .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_ANYEDGE};
    ESP_ERROR_CHECK(gpio_config(&input));
    int initial_level = gpio_get_level(SOS_BUTTON_GPIO);
    s_sos_button_last_level = initial_level;
    if (initial_level != s_sos_button_idle_level) {
        ESP_LOGW(TAG, "SOS GPIO %d booted in active state or is held low; check wiring and button contact", SOS_BUTTON_GPIO);
    }
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SOS_BUTTON_GPIO, sos_isr, s_sos_sem));
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(SOS_BUTTON_GPIO, s_sos_button_active_level));
}

/* The communication task is the only task that calls the modem API. This keeps
 * ESP-Modem's UART/DTE state serialized while SMS, calls, GPS and HTTP overlap. */
static void communication_task(void *argument)
{
    communication_event_t event;
    bool modem_ready = false;

    for (;;) {
        if (!modem_ready) {
            ESP_LOGI(TAG, "Powering and initializing SIM868 modem");
            modem_ready = gl868_modem_init();
            if (modem_ready) {
                xEventGroupSetBits(s_system_events, BIT_MODEM_READY);
                gl868_modem_set_status_led(true);
                ESP_LOGI(TAG, "SIM868 ready for emergency and GPS services");
                ESP_LOGI(TAG, "Boot complete. Emergency SMS recipient: %s", gl868_modem_get_emergency_sms_number());
                ESP_LOGI(TAG, "Boot complete. Emergency call recipient: %s", gl868_modem_get_emergency_call_number());
            } else {
                ESP_LOGW(TAG, "SIM868 initialization failed; retrying in 15 seconds");
                vTaskDelay(pdMS_TO_TICKS(15000));
                continue;
            }
        }

        if (xQueueReceive(s_communication_events, &event, portMAX_DELAY) != pdTRUE) continue;
        if (event.type == COMM_EVENT_EMERGENCY) {
            ESP_LOGW(TAG, "SOS emergency received from %s", event.source);
            s_sos_active = true;
            if (!gl868_modem_register_network(45000)) {
                ESP_LOGW(TAG, "GSM registration after SOS wake failed; continuing emergency attempt");
            }
                gl868_modem_trigger_emergency(event.source);
            gl868_modem_sleep();
            s_sos_active = false;
            gl868_modem_set_status_led(true);
        } else if (event.type == COMM_EVENT_GPS_UPLOAD) {
            if (!gl868_modem_upload_telemetry(event.source)) {
                ESP_LOGW(TAG, "HTTP telemetry upload failed");
            }
        } else if (event.type == COMM_EVENT_LIVE_TRACKING) {
            ESP_LOGI(TAG, "Sending scheduled live location");
            if (!gl868_modem_upload_telemetry("live_tracking")) {
                ESP_LOGW(TAG, "Scheduled live location was not sent");
            }
        }
    }
}

static void sos_location_update_task(void *argument)
{
    xEventGroupWaitBits(s_system_events, BIT_MODEM_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SOS_LOCATION_UPDATE_INTERVAL_MS);
    for (;;) {
        if (s_sos_active) {
            queue_communication_event(COMM_EVENT_GPS_UPLOAD, "sos.updated");
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

/* Schedules two-minute live-location uploads without competing for modem UART. */
static void gps_task(void *argument)
{
    xEventGroupWaitBits(s_system_events, BIT_MODEM_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(GPS_UPDATE_INTERVAL_MS);
    for (;;) {
        queue_communication_event(COMM_EVENT_LIVE_TRACKING, "two-minute live tracking update");
        vTaskDelayUntil(&last_wake, period);
    }
}

static void sos_button_task(void *argument)
{
    (void)argument;
    xEventGroupWaitBits(s_system_events, BIT_MODEM_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    for (;;) {
        if (s_sos_active) {
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        s_sos_button_last_level = gpio_get_level(SOS_BUTTON_GPIO);
        if (s_sos_button_last_level == s_sos_button_active_level) {
            vTaskDelay(pdMS_TO_TICKS(SOS_DEBOUNCE_MS));
            continue;
        }

        ESP_LOGI(TAG, "Entering ESP32 light sleep; SIM868 modem is already sleeping");
        esp_err_t sleep_error = esp_light_sleep_start();
        if (sleep_error != ESP_OK) {
            ESP_LOGW(TAG, "Light sleep failed: %s", esp_err_to_name(sleep_error));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (gpio_get_level(SOS_BUTTON_GPIO) == s_sos_button_active_level) {
            vTaskDelay(pdMS_TO_TICKS(SOS_DEBOUNCE_MS));
            if (gpio_get_level(SOS_BUTTON_GPIO) == s_sos_button_active_level) {
                ESP_LOGW(TAG, "SOS button woke device; requesting emergency response");
                queue_communication_event(COMM_EVENT_EMERGENCY, "SOS button");
                while (gpio_get_level(SOS_BUTTON_GPIO) == s_sos_button_active_level) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }
        }
    }
}

void app_main(void)
{
    s_sos_sem = xSemaphoreCreateBinary();
    s_communication_events = xQueueCreate(COMMUNICATION_QUEUE_DEPTH, sizeof(communication_event_t));
    s_system_events = xEventGroupCreate();
    configASSERT(s_sos_sem && s_communication_events && s_system_events);
    init_io();
    xTaskCreate(communication_task, "communication", 6144, NULL, 10, NULL);
    ESP_LOGI(TAG, "Communication task started for modem UART access");
    xTaskCreate(sos_button_task, "sos_button", 2048, NULL, 8, NULL);
    ESP_LOGI(TAG, "SOS button task started on GPIO %d", SOS_BUTTON_GPIO);
    ESP_LOGI(TAG, "Periodic modem/GPS uploads disabled; modem sleeps until SOS");
}