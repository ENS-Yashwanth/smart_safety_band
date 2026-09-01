#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
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
#define COMMUNICATION_QUEUE_DEPTH 8
#define GPS_UPDATE_INTERVAL_MS (3 * 60 * 1000)
#define SOS_DEBOUNCE_MS 60

#define BIT_MODEM_READY BIT0
#define BIT_EMERGENCY BIT1

static const char *TAG = "SMART_SAFETY_BAND_001";

typedef enum { COMM_EVENT_EMERGENCY, COMM_EVENT_GPS_UPLOAD, COMM_EVENT_LIVE_TRACKING } communication_event_type_t;
typedef struct { communication_event_type_t type; const char *source; } communication_event_t;

static SemaphoreHandle_t s_i2c_mutex;
static SemaphoreHandle_t s_sos_sem;
static QueueHandle_t s_communication_events;
static EventGroupHandle_t s_system_events;
static const int s_sos_button_idle_level = 1;
static const int s_sos_button_active_level = 0;
static int s_sos_button_last_level = 1;

static void queue_communication_event(communication_event_type_t type, const char *source)
{
    const communication_event_t event = {.type = type, .source = source};
    if (xQueueSend(s_communication_events, &event, 0) != pdPASS) {
        ESP_LOGW(TAG, "Communication queue full; dropped %s", source);
    }
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
            gl868_modem_trigger_emergency(event.source, 0);
        } else if (event.type == COMM_EVENT_GPS_UPLOAD) {
            double latitude = 0.0;
            double longitude = 0.0;
            if (!gl868_modem_get_gps_coordinates(&latitude, &longitude)) {
                ESP_LOGW(TAG, "No valid GPS fix");
                continue;
            }
            const int battery = gl868_modem_get_battery_percent();
            ESP_LOGI(TAG, "GPS sample: %.6f,%.6f (battery=%d%%)", latitude, longitude, battery);
        } else if (event.type == COMM_EVENT_LIVE_TRACKING) {
            ESP_LOGI(TAG, "Sending scheduled live location");
            if (!gl868_modem_send_live_location()) {
                ESP_LOGW(TAG, "Scheduled live location was not sent");
            }
        }
    }
}

/* Schedules three-minute live-location SMS updates without competing for modem UART. */
static void gps_task(void *argument)
{
    xEventGroupWaitBits(s_system_events, BIT_MODEM_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    for (;;) {
        queue_communication_event(COMM_EVENT_LIVE_TRACKING, "three-minute live tracking update");
        vTaskDelay(pdMS_TO_TICKS(GPS_UPDATE_INTERVAL_MS));
    }
}

static __attribute__((unused)) void sos_button_task(void *argument)
{
    int last_level = gpio_get_level(SOS_BUTTON_GPIO);
    for (;;) {
        int level = gpio_get_level(SOS_BUTTON_GPIO);
        if (level != last_level) {
            if (level == s_sos_button_active_level) {
                ESP_LOGW(TAG, "SOS button became active (press detected)");
                queue_communication_event(COMM_EVENT_EMERGENCY, "SOS button");
            } else if (level == s_sos_button_idle_level) {
                ESP_LOGI(TAG, "SOS button became idle (release detected)");
            } else {
                ESP_LOGI(TAG, "SOS button unusual level: %d", level);
            }
            last_level = level;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    s_i2c_mutex = xSemaphoreCreateMutex(); 
    s_sos_sem = xSemaphoreCreateBinary();
    s_communication_events = xQueueCreate(COMMUNICATION_QUEUE_DEPTH, sizeof(communication_event_t));
    s_system_events = xEventGroupCreate();
    configASSERT(s_i2c_mutex && s_sos_sem && s_communication_events && s_system_events);
    init_io();
    xTaskCreate(communication_task, "communication", 6144, NULL, 10, NULL);
    ESP_LOGI(TAG, "Communication task started for modem UART access");
    xTaskCreate(sos_button_task, "sos_button", 2048, NULL, 8, NULL);
    ESP_LOGI(TAG, "SOS button task started on GPIO %d", SOS_BUTTON_GPIO);
    xTaskCreate(gps_task, "gps_upload", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "GPS live-tracking task started; updates every 3 minutes");
}
