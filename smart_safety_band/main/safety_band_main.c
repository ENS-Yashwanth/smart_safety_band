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
#define GPS_UPDATE_INTERVAL_MS (2 * 60 * 1000)
#define SOS_DEBOUNCE_MS 60

#define BIT_MODEM_READY BIT0
#define BIT_EMERGENCY BIT1

static const char *TAG = "SMART_SAFETY_BAND_001";

typedef enum { MOTION_NONE, MOTION_LSM6DSOX, MOTION_LIS3DH } motion_sensor_t;
typedef struct { int32_t x_mg, y_mg, z_mg, magnitude_mg; } acceleration_sample_t;
typedef enum { COMM_EVENT_EMERGENCY, COMM_EVENT_GPS_UPLOAD } communication_event_type_t;
typedef struct { communication_event_type_t type; const char *source; } communication_event_t;
typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t motion;
    motion_sensor_t motion_type;
    bool bme68x_present, scd4x_present, veml6075_present, as3935_present;
    bool pulse_ox_present, skin_temp_present, gsr_present;
} sensor_bus_t;

static sensor_bus_t s_sensors;
static SemaphoreHandle_t s_i2c_mutex;
static SemaphoreHandle_t s_sos_sem;
static QueueHandle_t s_communication_events;
static EventGroupHandle_t s_system_events;

static void queue_communication_event(communication_event_type_t type, const char *source)
{
    const communication_event_t event = {.type = type, .source = source};
    if (xQueueSend(s_communication_events, &event, 0) != pdPASS) {
        ESP_LOGW(TAG, "Communication queue full; dropped %s", source);
    }
}

static esp_err_t motion_read(uint8_t reg, uint8_t *data, size_t length)
{
    if (!s_sensors.motion) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t result = i2c_master_transmit_receive(s_sensors.motion, &reg, 1, data, length, I2C_TIMEOUT_MS);
    xSemaphoreGive(s_i2c_mutex);
    return result;
}

static __attribute__((unused)) esp_err_t motion_write(uint8_t reg, uint8_t value)
{
    uint8_t data[] = {reg, value};
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) return ESP_ERR_TIMEOUT;
    esp_err_t result = i2c_master_transmit(s_sensors.motion, data, sizeof(data), I2C_TIMEOUT_MS);
    xSemaphoreGive(s_i2c_mutex);
    return result;
}

static bool i2c_probe(uint8_t address)
{
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) return false;
    esp_err_t result = i2c_master_probe(s_sensors.bus, address, I2C_TIMEOUT_MS);
    xSemaphoreGive(s_i2c_mutex);
    return result == ESP_OK;
}

static __attribute__((unused)) bool any_present(const uint8_t *addresses, size_t count)
{
    for (size_t i = 0; i < count; ++i) if (i2c_probe(addresses[i])) return true;
    return false;
}

static __attribute__((unused)) bool add_motion_device(const uint8_t *addresses, size_t count, motion_sensor_t type)
{
    for (size_t i = 0; i < count; ++i) {
        if (!i2c_probe(addresses[i])) continue;
        i2c_device_config_t config = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addresses[i], .scl_speed_hz = 400000};
        if (i2c_master_bus_add_device(s_sensors.bus, &config, &s_sensors.motion) == ESP_OK) {
            s_sensors.motion_type = type;
            ESP_LOGI(TAG, "Motion sensor found at 0x%02X", addresses[i]);
            return true;
        }
    }
    return false;
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
                ESP_LOGI(TAG, "SIM868 ready for emergency, GPS and GPRS services");
                ESP_LOGI(TAG, "Boot complete. Emergency SMS recipient(s): %s", gl868_modem_get_emergency_sms_number());
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
                ESP_LOGW(TAG, "No valid GPS fix; GeoLinker upload deferred");
                continue;
            }
            const int battery = gl868_modem_get_battery_percent();
            if (!gl868_modem_send_geolinker_location(latitude, longitude, battery)) {
                ESP_LOGW(TAG, "GeoLinker upload failed; it will be retried at the next interval");
            }
        }
    }
}

/* Schedules two-minute live-location uploads without competing for modem UART. */
static void gps_task(void *argument)
{
    xEventGroupWaitBits(s_system_events, BIT_MODEM_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    for (;;) {
        queue_communication_event(COMM_EVENT_GPS_UPLOAD, "two-minute GPS update");
        vTaskDelay(pdMS_TO_TICKS(GPS_UPDATE_INTERVAL_MS));
    }
}

static void sos_button_task(void *argument)
{
    for (;;) {
        xSemaphoreTake(s_sos_sem, portMAX_DELAY);
        if (gpio_get_level(SOS_BUTTON_GPIO) != 0) continue;
        vTaskDelay(pdMS_TO_TICKS(SOS_DEBOUNCE_MS));
        if (gpio_get_level(SOS_BUTTON_GPIO) == 0) {
            ESP_LOGW(TAG, "SOS button pressed; sending emergency SMS and call");
            queue_communication_event(COMM_EVENT_EMERGENCY, "SOS button");
            while (gpio_get_level(SOS_BUTTON_GPIO) == 0) {
                xSemaphoreTake(s_sos_sem, pdMS_TO_TICKS(100));
            }
        }
    }
}

static uint32_t integer_sqrt(uint32_t value)
{
    uint32_t result = 0;
    uint32_t bit = 1UL << 30;
    while (bit > value) bit >>= 2;
    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else result >>= 1;
        bit >>= 2;
    }
    return result;
}

bool motion_read_acceleration(acceleration_sample_t *sample)
{
    uint8_t raw[6];
    /* LIS3DH requires bit 7 for multi-register address auto-increment. */
    uint8_t data_register = s_sensors.motion_type == MOTION_LIS3DH ? 0xA8 : 0x28;
    if (s_sensors.motion_type == MOTION_NONE || motion_read(data_register, raw, sizeof(raw)) != ESP_OK) return false;
    int16_t x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    int16_t y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    int16_t z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
    if (s_sensors.motion_type == MOTION_LIS3DH) {
        x >>= 4;
        y >>= 4;
        z >>= 4;
    }
    int32_t x_mg = s_sensors.motion_type == MOTION_LSM6DSOX ? (x * 61) / 1000 : x;
    int32_t y_mg = s_sensors.motion_type == MOTION_LSM6DSOX ? (y * 61) / 1000 : y;
    int32_t z_mg = s_sensors.motion_type == MOTION_LSM6DSOX ? (z * 61) / 1000 : z;
    sample->x_mg = x_mg;
    sample->y_mg = y_mg;
    sample->z_mg = z_mg;
    sample->magnitude_mg = (int32_t)integer_sqrt((uint32_t)(x_mg * x_mg + y_mg * y_mg + z_mg * z_mg));
    return true;
}

// Expose a simple C API to get the current motion magnitude in mg.
int32_t get_motion_magnitude_mg(void)
{
    acceleration_sample_t sample;
    if (!motion_read_acceleration(&sample)) return 0;
    return sample.magnitude_mg;
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
    xTaskCreate(sos_button_task, "sos_button", 2048, NULL, 8, NULL);
    xTaskCreate(gps_task, "gps_upload", 2048, NULL, 4, NULL);
}
