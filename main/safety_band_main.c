#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
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
#define GPS_UPDATE_INTERVAL_MS (2 * 60 * 1000)
#define SOS_DEBOUNCE_MS 60
#define RESOURCE_MONITOR_INTERVAL_MS (5 * 1000)

#ifndef CONFIG_SAFETY_BAND_POWER_MANAGEMENT
#define CONFIG_SAFETY_BAND_POWER_MANAGEMENT 1
#endif
#ifndef CONFIG_SAFETY_BAND_PM_MIN_FREQ_MHZ
#define CONFIG_SAFETY_BAND_PM_MIN_FREQ_MHZ 40
#endif
#ifndef CONFIG_SAFETY_BAND_GNSS_POWER_SAVE
#define CONFIG_SAFETY_BAND_GNSS_POWER_SAVE 1
#endif

#define BIT_MODEM_READY BIT0
#define BIT_EMERGENCY_PENDING BIT1
#define BIT_GPS_UPLOAD_PENDING BIT2

static const char *TAG = "SMART_SAFETY_BAND_001";

static SemaphoreHandle_t s_i2c_mutex;
static SemaphoreHandle_t s_sos_sem;
static EventGroupHandle_t s_system_events;
#if CONFIG_PM_ENABLE && CONFIG_SAFETY_BAND_POWER_MANAGEMENT
static esp_pm_lock_handle_t s_modem_no_light_sleep_lock;
static esp_pm_lock_handle_t s_modem_apb_lock;
#endif
static const int s_sos_button_idle_level = 1;
static const int s_sos_button_active_level = 0;

/* Event bits */
static void signal_communication_event(EventBits_t event_bit)
{
    xEventGroupSetBits(s_system_events, event_bit);
}

static void modem_power_lock_acquire(void)
{
#if CONFIG_PM_ENABLE && CONFIG_SAFETY_BAND_POWER_MANAGEMENT
    ESP_ERROR_CHECK(esp_pm_lock_acquire(s_modem_no_light_sleep_lock));
    ESP_ERROR_CHECK(esp_pm_lock_acquire(s_modem_apb_lock));
#endif
}

static void modem_power_lock_release(void)
{
#if CONFIG_PM_ENABLE && CONFIG_SAFETY_BAND_POWER_MANAGEMENT
    ESP_ERROR_CHECK(esp_pm_lock_release(s_modem_apb_lock));
    ESP_ERROR_CHECK(esp_pm_lock_release(s_modem_no_light_sleep_lock));
#endif
}

static void init_power_management(void)
{
#if CONFIG_PM_ENABLE && CONFIG_SAFETY_BAND_POWER_MANAGEMENT
    const esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_SAFETY_BAND_PM_MIN_FREQ_MHZ,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "sim868", &s_modem_no_light_sleep_lock));
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "sim868_uart", &s_modem_apb_lock));
    ESP_ERROR_CHECK(gpio_wakeup_enable(SOS_BUTTON_GPIO, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_LOGI(TAG, "Power management enabled: %d-%d MHz with automatic light sleep",
             CONFIG_SAFETY_BAND_PM_MIN_FREQ_MHZ, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
#else
    ESP_LOGW(TAG, "Automatic light sleep disabled; enable CONFIG_PM_ENABLE and CONFIG_FREERTOS_USE_TICKLESS_IDLE");
#endif
}

static void enter_gnss_idle_power_save(void)
{
#if CONFIG_SAFETY_BAND_GNSS_POWER_SAVE
    if (!gl868_modem_set_gnss_power(false)) {
        ESP_LOGW(TAG, "Could not enter GNSS power-save mode");
    }
#endif
}

void gl868_modem_request_deferred_gps_upload(void)
{
    signal_communication_event(BIT_GPS_UPLOAD_PENDING);
}
/*
static void log_ram_usage(void)
{
    ESP_LOGI(TAG,
             "RAM: free=%" PRIu32 " B, minimum-free=%" PRIu32
             " B, internal-free=%" PRIu32 " B",
             esp_get_free_heap_size(),
             esp_get_minimum_free_heap_size(),
             esp_get_free_internal_heap_size());
}

#if (configUSE_TRACE_FACILITY == 1) && defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS == 1)
static void log_task_runtime_stats(void)
{
    const UBaseType_t task_count = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_status = pvPortMalloc(task_count * sizeof(*task_status));
    if (task_status == NULL) {
        ESP_LOGW(TAG, "CPU statistics skipped: insufficient heap for task snapshot");
        return;
    }

    configRUN_TIME_COUNTER_TYPE total_runtime = 0;
    const UBaseType_t captured = uxTaskGetSystemState(task_status, task_count, &total_runtime);
    ESP_LOGI(TAG, "CPU task statistics (runtime since boot):");
    for (UBaseType_t i = 0; i < captured; ++i) {
        const unsigned int cpu_percent = total_runtime == 0 ? 0 :
            (unsigned int)((task_status[i].ulRunTimeCounter * 100ULL) / total_runtime);
        ESP_LOGI(TAG, "  %-16s CPU=%u%% stack-low-water=%u B",
                 task_status[i].pcTaskName,
                 cpu_percent,
                 (unsigned int)task_status[i].usStackHighWaterMark);
    }
    vPortFree(task_status);
}
#else
static void log_task_runtime_stats(void)
{
    ESP_LOGW(TAG,
             "CPU statistics disabled. Enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS in menuconfig.");
}
#endif


// Runs at the lowest application priority so monitoring never delays SOS work. 
static void resource_monitor_task(void *argument)
{
    for (;;) {
        log_ram_usage();
        log_task_runtime_stats();
        vTaskDelay(pdMS_TO_TICKS(RESOURCE_MONITOR_INTERVAL_MS));
    }
}
*/
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
    ESP_LOGI(TAG, "SOS GPIO %d initial level: %d; expected idle=%d, active=%d",
             SOS_BUTTON_GPIO, initial_level, s_sos_button_idle_level, s_sos_button_active_level);
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
    bool modem_ready = false;

    for (;;) {
        if (!modem_ready) {
            ESP_LOGI(TAG, "Powering and initializing SIM868 modem");
            modem_power_lock_acquire();
            modem_ready = gl868_modem_init();
            modem_power_lock_release();
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

        const EventBits_t pending = xEventGroupWaitBits(
            s_system_events,
            BIT_EMERGENCY_PENDING | BIT_GPS_UPLOAD_PENDING,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY);

        /* SOS always wins. Retain a simultaneous GPS request for processing
         * after the emergency flow completes. */
        if (pending & BIT_EMERGENCY_PENDING) {
            if (pending & BIT_GPS_UPLOAD_PENDING) {
                signal_communication_event(BIT_GPS_UPLOAD_PENDING);
            }
            ESP_LOGW(TAG, "SOS emergency received from SOS button");
            modem_power_lock_acquire();
            gl868_modem_trigger_emergency("SOS button", 0);
            enter_gnss_idle_power_save();
            modem_power_lock_release();
        } else if (pending & BIT_GPS_UPLOAD_PENDING) {
            double latitude = 0.0;
            double longitude = 0.0;
            modem_power_lock_acquire();
            if (!gl868_modem_get_gps_coordinates(&latitude, &longitude)) {
                ESP_LOGW(TAG, "No valid GPS fix");
            } else {
                const int battery = gl868_modem_get_battery_percent();
                ESP_LOGI(TAG, "GPS sample: %.6f,%.6f (battery=%d%%)", latitude, longitude, battery);
            }
            enter_gnss_idle_power_save();
            modem_power_lock_release();
        }
    }
}

/* Schedules two-minute live-location uploads without competing for modem UART. */
static void gps_task(void *argument)
{
    xEventGroupWaitBits(s_system_events, BIT_MODEM_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    for (;;) {
        signal_communication_event(BIT_GPS_UPLOAD_PENDING);
        vTaskDelay(pdMS_TO_TICKS(GPS_UPDATE_INTERVAL_MS));
    }
}

static void sos_button_task(void *argument)
{
    ESP_LOGI(TAG, "SOS button task waiting for GPIO interrupts");
    for (;;) {
        /* The ISR only wakes this task; debounce and GPIO access remain in
         * normal task context. */
        if (xSemaphoreTake(s_sos_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(SOS_DEBOUNCE_MS));
        if (gpio_get_level(SOS_BUTTON_GPIO) == s_sos_button_active_level) {
            ESP_LOGW(TAG, "SOS button press detected");
            signal_communication_event(BIT_EMERGENCY_PENDING);
        }
    }
}

void app_main(void)
{
    s_i2c_mutex = xSemaphoreCreateMutex(); 
    s_sos_sem = xSemaphoreCreateBinary();
    s_system_events = xEventGroupCreate();
    configASSERT(s_i2c_mutex && s_sos_sem && s_system_events);
    init_io();
    init_power_management();
    xTaskCreate(communication_task, "communication", 6144, NULL, 10, NULL);
    ESP_LOGI(TAG, "Communication task started for modem UART access");
    xTaskCreate(sos_button_task, "sos_button", 2048, NULL, 8, NULL);
    ESP_LOGI(TAG, "SOS button task started on GPIO %d", SOS_BUTTON_GPIO);
    xTaskCreate(gps_task, "gps_upload", 2048, NULL, 4, NULL);
    ESP_LOGI(TAG, "GPS task started for location updates");
    //xTaskCreate(resource_monitor_task, "resource_monitor", 2048, NULL, 1, NULL);
    //ESP_LOGI(TAG, "CPU and RAM resource monitor started (5 second interval)");
}