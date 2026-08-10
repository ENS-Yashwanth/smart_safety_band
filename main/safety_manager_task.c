#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "gl868_modem.h"
#include "safety_shared.h"

static const char *TAG = "safety_manager";

void safety_manager_task(void *argument)
{
    safety_event_t event;
    while (true) {
        if (xQueueReceive(s_safety_events, &event, portMAX_DELAY) == pdTRUE) {
            if (event.type == EVENT_MANUAL_SOS || event.type == EVENT_FALL ||
                event.type == EVENT_MAN_DOWN || event.type == EVENT_GEOFENCE_BREACH) {
                ESP_LOGI(TAG, "EMERGENCY: %s (%ld)", event.source, (long)event.value);
                if (xQueueSend(s_dispatch_events, &event, 0) != pdPASS) {
                    ESP_LOGW(TAG, "Emergency dispatch queue full; request dropped");
                }
                xTaskNotifyGive((TaskHandle_t)argument);
            } else if (event.type == EVENT_SENSOR_MISSING) {
                ESP_LOGW(TAG, "Optional sensor missing: %s", event.source);
            }
        }
    }
}

void safety_manager_start(TaskHandle_t comm)
{
    xTaskCreate(safety_manager_task, "safety_manager", 4096, comm, 9, NULL);
}

void safety_emergency_deactivate_authorized(void)
{
    s_emergency_latched = false;
    xEventGroupClearBits(s_system_events, BIT_EMERGENCY);
    gl868_modem_shutdown();
    ESP_LOGW(TAG, "Emergency state cleared by authorized command");
}
