#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "gl868_modem.h"
#include "safety_shared.h"

static const char *TAG = "communication";

void communication_task(void *argument)
{
    ESP_LOGI(TAG, "Starting SIM868 modem bridge task");

    while (true) {
        gl868_modem_update();
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000));
    }
}

TaskHandle_t communication_start(void)
{
    TaskHandle_t handle = NULL;
    xTaskCreate(communication_task, "communication", 4096, NULL, 10, &handle);
    return handle;
}
