#ifndef COMMUNICATION_TASK_H
#define COMMUNICATION_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void communication_task(void *argument);
TaskHandle_t communication_start(void);

#endif // COMMUNICATION_TASK_H
