#ifndef SAFETY_MANAGER_TASK_H
#define SAFETY_MANAGER_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void safety_manager_task(void *argument);
void safety_manager_start(TaskHandle_t comm);

#endif // SAFETY_MANAGER_TASK_H
