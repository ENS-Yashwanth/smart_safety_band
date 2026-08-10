#ifndef MANUAL_SOS_TASK_H
#define MANUAL_SOS_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void manual_sos_task(void *argument);
void manual_sos_start(TaskHandle_t comm);

#endif // MANUAL_SOS_TASK_H
