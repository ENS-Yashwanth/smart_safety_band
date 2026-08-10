#ifndef SAFETY_SHARED_H
#define SAFETY_SHARED_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

typedef enum { EVENT_BOOT, EVENT_MANUAL_SOS, EVENT_FALL, EVENT_MAN_DOWN, EVENT_GEOFENCE_BREACH, EVENT_LIVE_LOCATION, EVENT_SENSOR_MISSING } safety_event_type_t;
typedef struct { safety_event_type_t type; int32_t value; const char *source; } safety_event_t;

typedef struct {
    bool valid;
    bool powered;
    double latitude;
    double longitude;
    float hdop;
    float speed_kph;
    float altitude_m;
    float bearing_degrees;
    int32_t satellites_in_view;
    int32_t satellites_used;
    int32_t fix_mode;
    TickType_t timestamp;
    char utc[24];
    char raw_info[128];
} gnss_fix_t;

#define BIT_MODEM_READY BIT0
#define BIT_EMERGENCY BIT1

extern SemaphoreHandle_t s_sos_sem;
extern QueueHandle_t s_safety_events;
extern QueueHandle_t s_voice_events;
extern QueueHandle_t s_sms_events;
extern QueueHandle_t s_dispatch_events;
extern QueueHandle_t s_telemetry;
extern EventGroupHandle_t s_system_events;
extern SemaphoreHandle_t s_gnss_mutex;
extern gnss_fix_t s_gnss_fix;
extern bool s_emergency_latched;

void publish_event(safety_event_type_t type, int32_t value, const char *source);
void safety_emergency_deactivate_authorized(void);
void voice_call_task_start(void);
void gnss_task_start(void);

#endif // SAFETY_SHARED_H
