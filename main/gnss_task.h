#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void gnss_task_start(void);
/* Accept an immediate SIM868 +CGNSINF response from the emergency dispatcher. */
void gnss_task_process_response(const char *response);
void gnss_task_apply_filtered_location(double latitude, double longitude);
bool gnss_task_set_normal_reporting_interval(uint32_t seconds);

#ifdef __cplusplus
}
#endif
