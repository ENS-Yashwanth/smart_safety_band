#pragma once

#ifdef __cplusplus
extern "C" {
#endif

bool gl868_modem_init(void);
void gl868_modem_update(void);
void gl868_modem_trigger_emergency(const char *source, int32_t value);
bool gl868_modem_send_at_command(const char *command, char *response, size_t response_len, uint32_t timeout_ms);
void gl868_modem_run_diagnostics(void);
void gl868_modem_run_full_diagnostics(void);
bool gl868_modem_send_test_sms(const char *message);
bool gl868_modem_send_sms_to(const char *number, const char *message);
bool gl868_modem_make_call_to(const char *number);
bool gl868_modem_get_gps_now(char *buf, size_t buf_len);
bool gl868_modem_get_gps_coordinates(double *latitude, double *longitude);
const char *gl868_modem_get_emergency_call_number(void);
const char *gl868_modem_get_emergency_sms_number(void);
int gl868_modem_get_battery_percent(void);
void gl868_modem_request_deferred_gps_upload(void);

#ifdef __cplusplus
}
#endif
