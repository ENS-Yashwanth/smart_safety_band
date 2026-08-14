#include <memory>
#include <string>
#include <cstring>
#include <cstdio>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_modem_config.h"
#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_dte.hpp"
#include "cxx_include/esp_modem_types.hpp"
#include "gl868_modem.h"

static const char *TAG = "sim868_bridge";

namespace {
static const char *DEFAULT_EMERGENCY_CALL_NUMBER = "+916309538622";
static const char *DEFAULT_EMERGENCY_SMS_NUMBER = "+916309538622";
static const uint32_t GPS_FIX_RETRY_DELAY_MS = 5000;
static const int GPS_FIX_RETRY_COUNT = 2;
// Accuracy thresholds and fallback behavior
static const double GPS_ACCEPTABLE_HDOP = 2.0; // lower is better
static const int GPS_MIN_SATELLITES = 4;
// When metadata is missing or accuracy is poor, apply exponential smoothing to reduce jump/noise
static const bool GPS_ENABLE_SMOOTHING_FALLBACK = true;
static const double GPS_SMOOTHING_ALPHA = 0.3; // EMA weight for new value


struct Sim868State {
    std::shared_ptr<esp_modem::DTE> dte;
    bool initialized = false;
    bool gps_enabled = false;
};

static Sim868State s_state;

struct GpsFixInfo;

static std::string trim_response(const std::string &input)
{
    const char *whitespace = "\r\n";
    const size_t start = input.find_first_not_of(whitespace);
    if (start == std::string::npos) {
        return "";
    }
    const size_t end = input.find_last_not_of(whitespace);
    return input.substr(start, end - start + 1);
}

static std::string flatten_response(const std::string &input)
{
    std::string out;
    out.reserve(input.size());
    for (char c : input) {
        if (c == '\r' || c == '\n') {
            out.push_back(' ');
        } else {
            out.push_back(c);
        }
    }
    return trim_response(out);
}

static void power_cycle_modem()
{
    gpio_set_level(GPIO_NUM_42, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
    gpio_set_level(GPIO_NUM_42, 1);
    vTaskDelay(pdMS_TO_TICKS(2500));
}

static bool send_at_command(const std::string &cmd, std::string *response, uint32_t timeout_ms, const char *success_marker = nullptr);
static bool wait_for_prompt(const std::string &cmd, const std::string &prompt, std::string *response, uint32_t timeout_ms, char separator = '\n');
static void log_sim_status(void);
static void log_call_activity_status(void);
static const char *get_emergency_call_number(void);
static const char *get_emergency_sms_number(void);
static std::vector<std::string> split_recipients(const std::string &list);
static bool run_at_step(const char *label, const std::string &command, uint32_t timeout_ms, const char *success_marker = nullptr);
static bool enable_gps(void);
static bool get_gps_location(std::string *response, uint32_t timeout_ms = 5000);
static bool is_valid_coordinate(double latitude, double longitude);
static bool wait_for_sim_ready(uint32_t timeout_ms = 15000);
static bool ensure_apn_configured(void);

static std::string gps_status_line(const std::string &response)
{
    const size_t start = response.find("+CGNSINF:");
    if (start == std::string::npos) return "no +CGNSINF response";
    const size_t end = response.find_first_of("\r\n", start);
    return trim_response(response.substr(start, end == std::string::npos ? std::string::npos : end - start));
}

static __attribute__((unused)) const char *get_geolinker_api_key(void)
{
#ifdef CONFIG_SAFETY_BAND_GEOLINKER_API_KEY
    return CONFIG_SAFETY_BAND_GEOLINKER_API_KEY;
#else
    return "";
#endif
}

static __attribute__((unused)) const char *get_geolinker_device_id(void)
{
#ifdef CONFIG_SAFETY_BAND_GEOLINKER_DEVICE_ID
    return CONFIG_SAFETY_BAND_GEOLINKER_DEVICE_ID[0] != '\0' ? CONFIG_SAFETY_BAND_GEOLINKER_DEVICE_ID : "smart_safety_band";
#else
    return "smart_safety_band";
#endif
}

static bool is_sim_ready(std::string *out_status = nullptr)
{
    std::string response;
    if (!send_at_command("AT+CPIN?\r", &response, 5000)) {
        if (out_status) *out_status = trim_response(response);
        return false;
    }
    const std::string trimmed = trim_response(response);
    if (out_status) *out_status = trimmed;
    return trimmed.find("READY") != std::string::npos;
}

static bool wait_for_sim_ready(uint32_t timeout_ms)
{
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    std::string sim_status;
    while (xTaskGetTickCount() < deadline) {
        if (is_sim_ready(&sim_status)) {
            ESP_LOGI(TAG, "SIM card ready: %s", sim_status.c_str());
            return true;
        }
        ESP_LOGW(TAG, "SIM not ready: %s; retrying", sim_status.c_str());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGW(TAG, "SIM readiness timeout: %s", sim_status.c_str());
    return false;
}

static bool ensure_apn_configured(void)
{
#ifdef CONFIG_SAFETY_BAND_GPRS_APN
    const char *apn = CONFIG_SAFETY_BAND_GPRS_APN;
    if (apn == nullptr || apn[0] == '\0') {
        ESP_LOGW(TAG, "No APN configured in sdkconfig; set SAFETY_BAND_GPRS_APN");
        return false;
    }

    std::string response;
    if (send_at_command("AT+CGDCONT?\r", &response, 5000)) {
        if (response.find(apn) != std::string::npos) {
            ESP_LOGI(TAG, "APN already configured: %s", apn);
            return true;
        }
    } else {
        ESP_LOGW(TAG, "Failed to query configured PDP contexts");
    }

    const std::string command = std::string("AT+CGDCONT=1,\"IP\",\"") + apn + "\"\r";
    if (!send_at_command(command, &response, 5000)) {
        ESP_LOGW(TAG, "APN configuration failed: %s", trim_response(response).c_str());
        return false;
    }
    ESP_LOGI(TAG, "APN configured: %s", apn);
    return true;
#else
    ESP_LOGW(TAG, "No APN configured in sdkconfig; set SAFETY_BAND_GPRS_APN");
    return false;
#endif
}

bool send_at_command(const std::string &cmd, std::string *response, uint32_t timeout_ms, const char *success_marker)
{
    if (!s_state.dte) {
        return false;
    }

    std::string buffer;
    bool marker_seen = false;
    auto result = s_state.dte->command(cmd, [&buffer,&marker_seen,success_marker](uint8_t *data, size_t len) -> esp_modem::command_result {
        buffer.append(reinterpret_cast<char *>(data), len);
        const std::string line(reinterpret_cast<char *>(data), len);
        const std::string trimmed = trim_response(line);
        if (trimmed.find("ERROR") != std::string::npos || trimmed.find("FAIL") != std::string::npos || trimmed.find("+CME ERROR") != std::string::npos || trimmed.find("+CMS ERROR") != std::string::npos) {
            return esp_modem::command_result::FAIL;
        }
        if (success_marker != nullptr && trimmed.find(success_marker) != std::string::npos) {
            marker_seen = true;
            return esp_modem::command_result::OK;
        }
        /* Commands such as AT+HTTPACTION first return OK, then emit their
         * asynchronous result URC. Do not finish at the first OK when a
         * marker was requested. */
        if (success_marker != nullptr) {
            return esp_modem::command_result::TIMEOUT;
        }
        if (trimmed.find("OK") != std::string::npos) {
            return esp_modem::command_result::OK;
        }
        return esp_modem::command_result::TIMEOUT;
    }, timeout_ms);

    if (response != nullptr) {
        *response = buffer;
    }
    if (result == esp_modem::command_result::OK) {
        return true;
    }
    if (success_marker != nullptr && marker_seen && buffer.find("ERROR") == std::string::npos && buffer.find("FAIL") == std::string::npos && buffer.find("+CME ERROR") == std::string::npos && buffer.find("+CMS ERROR") == std::string::npos) {
        return true;
    }
    return false;
}

static const char *get_emergency_call_number(void)
{
#ifdef CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER
    return CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER[0] != '\0' ? CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER : DEFAULT_EMERGENCY_CALL_NUMBER;
#else
    return DEFAULT_EMERGENCY_CALL_NUMBER;
#endif
}

static const char *get_emergency_sms_number(void)
{
    /* Force emergency SMS recipient to the hard-coded emergency SMS number.
     * Do not honor runtime or build-time SMS overrides for emergency alerts. */
    return DEFAULT_EMERGENCY_SMS_NUMBER;
}

static std::vector<std::string> split_recipients(const std::string &list)
{
    std::vector<std::string> out;
    size_t start = 0;
    while (start < list.size()) {
        // find next comma or semicolon
        size_t pos = list.find_first_of(",;", start);
        std::string token;
        if (pos == std::string::npos) {
            token = trim_response(list.substr(start));
            start = list.size();
        } else {
            token = trim_response(list.substr(start, pos - start));
            start = pos + 1;
        }
        if (!token.empty()) out.push_back(token);
    }
    return out;
}

struct GpsFixInfo {
    bool valid = false;
    int run_status = -1;
    int fix_status = -1;
    int fix_mode = -1;
    double latitude = 0.0;
    double longitude = 0.0;
    double hdop = -1.0;
    int satellites_in_view = -1;
    int satellites_used = -1;
    std::string timestamp;
};

static bool parse_gps_fix_info_from_cgnsinf(const std::string &cgnsinf, GpsFixInfo &info)
{
    const size_t pos = cgnsinf.find("+CGNSINF:");
    if (pos == std::string::npos) return false;
    const size_t line_end = cgnsinf.find('\n', pos);
    const std::string line = (line_end == std::string::npos) ? cgnsinf.substr(pos) : cgnsinf.substr(pos, line_end - pos);
    size_t colon = line.find(':');
    if (colon == std::string::npos) return false;
    std::string rest = line.substr(colon + 1);
    const std::string trimmed = trim_response(rest);

    std::vector<std::string> fields;
    size_t s = 0;
    while (s < trimmed.size()) {
        size_t p = trimmed.find(',', s);
        if (p == std::string::npos) { fields.push_back(trimmed.substr(s)); break; }
        fields.push_back(trimmed.substr(s, p - s));
        s = p + 1;
    }

    if (fields.size() < 3) return false;
    info.run_status = static_cast<int>(strtol(fields[0].c_str(), nullptr, 10));
    info.fix_status = static_cast<int>(strtol(fields[1].c_str(), nullptr, 10));
    info.timestamp = fields[2];

    if (info.fix_status <= 0) {
        info.valid = false;
        return true;
    }

    if (fields.size() < 5) return false;
    char *endptr = nullptr;
    info.latitude = strtod(fields[3].c_str(), &endptr);
    if (endptr == fields[3].c_str()) return false;
    info.longitude = strtod(fields[4].c_str(), &endptr);
    if (endptr == fields[4].c_str()) return false;
    if (!is_valid_coordinate(info.latitude, info.longitude)) {
        return false;
    }

    if (fields.size() > 8) {
        info.fix_mode = static_cast<int>(strtol(fields[8].c_str(), nullptr, 10));
    }
    if (fields.size() > 10) {
        info.hdop = strtod(fields[10].c_str(), &endptr);
        if (endptr == fields[10].c_str()) info.hdop = -1.0;
    }
    if (fields.size() > 14) {
        info.satellites_in_view = static_cast<int>(strtol(fields[14].c_str(), nullptr, 10));
    }
    if (fields.size() > 15) {
        info.satellites_used = static_cast<int>(strtol(fields[15].c_str(), nullptr, 10));
    }
    info.valid = true;
    return true;
}

/* Attempt to gather additional GNSS metadata from other vendor-specific AT responses.
 * This will try a set of common commands and extract HDOP and satellite counts
 * when present, merging them into the provided `info` structure.
 */
static bool fetch_additional_gnss_metadata(GpsFixInfo &info)
{
    bool found_any = false;
    const char *cmds[] = { "AT+CGNSPVT?\r", "AT+QGPSLOC?\r", "AT+CGPSINFO\r" };
    for (const char *cmd : cmds) {
        std::string resp;
        if (!send_at_command(cmd, &resp, 1500)) continue;
        const std::string r = trim_response(resp);
        // Try to parse as CGNSINF-like first
        GpsFixInfo tmp;
        if (r.find("+CGNSINF:") != std::string::npos) {
            if (parse_gps_fix_info_from_cgnsinf(r, tmp)) {
                if (tmp.hdop > 0.0 && info.hdop <= 0.0) { info.hdop = tmp.hdop; found_any = true; }
                if (tmp.satellites_used >= 0 && info.satellites_used < tmp.satellites_used) { info.satellites_used = tmp.satellites_used; found_any = true; }
                if (tmp.satellites_in_view >= 0 && info.satellites_in_view < tmp.satellites_in_view) { info.satellites_in_view = tmp.satellites_in_view; found_any = true; }
                continue;
            }
        }

        // Generic heuristics: look for hdop or sat tokens
        std::string lower = r;
        for (char &c : lower) c = static_cast<char>(tolower(c));

        // hdop patterns
        size_t pos = std::string::npos;
        const char *hdop_tokens[] = { "hdop", "hdp", "pdop" };
        for (const char *t : hdop_tokens) {
            pos = lower.find(t);
            if (pos != std::string::npos) break;
        }
        if (pos != std::string::npos) {
            // find first number after token
            size_t i = pos;
            while (i < lower.size() && !( (lower[i] >= '0' && lower[i] <= '9') || lower[i] == '+' || lower[i] == '-' )) i++;
            if (i < lower.size()) {
                char *endp = nullptr;
                double v = strtod(lower.c_str() + i, &endp);
                if (endp != nullptr && (lower.c_str() + i) != endp) {
                    if (v > 0.0 && info.hdop <= 0.0) { info.hdop = v; found_any = true; }
                }
            }
        }

        // satellites patterns
        const char *sat_tokens[] = { "sat", "sats", "satellites" };
        pos = std::string::npos;
        for (const char *t : sat_tokens) {
            pos = lower.find(t);
            if (pos != std::string::npos) break;
        }
        if (pos != std::string::npos) {
            size_t i = pos;
            while (i < lower.size() && !(lower[i] >= '0' && lower[i] <= '9')) i++;
            if (i < lower.size()) {
                int v = static_cast<int>(strtol(lower.c_str() + i, nullptr, 10));
                if (v > 0 && info.satellites_used < v) { info.satellites_used = v; found_any = true; }
            }
        }
    }
    return found_any;
}

static bool is_valid_coordinate(double latitude, double longitude)
{
    if (latitude == 0.0 && longitude == 0.0) {
        return false;
    }
    if (latitude < -90.0 || latitude > 90.0) {
        return false;
    }
    if (longitude < -180.0 || longitude > 180.0) {
        return false;
    }
    return true;
}

static bool wait_for_gps_fix(GpsFixInfo &info, uint32_t timeout_ms)
{
    if (!s_state.gps_enabled && !enable_gps()) {
        ESP_LOGW(TAG, "GPS power enable failed before fix attempt");
        return false;
    }

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    std::string first_timestamp;
    int last_run_status = -1;
    int last_fix_status = -1;
    std::string last_timestamp;
    while (xTaskGetTickCount() < deadline) {
        std::string gps_response;
        if (!get_gps_location(&gps_response, 5000)) {
            ESP_LOGI(TAG, "GPS query failed during fix wait: %s", gps_status_line(gps_response).c_str());
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!parse_gps_fix_info_from_cgnsinf(gps_response, info)) {
            ESP_LOGW(TAG, "GPS response parse failed: %s", gps_status_line(gps_response).c_str());
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (info.run_status != last_run_status || info.fix_status != last_fix_status || info.timestamp != last_timestamp) {
            if (info.run_status != 1) {
                ESP_LOGI(TAG, "GPS engine not running; run=%d", info.run_status);
            } else if (info.fix_status <= 0) {
                ESP_LOGI(TAG, "GPS running but no fix yet (fix_status=%d, timestamp=%s)", info.fix_status, info.timestamp.c_str());
            }
            last_run_status = info.run_status;
            last_fix_status = info.fix_status;
            last_timestamp = info.timestamp;
        }

        if (info.run_status != 1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (info.fix_status <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!first_timestamp.empty() && !info.timestamp.empty() && info.timestamp == first_timestamp) {
            if (info.timestamp != last_timestamp) {
                ESP_LOGI(TAG, "GPS hot-start fix stale (%s); waiting for current fix", info.timestamp.c_str());
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (first_timestamp.empty() && !info.timestamp.empty()) {
            first_timestamp = info.timestamp;
            ESP_LOGI(TAG, "Hot-start fix (stale): %s — waiting for current", info.timestamp.c_str());
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Try to fetch richer GNSS metadata (GGA) to get HDOP and satellite counts when possible. */
        {
            std::string saved_seq;
            std::string resp;
            if (send_at_command("AT+CGNSSEQ?\r", &resp, 1000)) {
                saved_seq = trim_response(resp);
            }

            /* Request GGA which often includes HDOP and satellite counts */
            send_at_command("AT+CGNSSEQ=\"GGA\"\r", &resp, 1000);
            vTaskDelay(pdMS_TO_TICKS(500));

            std::string meta_resp;
            GpsFixInfo meta_info;
            if (get_gps_location(&meta_resp, 3000) && parse_gps_fix_info_from_cgnsinf(meta_resp, meta_info)) {
                if (meta_info.hdop > 0.0) info.hdop = meta_info.hdop;
                if (meta_info.satellites_used >= 0) info.satellites_used = meta_info.satellites_used;
                if (meta_info.satellites_in_view >= 0) info.satellites_in_view = meta_info.satellites_in_view;
            }

            /* If still missing or incomplete metadata, try other vendor-specific commands */
            if ((info.hdop <= 0.0 || info.satellites_used <= 0) && fetch_additional_gnss_metadata(info)) {
                ESP_LOGI(TAG, "GPS fix obtained (from supplemental metadata): %.6f, %.6f (sats=%d/%d, hdop=%.2f)",
                         info.latitude, info.longitude, info.satellites_used, info.satellites_in_view, info.hdop);
            } else if (info.hdop > 0.0 || info.satellites_used > 0) {
                ESP_LOGI(TAG, "GPS fix obtained (partial metadata): %.6f, %.6f (hdop=%.2f, sats=%d)",
                         info.latitude, info.longitude, info.hdop, info.satellites_used);
            } else {
                ESP_LOGI(TAG, "GPS fix obtained (no rich metadata): %.6f, %.6f (hdop=%.2f, sats=%d)",
                         info.latitude, info.longitude, info.hdop, info.satellites_used);
            }

            /* Restore previous sequence if available */
            if (!saved_seq.empty()) {
                size_t q1 = saved_seq.find('"');
                size_t q2 = (q1 == std::string::npos) ? std::string::npos : saved_seq.find('"', q1 + 1);
                if (q1 != std::string::npos && q2 != std::string::npos && q2 > q1) {
                    std::string seqval = saved_seq.substr(q1, q2 - q1 + 1); // includes quotes
                    std::string restore_cmd = std::string("AT+CGNSSEQ=") + seqval + "\r";
                    send_at_command(restore_cmd, &resp, 1000);
                }
            }
        }
        return true;
    }

    ESP_LOGW(TAG, "GPS fix timeout");
    return false;
}

static void log_sim_status(void)
{
    std::string response;
    if (send_at_command("AT+CPIN?\r", &response, 6000)) {
        ESP_LOGI(TAG, "SIM status: %s", trim_response(response).c_str());
    } else {
        ESP_LOGW(TAG, "SIM status query failed: %s", trim_response(response).c_str());
    }

    if (send_at_command("AT+CCID?\r", &response, 6000)) {
        ESP_LOGI(TAG, "SIM ICCID: %s", trim_response(response).c_str());
    }

    if (send_at_command("AT+CSQ\r", &response, 6000)) {
        ESP_LOGI(TAG, "Signal quality: %s", trim_response(response).c_str());
    }
}

static bool parse_registration_response(const std::string &response)
{
    const std::string trimmed = trim_response(response);
    return trimmed.find(",1") != std::string::npos || trimmed.find(",5") != std::string::npos;
}

static void report_registration_query(const char *label, const std::string &response)
{
    const std::string trimmed = trim_response(response);
    ESP_LOGI(TAG, "%s response: %s", label, trimmed.c_str());
}

static __attribute__((unused)) bool log_network_registration(void)
{
    std::string creg_response;
    const bool creg_ok = send_at_command("AT+CREG?\r", &creg_response, 5000);
    report_registration_query("AT+CREG?", creg_response);

    std::string cgreg_response;
    const bool cgreg_ok = send_at_command("AT+CGREG?\r", &cgreg_response, 5000);
    report_registration_query("AT+CGREG?", cgreg_response);

    const bool creg_registered = creg_ok && parse_registration_response(creg_response);
    const bool cgreg_registered = cgreg_ok && parse_registration_response(cgreg_response);
    const bool registered = creg_registered || cgreg_registered;
    ESP_LOGI(TAG, "GSM network registration status: %s (CREG=%s, CGREG=%s)",
             registered ? "READY" : "NOT READY",
             creg_registered ? "READY" : "NOT READY",
             cgreg_registered ? "READY" : "NOT READY");
    return registered;
}

static bool is_network_registered(void)
{
    std::string response;
    if (send_at_command("AT+CREG?\r", &response, 5000)) {
        const std::string result = trim_response(response);
        if (result.find(",1") != std::string::npos || result.find(",5") != std::string::npos) {
            return true;
        }
    }
    if (send_at_command("AT+CGREG?\r", &response, 5000)) {
        const std::string result = trim_response(response);
        if (result.find(",1") != std::string::npos || result.find(",5") != std::string::npos) {
            return true;
        }
    }
    return false;
}

static bool wait_for_network_registration(uint32_t timeout_ms)
{
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    bool saw_signal_issue = false;
    while (xTaskGetTickCount() < deadline) {
        std::string creg_response;
        const bool creg_ok = send_at_command("AT+CREG?\r", &creg_response, 5000);
        std::string cgreg_response;
        const bool cgreg_ok = send_at_command("AT+CGREG?\r", &cgreg_response, 5000);

        const bool creg_registered = creg_ok && parse_registration_response(creg_response);
        const bool cgreg_registered = cgreg_ok && parse_registration_response(cgreg_response);
        const bool registered = creg_registered || cgreg_registered;

        ESP_LOGI(TAG, "SIM registration check: CREG='%s' CGREG='%s' => %s",
                 flatten_response(creg_response).c_str(),
                 flatten_response(cgreg_response).c_str(),
                 registered ? "READY" : "NOT READY");

        if (registered) {
            ESP_LOGI(TAG, "Cellular network registered");
            return true;
        }

        if (!creg_ok || !cgreg_ok) {
            ESP_LOGW(TAG, "Network registration query failed; retrying");
        } else if (!saw_signal_issue) {
            ESP_LOGW(TAG, "Cellular modem is not registered yet; signal or carrier conditions may be poor");
            saw_signal_issue = true;
        }

        if ((xTaskGetTickCount() + pdMS_TO_TICKS(10000)) >= deadline) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    ESP_LOGW(TAG, "GSM registration timeout after %u ms; checking SIM/APN state", timeout_ms);
    ensure_apn_configured();

    std::string csq_response;
    if (send_at_command("AT+CSQ\r", &csq_response, 5000)) {
        ESP_LOGW(TAG, "Final signal quality after registration timeout: %s", trim_response(csq_response).c_str());
    } else {
        ESP_LOGW(TAG, "Unable to query signal quality after registration timeout");
    }
    return false;
}

static void log_call_activity_status(void)
{
    std::string response;

    if (send_at_command("AT+CPAS\r", &response, 4000, "+CPAS:")) {
        ESP_LOGI(TAG, "Phone activity status: %s", trim_response(response).c_str());
    } else {
        ESP_LOGI(TAG, "Phone activity status query returned no usable response: %s", trim_response(response).c_str());
    }
}

static void log_call_preflight(void)
{
    std::string response;
    if (send_at_command("AT+CSQ\r", &response, 5000, "+CSQ:")) {
        ESP_LOGI(TAG, "Signal quality before call: %s", trim_response(response).c_str());
    } else {
        ESP_LOGW(TAG, "Unable to query signal quality before call: %s", trim_response(response).c_str());
    }
    if (send_at_command("AT+CPAS\r", &response, 4000, "+CPAS:")) {
        ESP_LOGI(TAG, "Phone activity before call: %s", trim_response(response).c_str());
    } else {
        ESP_LOGW(TAG, "Unable to query phone activity before call: %s", trim_response(response).c_str());
    }
}

static bool clcc_has_active_call(const std::string &response)
{
    size_t pos = 0;
    while ((pos = response.find("+CLCC:", pos)) != std::string::npos) {
        const size_t line_end = response.find('\n', pos);
        std::string line = response.substr(pos, line_end == std::string::npos ? std::string::npos : line_end - pos);
        pos = line_end == std::string::npos ? std::string::npos : line_end + 1;
        const size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string rest = line.substr(colon + 1);
        std::vector<std::string> fields;
        size_t start = 0;
        while (start < rest.size()) {
            size_t comma = rest.find(',', start);
            if (comma == std::string::npos) {
                fields.push_back(trim_response(rest.substr(start)));
                break;
            }
            fields.push_back(trim_response(rest.substr(start, comma - start)));
            start = comma + 1;
        }
        if (fields.size() < 3) continue;
        int stat = atoi(fields[2].c_str());
        if (stat == 0 || stat == 1 || stat == 2 || stat == 3 || stat == 5) {
            return true;
        }
    }
    return false;
}

static void log_call_failure_details(void)
{
    std::string response;
    if (send_at_command("AT+CEER\r", &response, 5000)) {
        ESP_LOGI(TAG, "Call extended error report: %s", trim_response(response).c_str());
    } else {
        ESP_LOGI(TAG, "Call extended error report not available; call may have completed successfully or no extended error was reported");
    }
}

bool wait_for_prompt(const std::string &cmd, const std::string &prompt, std::string *response, uint32_t timeout_ms, char separator)
{
    if (!s_state.dte) {
        return false;
    }

    std::string buffer;
    auto result = s_state.dte->command(cmd, [&buffer,&prompt](uint8_t *data, size_t len) -> esp_modem::command_result {
        buffer.append(reinterpret_cast<char *>(data), len);
        const std::string line(reinterpret_cast<char *>(data), len);
        const std::string trimmed = trim_response(line);
        if (trimmed.find(prompt) != std::string::npos) {
            return esp_modem::command_result::OK;
        }
        if (trimmed.find("ERROR") != std::string::npos || trimmed.find("FAIL") != std::string::npos || trimmed.find("+CME ERROR") != std::string::npos || trimmed.find("+CMS ERROR") != std::string::npos) {
            return esp_modem::command_result::FAIL;
        }
        return esp_modem::command_result::TIMEOUT;
    }, timeout_ms, separator);

    if (response != nullptr) {
        *response = buffer;
    }
    return result == esp_modem::command_result::OK;
}

bool send_sms(const std::string &number, const std::string &message)
{
    if (!s_state.dte) {
        return false;
    }

    std::string response;
    if (!send_at_command("AT+CMGF=1\r", &response, 5000)) {
        ESP_LOGW(TAG, "Failed to set SMS text mode: %s", trim_response(response).c_str());
        return false;
    }

    std::string sms_cmd = "AT+CMGS=\"" + number + "\"\r";
    if (!wait_for_prompt(sms_cmd, ">", &response, 10000, '>')) {
        ESP_LOGW(TAG, "SMS command failed or timed out waiting for prompt: %s", trim_response(response).c_str());
        return false;
    }

    if (response.find('>') == std::string::npos) {
        ESP_LOGW(TAG, "Modem did not return SMS prompt: %s", trim_response(response).c_str());
        return false;
    }

    const std::string payload = message + std::string(1, '\x1A');
    std::string final_response;
    auto result = s_state.dte->command(payload, [&final_response](uint8_t *data, size_t len) -> esp_modem::command_result {
        final_response.append(reinterpret_cast<char *>(data), len);
        const std::string line(reinterpret_cast<char *>(data), len);
        const std::string trimmed = trim_response(line);
        if (trimmed.find("OK") != std::string::npos) {
            return esp_modem::command_result::OK;
        }
        if (trimmed.find("ERROR") != std::string::npos || trimmed.find("FAIL") != std::string::npos || trimmed.find("+CME ERROR") != std::string::npos || trimmed.find("+CMS ERROR") != std::string::npos) {
            return esp_modem::command_result::FAIL;
        }
        return esp_modem::command_result::TIMEOUT;
    }, 15000, '\n');

    if (result != esp_modem::command_result::OK) {
        ESP_LOGW(TAG, "SMS send failed during final write: %d, response: %s", static_cast<int>(result), trim_response(final_response).c_str());
        return false;
    }

    const std::string final_trim = trim_response(final_response);
    if (final_trim.find("OK") == std::string::npos) {
        ESP_LOGW(TAG, "SMS send response indicates failure: %s", final_trim.c_str());
        return false;
    }

    ESP_LOGI(TAG, "SMS send succeeded: %s", final_trim.c_str());
    return true;
}

bool make_call(const std::string &number)
{
    std::string response;
    std::string cmd = "ATD" + number + ";\r";
    ESP_LOGI(TAG, "Issuing emergency call command: %s", cmd.c_str());
    const bool ok = send_at_command(cmd, &response, 15000);
    const std::string trimmed = trim_response(response);
    ESP_LOGI(TAG, "Emergency call raw response: %s", trimmed.c_str());

    bool explicit_error = false;
    if (trimmed.find("ERROR") != std::string::npos || trimmed.find("FAIL") != std::string::npos || trimmed.find("+CME ERROR") != std::string::npos || trimmed.find("+CMS ERROR") != std::string::npos) {
        explicit_error = true;
    }

    if (!ok && explicit_error) {
        ESP_LOGW(TAG, "Emergency call command failed: %s", trimmed.c_str());
        return false;
    }

    if (!ok && !explicit_error) {
        ESP_LOGI(TAG, "Emergency call command completed without explicit OK; verifying call state");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    std::string clcc_response;
    bool had_active_call = false;
    if (send_at_command("AT+CLCC?\r", &clcc_response, 5000)) {
        ESP_LOGI(TAG, "Post-dial call list: %s", trim_response(clcc_response).c_str());
        if (clcc_has_active_call(clcc_response)) {
            ESP_LOGI(TAG, "Ongoing call confirmed by AT+CLCC?");
            had_active_call = true;
        }
    } else {
        ESP_LOGI(TAG, "Post-dial call list query failed: %s", trim_response(clcc_response).c_str());
    }

    if (had_active_call) {
        return true;
    }

    std::string pas_response;
    if (send_at_command("AT+CPAS\r", &pas_response, 5000, "+CPAS:")) {
        ESP_LOGI(TAG, "Phone activity after dial: %s", trim_response(pas_response).c_str());
        const std::string pas_trimmed = trim_response(pas_response);
        if (pas_trimmed.find("+CPAS: 3") != std::string::npos || pas_trimmed.find("+CPAS: 4") != std::string::npos) {
            ESP_LOGI(TAG, "Call activity detected after dial");
            return true;
        }
    }

    if (trimmed.find("NO CARRIER") != std::string::npos) {
        ESP_LOGI(TAG, "Call command returned NO CARRIER; confirming active call state with AT+CLCC?/AT+CPAS?");
    }

    return false;
}

bool enable_gps(void)
{
    std::string response;
    if (!send_at_command("AT+CGNSPWR=1\r", &response, 3000)) {
        ESP_LOGW(TAG, "GPS power on: FAILED -> %s", trim_response(response).c_str());
        return false;
    }
    ESP_LOGI(TAG, "GPS power on: SUCCESS");
    if (!send_at_command("AT+CGNSSEQ=\"RMC\"\r", &response, 3000)) {
        ESP_LOGW(TAG, "GPS RMC configuration: FAILED -> %s", trim_response(response).c_str());
        return false;
    }
    ESP_LOGI(TAG, "GPS RMC configuration: SUCCESS");
    return true;
}

bool get_gps_location(std::string *response, uint32_t timeout_ms)
{
    std::string gps_response;
    const bool ok = send_at_command("AT+CGNSINF\r", &gps_response, timeout_ms);
    if (response != nullptr) {
        *response = gps_response;
    }
    if (!ok) ESP_LOGW(TAG, "GPS query: FAILED (%s)", gps_status_line(gps_response).c_str());
    return ok;
}

__attribute__((unused)) bool run_at_step(const char *label, const std::string &command, uint32_t timeout_ms, const char *success_marker)
{
    std::string response;
    const bool ok = send_at_command(command, &response, timeout_ms, success_marker);
    ESP_LOGI(TAG, "%s: %s%s", label, ok ? "SUCCESS" : "FAILED",
             response.empty() ? "" : (std::string(" -> ") + trim_response(response)).c_str());
    return ok;
}

}  // namespace

extern "C" bool gl868_modem_send_at_command(const char *command, char *response, size_t response_len, uint32_t timeout_ms)
{
    if (!s_state.initialized || command == nullptr || response == nullptr || response_len == 0) {
        return false;
    }

    std::string cmd(command);
    if (cmd.empty()) {
        return false;
    }
    if (cmd.back() != '\r') {
        cmd.push_back('\r');
    }

    std::string resp;
    const bool ok = send_at_command(cmd, &resp, timeout_ms);
    const size_t copy_len = resp.size() < (response_len - 1) ? resp.size() : (response_len - 1);
    std::memcpy(response, resp.c_str(), copy_len);
    response[copy_len] = '\0';
    return ok;
}

extern "C" void gl868_modem_run_diagnostics(void)
{
    if (!s_state.initialized) {
        ESP_LOGW(TAG, "Modem not initialized; skipping diagnostics");
        return;
    }

    static const char *diagnostic_commands[] = {
        "AT", "ATI", "AT+CSQ", "AT+COPS?", "AT+CREG?", "AT+CGNSPWR=1", "AT+CGNSINF", "AT+CBC"
    };

    char response[256];
    for (size_t i = 0; i < sizeof(diagnostic_commands) / sizeof(diagnostic_commands[0]); ++i) {
        const bool ok = gl868_modem_send_at_command(diagnostic_commands[i], response, sizeof(response), 4000);
        ESP_LOGI(TAG, "AT[%zu] %s => %s", i, diagnostic_commands[i], ok ? response : "timeout/no-response");
    }
}

extern "C" void gl868_modem_run_full_diagnostics(void)
{
    if (!s_state.initialized) {
        ESP_LOGW(TAG, "Modem not initialized; skipping full diagnostics");
        return;
    }

    char response[512];
    const auto run_command = [&](const char *command, const char *label, uint32_t timeout_ms = 5000) {
        const bool ok = gl868_modem_send_at_command(command, response, sizeof(response), timeout_ms);
        const std::string value = ok ? trim_response(response) : std::string("timeout/no-response");
        ESP_LOGI(TAG, "%s: %s", label, value.c_str());
    };

    ESP_LOGI(TAG, "---------");
    ESP_LOGI(TAG, "Modem");
    ESP_LOGI(TAG, "---------");
    run_command("AT", "AT");
    run_command("ATI", "ATI");
    run_command("AT+CGSN", "IMEI");
    run_command("AT+CIMI", "IMSI");
    run_command("AT+CCID", "ICCID");
    run_command("AT+CNUM", "Phone Number");

    ESP_LOGI(TAG, "-----------");
    ESP_LOGI(TAG, "Network");
    ESP_LOGI(TAG, "-----------");
    run_command("AT+COPS?", "Operator");
    run_command("AT+CSQ", "RSSI");
    run_command("AT+CREG?", "Registration");

    ESP_LOGI(TAG, "-----------");
    ESP_LOGI(TAG, "Battery");
    ESP_LOGI(TAG, "-----------");
    run_command("AT+CBC", "Battery / Power Status");
}

extern "C" bool gl868_modem_send_test_sms(const char *message)
{
    if (!s_state.initialized) {
        ESP_LOGW(TAG, "SIM868 modem bridge is not initialized; cannot send diagnostic SMS");
        return false;
    }

    const char *number = get_emergency_sms_number();
    ESP_LOGI(TAG, "Emergency SMS target: %s", number);

    std::string sim_status;
    if (!is_sim_ready(&sim_status)) {
        ESP_LOGW(TAG, "SIM not ready for SMS test: %s", sim_status.c_str());
        return false;
    }

    log_sim_status();

    const std::string sms_message = message != nullptr ? message : "[ALERT]: SMART_SAETY_BAND_001";
    const bool sent = send_sms(number, sms_message);
    ESP_LOGI(TAG, "Diagnostic SMS to %s -> %s", number, sent ? "sent" : "failed");
    return sent;
}

extern "C" bool gl868_modem_init(void)
{
    if (s_state.initialized) {
        return true;
    }

    esp_modem_dte_config_t config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    config.uart_config.port_num = UART_NUM_1;
    config.uart_config.tx_io_num = 17;
    config.uart_config.rx_io_num = 18;
    config.uart_config.baud_rate = 115200;
    config.uart_config.rx_buffer_size = 4096;
    config.uart_config.tx_buffer_size = 512;

    power_cycle_modem();

    s_state.dte = esp_modem::create_uart_dte(&config);
    if (!s_state.dte) {
        ESP_LOGE(TAG, "Failed to create UART DTE for SIM868");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    bool at_ok = false;

    /* First, attempt an extended initial AT handshake to give the modem
     * extra time to boot/respond on the very first attempt. If this fails,
     * fall back to the existing retry + power-cycle sequence. */
    {
        std::string response;
        if (send_at_command("AT\r", &response, 10000)) {
            ESP_LOGI(TAG, "Modem AT handshake: SUCCESS");
            at_ok = true;
        }
    }

    if (!at_ok) {
        for (int attempt = 1; attempt <= 3; ++attempt) {
            std::string response;
            if (send_at_command("AT\r", &response, 3000)) {
                ESP_LOGI(TAG, "Modem AT handshake: SUCCESS");
                at_ok = true;
                break;
            }

            if (attempt == 3) {
                ESP_LOGW(TAG, "Modem AT handshake: FAILED after 3 attempts -> %s", trim_response(response).c_str());
            }
            if (attempt < 3) {
                ESP_LOGI(TAG, "Waiting for modem boot and retrying initialization");
                vTaskDelay(pdMS_TO_TICKS(3000));
                ESP_LOGI(TAG, "Power-cycling SIM868 and retrying initialization");
                s_state.dte.reset();
                power_cycle_modem();
                s_state.dte = esp_modem::create_uart_dte(&config);
                if (!s_state.dte) {
                    ESP_LOGE(TAG, "Failed to recreate UART DTE after power cycle");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(2000));
            } else {
                s_state.dte.reset();
            }
        }
    }

    if (!at_ok) {
        s_state.dte.reset();
        gpio_set_level(GPIO_NUM_42, 0);
        return false;
    }

    if (!wait_for_sim_ready(30000)) {
        ESP_LOGW(TAG, "SIM card did not become ready during init; check SIM presence and PIN state");
        s_state.dte.reset();
        gpio_set_level(GPIO_NUM_42, 0);
        return false;
    }

    ensure_apn_configured();

    /* Wait for basic GSM network registration to complete during init so
     * that subsequent operations can assume network availability. */
    if (!wait_for_network_registration(45000)) {
        ESP_LOGW(TAG, "GSM network registration did not complete during init; poor signal, carrier, or SIM may be preventing registration");
    }

    std::string response;
    if (!send_at_command("AT+CMEE=2\r", &response, 3000)) {
        ESP_LOGW(TAG, "Failed to enable verbose modem errors: %s", trim_response(response).c_str());
    }
    if (!send_at_command("AT+CIURC=0\r", &response, 3000)) {
        ESP_LOGW(TAG, "Initial readiness URCs disable: FAILED -> %s", trim_response(response).c_str());
    } else {
        ESP_LOGI(TAG, "Initial readiness URCs disabled (Call Ready/SMS Ready)");
    }

    s_state.gps_enabled = enable_gps();
    s_state.initialized = true;
    ESP_LOGI(TAG, "SIM868 modem bridge initialized (GPS=%d)", s_state.gps_enabled);

    ESP_LOGI(TAG, "Running full SIM868 diagnostics after modem initialization");
    gl868_modem_run_full_diagnostics();
    return true;
}

extern "C" void gl868_modem_update(void)
{
    if (!s_state.initialized) {
        return;
    }
}

extern "C" void gl868_modem_trigger_emergency(const char *source, int32_t value)
{
    if (!s_state.initialized) {
        return;
    }

    const char *call_number = get_emergency_call_number();
    const char *sms_number = get_emergency_sms_number();
    ESP_LOGI(TAG, "Emergency call target: %s", call_number);
    ESP_LOGI(TAG, "Emergency SMS target (forced): %s", sms_number);

    std::string sim_status;
    if (!is_sim_ready(&sim_status)) {
        ESP_LOGW(TAG, "SIM not ready for emergency alert: %s", sim_status.c_str());
        return;
    }
    log_sim_status();
    if (!wait_for_network_registration(30000)) {
        ESP_LOGW(TAG, "GSM network registration failed after 30 seconds; emergency alert may not be delivered");
    }

    std::string gps_response;
    GpsFixInfo fix_info;
    bool gps_ok = false;
    /* Single immediate GPS attempt: if it fails, schedule deferred retries
     * and continue with emergency SMS/call without blocking here. */
    if (get_gps_location(&gps_response, 10000) && parse_gps_fix_info_from_cgnsinf(gps_response, fix_info)) {
        gps_ok = true;
        ESP_LOGI(TAG, "GPS fix accepted: status=%d mode=%d hdop=%.2f sats=%d/%d",
                 fix_info.fix_status, fix_info.fix_mode, fix_info.hdop,
                 fix_info.satellites_used, fix_info.satellites_in_view);
        ESP_LOGI(TAG, "AT+CGNSINF response: %s", gps_status_line(gps_response).c_str());
        if (fix_info.hdop > 5.0) {
            ESP_LOGW(TAG, "GPS accuracy is low (hdop=%.2f); map location may be imprecise", fix_info.hdop);
        }
    } else {
        ESP_LOGW(TAG, "Immediate GPS attempt failed: %s; requesting deferred retry via main task", gps_status_line(gps_response).c_str());
        gl868_modem_request_deferred_gps_upload();
    }

    // build message: include battery percent if available and Google Maps link when possible
    int batt = -1;
    {
        char cbc[128] = {0};
        if (gl868_modem_send_at_command("AT+CBC", cbc, sizeof(cbc), 3000)) {
            const std::string cbcs = trim_response(std::string(cbc));
            int bcs = 0, bcl = 0, volt = 0;
            if (sscanf(cbcs.c_str(), "+CBC: %d,%d,%d", &bcs, &bcl, &volt) >= 2) batt = bcl;
        }
    }
    char message[320];
    if (gps_ok) {
        snprintf(message, sizeof(message),
                 "ALERT: SOS activated! Loc: %.6f,%.6f. Map: [https://maps.google.com/?q=%.6f,%.6f] (https://maps.google.com/?q=%.6f,%.6f) Batt: %d%%",
                 fix_info.latitude, fix_info.longitude,
                 fix_info.latitude, fix_info.longitude,
                 fix_info.latitude, fix_info.longitude,
                 batt >= 0 ? batt : 0);
    } else {
        snprintf(message, sizeof(message),
                 "ALERT: SOS activated! Loc: unavailable. Map: unavailable Batt: %d%%",
                 batt >= 0 ? batt : 0);
    }

    ESP_LOGI(TAG, "Sending emergency SMS to %s", sms_number);
    // Support multiple recipients separated by comma or semicolon
    const std::vector<std::string> recipients = split_recipients(std::string(sms_number));
    bool any_sent = false;
    for (const auto &r : recipients) {
        const bool sms_ok = send_sms(r, std::string(message));
        ESP_LOGI(TAG, "Emergency SMS to %s -> %s", r.c_str(), sms_ok ? "sent" : "failed");
        if (sms_ok) any_sent = true;
    }
    if (any_sent) {
        ESP_LOGI(TAG, "Emergency SMS succeeded; waiting before initiating emergency call");
    } else {
        ESP_LOGW(TAG, "Emergency SMS failed for all recipients; continuing with independent call attempt");
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Initiating emergency call to %s", call_number);
    log_call_preflight();
    const bool call_ok = make_call(call_number);
    if (!call_ok) {
        ESP_LOGW(TAG, "Emergency call not confirmed for %s; it may still have been placed", call_number);
        log_call_failure_details();
    } else {
        ESP_LOGI(TAG, "Emergency call initiated to %s", call_number);
        vTaskDelay(pdMS_TO_TICKS(2000));
        log_call_activity_status();
    }
}

extern "C" bool gl868_modem_send_sms_to(const char *number, const char *message)
{
    if (!s_state.initialized || number == nullptr || message == nullptr) return false;
    return send_sms(std::string(number), std::string(message));
}

extern "C" bool gl868_modem_make_call_to(const char *number)
{
    if (!s_state.initialized || number == nullptr) return false;
    return make_call(std::string(number));
}

extern "C" bool gl868_modem_get_gps_now(char *buf, size_t buf_len)
{
    if (!s_state.initialized || buf == nullptr || buf_len == 0) return false;
    std::string gps;
    const bool ok = get_gps_location(&gps);
    const size_t copy_len = gps.size() < (buf_len - 1) ? gps.size() : (buf_len - 1);
    if (copy_len > 0) memcpy(buf, gps.c_str(), copy_len);
    buf[copy_len] = '\0';
    return ok;
}

extern "C" bool gl868_modem_get_gps_coordinates(double *latitude, double *longitude)
{
    if (!s_state.initialized || latitude == nullptr || longitude == nullptr) return false;

    std::string gps_response;
    GpsFixInfo fix;
    if (!get_gps_location(&gps_response, 10000)) {
        ESP_LOGW(TAG, "GPS location query failed: %s", gps_status_line(gps_response).c_str());
        return false;
    }
    if (!parse_gps_fix_info_from_cgnsinf(gps_response, fix)) {
        ESP_LOGW(TAG, "GPS response parse failed: %s", gps_status_line(gps_response).c_str());
        return false;
    }
    if (fix.fix_status <= 0) {
        ESP_LOGW(TAG, "GPS fix unavailable: run=%d fix_status=%d timestamp=%s", fix.run_status, fix.fix_status, fix.timestamp.c_str());
        return false;
    }

    bool poor_metadata = (fix.hdop <= 0.0 || fix.hdop > GPS_ACCEPTABLE_HDOP || fix.satellites_used < GPS_MIN_SATELLITES);

    if (poor_metadata && GPS_ENABLE_SMOOTHING_FALLBACK) {
        static double last_lat = 0.0;
        static double last_lon = 0.0;
        static bool have_last = false;
        double out_lat = fix.latitude;
        double out_lon = fix.longitude;

        if (!have_last) {
            last_lat = fix.latitude;
            last_lon = fix.longitude;
            have_last = true;
            ESP_LOGW(TAG, "GPS metadata poor (hdop=%.2f sats=%d); using first value as baseline", fix.hdop, fix.satellites_used);
        } else {
            out_lat = GPS_SMOOTHING_ALPHA * fix.latitude + (1.0 - GPS_SMOOTHING_ALPHA) * last_lat;
            out_lon = GPS_SMOOTHING_ALPHA * fix.longitude + (1.0 - GPS_SMOOTHING_ALPHA) * last_lon;
            last_lat = out_lat;
            last_lon = out_lon;
            ESP_LOGW(TAG, "GPS metadata poor (hdop=%.2f sats=%d); returning smoothed coord", fix.hdop, fix.satellites_used);
        }

        *latitude = out_lat;
        *longitude = out_lon;
        ESP_LOGI(TAG, "GPS fix: SUCCESS -> %.6f,%.6f (smoothed, hdop=%.2f, sats=%d)", *latitude, *longitude, fix.hdop, fix.satellites_used);
        return true;
    }

    *latitude = fix.latitude;
    *longitude = fix.longitude;
    ESP_LOGI(TAG, "GPS fix: SUCCESS -> %.6f,%.6f (hdop=%.2f, sats=%d)",
             fix.latitude, fix.longitude, fix.hdop, fix.satellites_used);
    return true;
}

extern "C" const char *gl868_modem_get_emergency_call_number(void)
{
    return get_emergency_call_number();
}

extern "C" const char *gl868_modem_get_emergency_sms_number(void)
{
    return get_emergency_sms_number();
}

extern "C" int gl868_modem_get_battery_percent(void)
{
    if (!s_state.initialized) return -1;
    char buf[128] = {0};
    if (!gl868_modem_send_at_command("AT+CBC", buf, sizeof(buf), 3000)) return -1;
    const std::string resp(buf);
    const std::string trimmed = trim_response(resp);
    // Expected: +CBC: <bcs>,<bcl>,<voltage>
    int bcs = 0, bcl = 0, volt = 0;
    if (sscanf(trimmed.c_str(), "+CBC: %d,%d,%d", &bcs, &bcl, &volt) >= 2) {
        return bcl; // battery percent
    }
    return -1;
}