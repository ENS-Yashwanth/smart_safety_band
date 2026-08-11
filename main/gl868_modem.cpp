#include <memory>
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_modem_config.h"
#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_dce.hpp"
#include "cxx_include/esp_modem_dte.hpp"
#include "cxx_include/esp_modem_types.hpp"

#if __has_include("esp_netif.h")
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_netif_ppp.h"
#define HAVE_ESP_NETIF 1
#else
#define HAVE_ESP_NETIF 0
#endif

static const char *TAG = "sim868_bridge";
static const char *DEFAULT_MODEM_APN = "internet";
static const gpio_num_t DEFAULT_MODEM_POWER_GPIO = GPIO_NUM_42;
static const uint32_t MODEM_STATUS_POLL_MS = 60000;

namespace {
static const char *DEFAULT_EMERGENCY_CALL_NUMBER = "+916309538622";
static const char *DEFAULT_EMERGENCY_SMS_NUMBER = "+917288973229";

    enum ModemState {
        MODEM_STATE_POWERED_ON,
        MODEM_STATE_SIM_CHECK,
        MODEM_STATE_APN_CONFIG,
        MODEM_STATE_ATTACH,
        MODEM_STATE_REGISTRATION,
        MODEM_STATE_LIVE,
        MODEM_STATE_RECONNECT,
        MODEM_STATE_ERROR,
    };

    struct Sim868State {
        std::shared_ptr<esp_modem::DTE> dte;
        bool initialized = false;
        bool gps_enabled = false;
        bool network_ready = false;
        bool live_signal_logged = false;
        ModemState state = MODEM_STATE_POWERED_ON;
        TickType_t next_action_tick = 0;
        uint32_t reconnect_delay_ms = 1000;
    };

    static Sim868State s_state;
    static SemaphoreHandle_t s_modem_mutex;
    static int s_last_csq_rssi = -1;
    static int s_last_csq_ber = -1;
    static int s_csq_poll_count = 0;

    class ModemLock {
    public:
        ModemLock() : locked(s_modem_mutex != nullptr &&
                             xSemaphoreTakeRecursive(s_modem_mutex, portMAX_DELAY) == pdTRUE) {}
        ~ModemLock() { if (locked) xSemaphoreGiveRecursive(s_modem_mutex); }
        bool locked;
    };

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

static bool send_at_command(const std::string &cmd, std::string *response, uint32_t timeout_ms, const char *success_marker = nullptr);
static void power_cycle_modem();

static void power_cycle_modem()
{
    gpio_set_level(DEFAULT_MODEM_POWER_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
    gpio_set_level(DEFAULT_MODEM_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(2500));
}

static void shutdown_modem(void);

static void shutdown_modem(void)
{
    if (!s_state.initialized) {
        gpio_set_level(DEFAULT_MODEM_POWER_GPIO, 0);
        return;
    }

    if (s_state.dte) {
        std::string response;
        if (send_at_command("AT+CPOWD=1\r", &response, 5000, "NORMAL POWER DOWN")) {
            ESP_LOGI(TAG, "Modem powered down gracefully");
        } else {
            ESP_LOGW(TAG, "Graceful modem shutdown failed; forcing power-cycle; response=%s", trim_response(response).c_str());
            s_state.dte.reset();
            power_cycle_modem();
        }
    } else {
        power_cycle_modem();
    }

    s_state.dte.reset();
    s_state.initialized = false;
    s_state.gps_enabled = false;
    s_state.network_ready = false;
    s_state.state = MODEM_STATE_POWERED_ON;
}

static bool wait_for_prompt(const std::string &cmd, const std::string &prompt, std::string *response, uint32_t timeout_ms, char separator = '\n');
static bool configure_apn(const char *apn);
static const char *get_configured_apn(char *buffer, size_t buffer_len);
static bool trigger_network_attach(void);
static bool poll_network_registration(std::string *status_out = nullptr);
static bool parse_registration_status(const std::string &response, bool *registered);
static bool monitor_signal(void);
static bool init_network_stack(void);
static void log_sim_status(void);
static const char *get_emergency_call_number(void);
static const char *get_emergency_sms_number(void);
static bool parse_latlon_from_cgnsinf(const std::string &cgnsinf, double &lat, double &lon);
static std::vector<std::string> split_recipients(const std::string &list);

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

static bool configure_apn(const char *apn)
{
    if (apn == nullptr || apn[0] == '\0') {
        ESP_LOGW(TAG, "APN not specified");
        return false;
    }
    std::string cmd = "AT+CGDCONT=1,\"IP\",\"";
    cmd += apn;
    cmd += "\"\r";
    std::string response;
    if (!send_at_command(cmd, &response, 10000)) {
        ESP_LOGW(TAG, "APN configuration failed: %s", trim_response(response).c_str());
        return false;
    }
    ESP_LOGI(TAG, "APN configured: %s", apn);
    return true;
}

static const char *get_configured_apn(char *buffer, size_t buffer_len)
{
    if (buffer == nullptr || buffer_len == 0) return DEFAULT_MODEM_APN;
    size_t length = buffer_len;
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READONLY, &handle) == ESP_OK) {
        esp_err_t err = nvs_get_str(handle, "cellular_apn", buffer, &length);
        nvs_close(handle);
        if (err == ESP_OK && buffer[0] != '\0') return buffer;
    }
    return DEFAULT_MODEM_APN;
}

static bool trigger_network_attach(void)
{
    std::string response;
    if (!send_at_command("AT+COPS=0\r", &response, 20000)) {
        ESP_LOGW(TAG, "Network attach command failed: %s", trim_response(response).c_str());
        return false;
    }
    ESP_LOGI(TAG, "Network attach requested");
    return true;
}

static bool parse_registration_status(const std::string &response, bool *registered)
{
    bool any = false;
    bool found_registered = false;
    const char *patterns[] = {"CEREG:", "CGREG:"};
    for (const char *pattern : patterns) {
        size_t pos = response.find(pattern);
        if (pos == std::string::npos) {
            continue;
        }
        size_t comma = response.find(',', pos);
        if (comma == std::string::npos || comma + 1 >= response.size()) {
            continue;
        }
        char status = response[comma + 1];
        if (status == '1' || status == '5') {
            found_registered = true;
        }
        any = true;
    }
    if (registered) {
        *registered = found_registered;
    }
    return any;
}

static bool poll_network_registration(std::string *status_out)
{
    std::string response;
    bool registered = false;
    if (send_at_command("AT+CEREG?\r", &response, 5000)) {
        if (parse_registration_status(response, &registered)) {
            if (status_out) *status_out = trim_response(response);
            return registered;
        }
    }
    if (send_at_command("AT+CGREG?\r", &response, 5000)) {
        if (parse_registration_status(response, &registered)) {
            if (status_out) *status_out = trim_response(response);
            return registered;
        }
    }
    if (status_out) *status_out = trim_response(response);
    return false;
}

static bool monitor_signal(void)
{
    std::string response;
    bool any_ok = false;
    if (send_at_command("AT+CESQ\r", &response, 5000)) {
        ESP_LOGD(TAG, "Signal metrics CESQ: %s", trim_response(response).c_str());
        any_ok = true;
    }
    if (send_at_command("AT+CSQ\r", &response, 5000)) {
        const std::string t = trim_response(response);
        int rssi = -1, ber = -1;
        if (sscanf(t.c_str(), "+CSQ: %d,%d", &rssi, &ber) >= 1) {
            ++s_csq_poll_count;
            bool changed = (rssi != s_last_csq_rssi) || (ber != s_last_csq_ber);
            if (s_csq_poll_count == 1 || changed) {
                ESP_LOGI(TAG, "Signal quality CSQ: %s", t.c_str());
                s_last_csq_rssi = rssi;
                s_last_csq_ber = ber;
            } else {
                ESP_LOGD(TAG, "Signal quality unchanged: %s", t.c_str());
            }
        } else {
            ESP_LOGW(TAG, "Unexpected CSQ response: %s", t.c_str());
        }
        any_ok = true;
    }
    return any_ok;
}

static bool init_network_stack(void)
{
#if HAVE_ESP_NETIF
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return false;
    }
#ifdef ESP_NETIF_DEFAULT_PPP
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_PPP();
    esp_netif_t *ppp_netif = esp_netif_new(&cfg);
    if (ppp_netif == NULL) {
        ESP_LOGW(TAG, "Failed to create PPP network interface");
        return false;
    }
    ESP_LOGI(TAG, "PPP network stack initialized");
    return true;
#else
    ESP_LOGW(TAG, "PPP is not enabled in sdkconfig; GSM voice/SMS remain available but cloud data transport is unavailable");
    return false;
#endif
#else
    ESP_LOGW(TAG, "ESP netif support unavailable; network stack init skipped");
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
        if (trimmed.find("OK") != std::string::npos) {
            return esp_modem::command_result::OK;
        }
        if (trimmed.find("ERROR") != std::string::npos || trimmed.find("FAIL") != std::string::npos || trimmed.find("+CME ERROR") != std::string::npos || trimmed.find("+CMS ERROR") != std::string::npos) {
            return esp_modem::command_result::FAIL;
        }
        if (success_marker != nullptr && trimmed.find(success_marker) != std::string::npos) {
            marker_seen = true;
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

static const char *load_nvs_string(const char *key, char *out, size_t out_len)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("safety", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return nullptr;
    }

    size_t length = out_len;
    err = nvs_get_str(handle, key, out, &length);
    nvs_close(handle);
    if (err == ESP_OK) {
        return out;
    }
    return nullptr;
}

static const char *get_emergency_call_number(void)
{
    static char nvs_call_number[128] = {0};
    char stored[256] = {0};

    if (load_nvs_string("emergency_numbers", stored, sizeof(stored))) {
        const std::vector<std::string> recipients = split_recipients(std::string(stored));
        if (!recipients.empty() && !recipients[0].empty()) {
            strncpy(nvs_call_number, recipients[0].c_str(), sizeof(nvs_call_number) - 1);
            nvs_call_number[sizeof(nvs_call_number) - 1] = '\0';
            return nvs_call_number;
        }
    }

    if (load_nvs_string("emergency_number", stored, sizeof(stored))) {
        if (stored[0] != '\0') {
            strncpy(nvs_call_number, stored, sizeof(nvs_call_number) - 1);
            nvs_call_number[sizeof(nvs_call_number) - 1] = '\0';
            return nvs_call_number;
        }
    }

#ifdef CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER
    return CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER[0] != '\0' ? CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER : DEFAULT_EMERGENCY_CALL_NUMBER;
#else
    return DEFAULT_EMERGENCY_CALL_NUMBER;
#endif
}

static const char *get_emergency_sms_number(void)
{
    static char nvs_sms_numbers[256] = {0};
    if (load_nvs_string("emergency_sms_numbers", nvs_sms_numbers, sizeof(nvs_sms_numbers))) {
        return nvs_sms_numbers;
    }
    if (load_nvs_string("emergency_sms_number", nvs_sms_numbers, sizeof(nvs_sms_numbers))) {
        return nvs_sms_numbers;
    }

#ifdef CONFIG_SAFETY_BAND_EMERGENCY_SMS_NUMBER
    return CONFIG_SAFETY_BAND_EMERGENCY_SMS_NUMBER[0] != '\0' ? CONFIG_SAFETY_BAND_EMERGENCY_SMS_NUMBER : DEFAULT_EMERGENCY_SMS_NUMBER;
#else
    return DEFAULT_EMERGENCY_SMS_NUMBER;
#endif
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

static bool parse_latlon_from_cgnsinf(const std::string &cgnsinf, double &lat, double &lon)
{
    // Look for a line starting with +CGNSINF: and parse CSV fields.
    const size_t pos = cgnsinf.find("+CGNSINF:");
    if (pos == std::string::npos) return false;
    const size_t line_end = cgnsinf.find('\n', pos);
    const std::string line = (line_end == std::string::npos) ? cgnsinf.substr(pos) : cgnsinf.substr(pos, line_end - pos);
    // strip prefix
    size_t colon = line.find(':');
    if (colon == std::string::npos) return false;
    std::string rest = line.substr(colon + 1);
    // trim
    const std::string trimmed = trim_response(rest);
    // tokenize by comma
    std::vector<std::string> fields;
    size_t s = 0;
    while (s < trimmed.size()) {
        size_t p = trimmed.find(',', s);
        if (p == std::string::npos) { fields.push_back(trimmed.substr(s)); break; }
        fields.push_back(trimmed.substr(s, p - s)); s = p + 1;
    }
    // CGNSINF format: <GNSS run status>,<Fix status>,<UTC>,<lat>,<lon>,... so lat is field[3], lon field[4]
    if (fields.size() < 5) return false;
    // convert using strtod to avoid exceptions (exceptions disabled in build)
    char *endptr = nullptr;
    lat = strtod(fields[3].c_str(), &endptr);
    if (endptr == fields[3].c_str()) return false;
    lon = strtod(fields[4].c_str(), &endptr);
    if (endptr == fields[4].c_str()) return false;
    // validate ranges
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;
    return true;
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
    if (final_trim.find("OK") == std::string::npos || final_trim.find("+CMGS:") == std::string::npos) {
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
    const bool ok = send_at_command(cmd, &response, 6000);
    const std::string trimmed = trim_response(response);
    if (ok) {
        ESP_LOGI(TAG, "Emergency call command succeeded: %s", trimmed.c_str());
        return true;
    }

    if (trimmed.empty()) {
        return true;
    }

    ESP_LOGW(TAG, "Emergency call command failed: %s", trimmed.c_str());
    return false;
}

bool enable_gps(void)
{
    std::string response;
    if (!send_at_command("AT+CGNSPWR=1\r", &response, 3000)) {
        return false;
    }
    /* Prefer GGA/RMC output where supported; older SIM868 firmware supports
     * only RMC. +CGNSINF remains the portable structured parsing interface. */
    if (!send_at_command("AT+CGNSSEQ=\"GGA,RMC\"\r", &response, 3000) &&
        !send_at_command("AT+CGNSSEQ=\"RMC\"\r", &response, 3000)) {
        ESP_LOGW(TAG, "GNSS sentence configuration failed: %s", trim_response(response).c_str());
    }
    return true;
}

bool disable_gps(void)
{
    std::string response;
    if (!send_at_command("AT+CGNSPWR=0\r", &response, 3000)) {
        return false;
    }
    return true;
}

bool get_gps_location(std::string *response)
{
    std::string gps_response;
    const bool ok = send_at_command("AT+CGNSINF\r", &gps_response, 5000);
    if (response != nullptr) {
        *response = gps_response;
    }
    return ok;
}
}  // namespace

extern "C" bool gl868_modem_send_at_command(const char *command, char *response, size_t response_len, uint32_t timeout_ms)
{
    ModemLock lock;
    if (!lock.locked) return false;
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
    ModemLock lock;
    if (!lock.locked) return;
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
    ModemLock lock;
    if (!lock.locked) return;
    if (!s_state.initialized) {
        ESP_LOGW(TAG, "Modem not initialized; skipping full diagnostics");
        return;
    }

    char response[512];
    const auto log_command = [&](const char *label, const char *command, uint32_t timeout_ms = 4000) {
        const bool ok = gl868_modem_send_at_command(command, response, sizeof(response), timeout_ms);
        const std::string value = ok ? trim_response(response) : std::string("timeout/no-response");
        ESP_LOGI(TAG, "[%s] %s", label, value.c_str());
    };

    ESP_LOGI(TAG, "Running full SIM868 diagnostics...");
    log_command("AT", "AT");
    log_command("ATI", "ATI");

    std::string sim_status;
    const bool sim_ready = is_sim_ready(&sim_status);
    ESP_LOGI(TAG, "[SIM status] %s", sim_status.c_str());

    log_command("Operator", "AT+COPS?");
    log_command("Signal", "AT+CSQ");
    log_command("Network reg", "AT+CREG?");
    log_command("Cell reg", "AT+CGREG?");
    log_command("Phone activity", "AT+CPAS");
    log_command("Current calls", "AT+CLCC?");

    if (sim_ready) {
        log_command("IMEI", "AT+CGSN");
        log_command("IMSI", "AT+CIMI");
        log_command("ICCID", "AT+CCID");
        log_command("Phone", "AT+CNUM");
        log_command("Service Center", "AT+CSCA?");
        log_command("SMS mode", "AT+CMGF=1");
        log_command("GPRS attach", "AT+CGATT?");
        log_command("CLIR", "AT+CLIR=2");
        log_command("COLP", "AT+COLP=1");
        log_command("CNUM?", "AT+CNUM?");
        log_command("CPBS", "AT+CPBS?");
    } else {
        ESP_LOGI(TAG, "SIM not ready; skipping SIM-dependent diagnostics");
    }

    log_command("GPS power", "AT+CGNSPWR=1");
    log_command("GPS info", "AT+CGNSINF", 5000);
    log_command("CBC", "AT+CBC");
    log_command("CENG", "AT+CENG=1,1");
    log_command("GSM location", "AT+CIPGSMLOC=1,1", 10000);
    ESP_LOGI(TAG, "Full diagnostics complete");
}

extern "C" bool gl868_modem_send_test_sms(const char *message)
{
    ModemLock lock;
    if (!lock.locked) return false;
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
    if (s_modem_mutex == nullptr) s_modem_mutex = xSemaphoreCreateRecursiveMutex();
    ModemLock lock;
    if (!lock.locked) return false;
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
    for (int attempt = 1; attempt <= 3; ++attempt) {
        std::string response;
        if (send_at_command("AT\r", &response, 3000)) {
            at_ok = true;
            break;
        }

        // ESP_LOGW(TAG, "SIM868 did not respond to AT (attempt %d): %s", attempt, trim_response(response).c_str());
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

    if (!at_ok) {
        s_state.dte.reset();
        gpio_set_level(DEFAULT_MODEM_POWER_GPIO, 0);
        return false;
    }

    std::string response;
    if (!send_at_command("AT+CMEE=2\r", &response, 3000)) {
        ESP_LOGW(TAG, "Failed to enable verbose modem errors: %s", trim_response(response).c_str());
    }

    s_state.gps_enabled = enable_gps();
    s_state.state = MODEM_STATE_SIM_CHECK;
    s_state.next_action_tick = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    s_state.reconnect_delay_ms = 1000;
    s_state.network_ready = false;
    s_state.initialized = true;
    ESP_LOGI(TAG, "SIM868 modem bridge initialized (GPS=%d)", s_state.gps_enabled);
    /* Log resolved emergency recipients so they appear whenever modem (re)initializes */
    ESP_LOGI(TAG, "Resolved emergency SMS recipients: %s", get_emergency_sms_number());
    ESP_LOGI(TAG, "Resolved emergency CALL number: %s", get_emergency_call_number());
    return true;
}

extern "C" void gl868_modem_shutdown(void)
{
    ModemLock lock;
    if (!lock.locked) return;
    shutdown_modem();
}

extern "C" void gl868_modem_update(void)
{
    ModemLock lock;
    if (!lock.locked) return;
    if (!s_state.initialized) {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    if (now < s_state.next_action_tick) {
        return;
    }

    switch (s_state.state) {
    case MODEM_STATE_SIM_CHECK: {
        std::string sim_status;
        if (is_sim_ready(&sim_status)) {
            ESP_LOGI(TAG, "SIM ready: %s", sim_status.c_str());
            s_state.state = MODEM_STATE_APN_CONFIG;
        } else {
            ESP_LOGW(TAG, "SIM not ready: %s", sim_status.c_str());
            s_state.next_action_tick = now + pdMS_TO_TICKS(5000);
        }
        break;
    }
    case MODEM_STATE_APN_CONFIG: {
        char apn[64] = {0};
        if (configure_apn(get_configured_apn(apn, sizeof(apn)))) {
            s_state.state = MODEM_STATE_ATTACH;
            s_state.next_action_tick = now + pdMS_TO_TICKS(2000);
        } else {
            s_state.next_action_tick = now + pdMS_TO_TICKS(10000);
        }
        break;
    }
    case MODEM_STATE_ATTACH: {
        if (trigger_network_attach()) {
            s_state.state = MODEM_STATE_REGISTRATION;
            s_state.next_action_tick = now + pdMS_TO_TICKS(5000);
        } else {
            s_state.state = MODEM_STATE_RECONNECT;
        }
        break;
    }
    case MODEM_STATE_REGISTRATION: {
        std::string registration_status;
        if (poll_network_registration(&registration_status)) {
            ESP_LOGI(TAG, "Network registration achieved: %s", registration_status.c_str());
            s_state.network_ready = init_network_stack();
            s_state.live_signal_logged = false;
            s_state.state = MODEM_STATE_LIVE;
            s_state.next_action_tick = now + pdMS_TO_TICKS(MODEM_STATUS_POLL_MS);
        } else {
            ESP_LOGW(TAG, "Network not registered yet: %s", registration_status.c_str());
            s_state.state = MODEM_STATE_RECONNECT;
        }
        break;
    }
    case MODEM_STATE_LIVE: {
        if (!s_state.live_signal_logged) {
            monitor_signal();
            s_state.live_signal_logged = true;
        }

        std::string registration_status;
        if (!poll_network_registration(&registration_status)) {
            ESP_LOGW(TAG, "Network attach dropped, reconnecting: %s", registration_status.c_str());
            s_state.state = MODEM_STATE_RECONNECT;
            s_state.next_action_tick = now + pdMS_TO_TICKS(s_state.reconnect_delay_ms);
        } else {
            s_state.next_action_tick = now + pdMS_TO_TICKS(MODEM_STATUS_POLL_MS);
        }
        break;
    }
    case MODEM_STATE_RECONNECT: {
        if (trigger_network_attach()) {
            s_state.state = MODEM_STATE_REGISTRATION;
            s_state.reconnect_delay_ms = 1000;
            s_state.next_action_tick = now + pdMS_TO_TICKS(5000);
        } else {
            s_state.reconnect_delay_ms = (s_state.reconnect_delay_ms < 60000) ? s_state.reconnect_delay_ms * 2 : 60000;
            s_state.next_action_tick = now + pdMS_TO_TICKS(s_state.reconnect_delay_ms);
        }
        break;
    }
    case MODEM_STATE_ERROR: {
        ESP_LOGW(TAG, "Modem state machine in error state, attempting reset");
        power_cycle_modem();
        s_state.state = MODEM_STATE_SIM_CHECK;
        s_state.next_action_tick = now + pdMS_TO_TICKS(5000);
        break;
    }
    default: {
        s_state.state = MODEM_STATE_SIM_CHECK;
        s_state.next_action_tick = now + pdMS_TO_TICKS(5000);
        break;
    }
    }
}

extern "C" void gl868_modem_trigger_emergency(const char *source, int32_t value)
{
    ModemLock lock;
    if (!lock.locked) return;
    if (!s_state.initialized) {
        return;
    }

    const char *call_number = get_emergency_call_number();
    const char *sms_number = get_emergency_sms_number();
    ESP_LOGI(TAG, "Emergency call target: %s", call_number);
    ESP_LOGI(TAG, "Emergency SMS target: %s", sms_number);

    std::string sim_status;
    if (!is_sim_ready(&sim_status)) {
        ESP_LOGW(TAG, "SIM not ready for emergency alert: %s", sim_status.c_str());
        return;
    }
    log_sim_status();

    std::string gps_response;
    get_gps_location(&gps_response);

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
    char message[256];
    double lat = 0.0, lon = 0.0;
    bool has_coords = parse_latlon_from_cgnsinf(gps_response, lat, lon);
    if (has_coords) {
        // include Google Maps link
        snprintf(message, sizeof(message), "SOS triggered by %s (%ld). Battery:%d%% GPS: https://maps.google.com/?q=%.6f,%.6f",
                 source ? source : "system", (long)value, batt >= 0 ? batt : -1, lat, lon);
    } else {
        // fallback to raw GPS info string
        snprintf(message, sizeof(message), "SOS triggered by %s (%ld). Battery:%d%% GPS: %s",
                 source ? source : "system", (long)value, batt >= 0 ? batt : -1, trim_response(gps_response).c_str());
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
    if (!any_sent) {
        ESP_LOGW(TAG, "Emergency SMS failed for all recipients; aborting call to %s", call_number);
        return;
    }

    ESP_LOGI(TAG, "Emergency SMS succeeded; emergency voice call will be handled by the voice task");
}

extern "C" bool gl868_modem_send_sms_to(const char *number, const char *message)
{
    ModemLock lock;
    if (!lock.locked || number == nullptr || message == nullptr) return false;
    if (!s_state.initialized && !gl868_modem_init()) return false;
    return send_sms(std::string(number), std::string(message));
}

extern "C" bool gl868_modem_make_call_to(const char *number)
{
    ModemLock lock;
    if (!lock.locked || number == nullptr) return false;
    if (!s_state.initialized && !gl868_modem_init()) return false;
    return make_call(std::string(number));
}

extern "C" bool gl868_modem_hangup_call(void)
{
    ModemLock lock;
    if (!lock.locked) return false;
    if (!s_state.initialized) return false;
    std::string response;
    const bool ok = send_at_command("ATH\r", &response, 5000);
    ESP_LOGI(TAG, "Emergency call hangup: %s", ok ? "accepted" : trim_response(response).c_str());
    return ok;
}

extern "C" bool gl868_modem_enable_gnss(void)
{
    ModemLock lock;
    if (!lock.locked) return false;
    if (!s_state.initialized && !gl868_modem_init()) return false;
    const bool ok = enable_gps();
    if (ok) {
        s_state.gps_enabled = true;
    }
    return ok;
}

extern "C" bool gl868_modem_disable_gnss(void)
{
    ModemLock lock;
    if (!lock.locked) return false;
    if (!s_state.initialized) return false;
    const bool ok = disable_gps();
    if (ok) {
        s_state.gps_enabled = false;
    }
    return ok;
}

extern "C" bool gl868_modem_get_gps_now(char *buf, size_t buf_len)
{
    ModemLock lock;
    if (!lock.locked || buf == nullptr || buf_len == 0) return false;
    if (!s_state.initialized && !gl868_modem_init()) return false;
    if (!s_state.gps_enabled && !enable_gps()) return false;
    std::string gps;
    const bool ok = get_gps_location(&gps);
    const size_t copy_len = gps.size() < (buf_len - 1) ? gps.size() : (buf_len - 1);
    if (copy_len > 0) memcpy(buf, gps.c_str(), copy_len);
    buf[copy_len] = '\0';
    return ok;
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
    ModemLock lock;
    if (!lock.locked) return -1;
    if (!s_state.initialized && !gl868_modem_init()) return -1;
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
