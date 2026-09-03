#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_dte.hpp"
#include "cxx_include/esp_modem_types.hpp"
#include "esp_modem_config.h"
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
// When metadata is missing or accuracy is poor, enable smoothing/fallback
// behavior
static const bool GPS_ENABLE_SMOOTHING_FALLBACK = true;
static const double GPS_SMOOTHING_ALPHA =
    0.3; // legacy EMA weight (kept for tuning)
static const double GPS_HDOP_TO_METERS =
    2.0; // rough multiplier: accuracy (m) ~= hdop * UERE(~5m)
static const int NETWORK_FALLBACK_MAX_ACCURACY_METERS =
    100; // accept network fallback if estimated <= this
static const double GPS_TARGET_ACCURACY_METERS = 5.0;
static const double GPS_MAXIMUM_ACCURACY_METERS = 10.0;
static const uint32_t SOS_GPS_QUALITY_TIMEOUT_MS = 30000;
static const size_t SIM868_CREC_READ_BYTES = 200;
static const char *SIM868_SOS_AUDIO_FILE = "C:\\User\\sos_audio.amr";
static const char *SIM868_SOS_RAW_FILE = "C:\\User\\sos_audio.raw";

struct Sim868State {
  std::shared_ptr<esp_modem::DTE> dte;
  bool initialized = false;
  bool gps_enabled = false;
};

static Sim868State s_state;
static uint32_t s_event_sequence = 0;
static std::string s_current_sos_correlation_id;
static std::string s_last_recorded_sos_audio_path;

struct GpsFixInfo;
static const char *http_upload_device_id(void);
static std::string json_escape(const std::string &value);

static std::string trim_response(const std::string &input) {
  const char *whitespace = "\r\n";
  const size_t start = input.find_first_not_of(whitespace);
  if (start == std::string::npos) {
    return "";
  }
  const size_t end = input.find_last_not_of(whitespace);
  return input.substr(start, end - start + 1);
}

static std::string flatten_response(const std::string &input) {
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

static std::string utc_rfc3339_now(void) {
  time_t now = time(nullptr);
  char buf[32];
  struct tm tm_info;
#if defined(_WIN32)
  gmtime_s(&tm_info, &now);
#else
  gmtime_r(&now, &tm_info);
#endif
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_info);
  return std::string(buf);
}

static std::string build_event_id(const std::string &band_id,
                                  const std::string &event_name) {
  char event_id[128];
  snprintf(event_id, sizeof(event_id), "%s-%s-%lu", band_id.c_str(),
           event_name.c_str(),
           static_cast<unsigned long>((++s_event_sequence) & 0xFFFFFFFFu));
  return std::string(event_id);
}

static std::string smart_band_payload_event_name(const char *event_type) {
  if (std::strcmp(event_type, "live_tracking") == 0)
    return "device.live_location";
  if (std::strcmp(event_type, "sos") == 0)
    return "sos.triggered";
  if (std::strcmp(event_type, "sos.updated") == 0)
    return "sos.updated";
  return std::string(event_type);
}

static std::string smart_band_correlation_id(const std::string &band_id,
                                             const char *event_type) {
  if (event_type != nullptr && std::strcmp(event_type, "sos") == 0) {
    if (s_current_sos_correlation_id.empty()) {
      s_current_sos_correlation_id = "sos-" + utc_rfc3339_now();
    }
    return s_current_sos_correlation_id;
  }
  if (event_type != nullptr && std::strcmp(event_type, "sos.updated") == 0) {
    if (s_current_sos_correlation_id.empty()) {
      s_current_sos_correlation_id = "sos-" + utc_rfc3339_now();
    }
    return s_current_sos_correlation_id;
  }
  return "device:" + band_id;
}

static bool send_at_command(const std::string &cmd, std::string *response,
                            uint32_t timeout_ms,
                            const char *success_marker = nullptr);
static bool wait_for_prompt(const std::string &cmd, const std::string &prompt,
                            std::string *response, uint32_t timeout_ms,
                            char separator = '\n');
static uint32_t get_call_recording_duration_seconds(void);

static std::string smart_band_audio_evidence_uri(const std::string &band_id) {
  return "https://vendor.example/evidence/" + band_id +
         "-sos-audio.amr?signature=demo";
}

static bool delete_modem_audio_file(const std::string &path) {
  if (path.empty() || !s_state.dte) {
    return false;
  }

  std::string response;
  const std::string command = "AT+FSDEL=\"" + path + "\"\r";
  const bool ok = send_at_command(command, &response, 10000);
  if (ok) {
    ESP_LOGI(TAG, "Deleted modem file %s", path.c_str());
    return true;
  }
  ESP_LOGW(TAG, "Unable to delete modem-local SOS audio file %s: %s",
           path.c_str(), trim_response(response).c_str());
  return false;
}

static void log_modem_storage(const char *stage) {
  if (!s_state.dte) {
    return;
  }

  std::string response;
  if (send_at_command("AT+FSMEM\r", &response, 10000)) {
    unsigned long free_bytes = 0;
    const size_t colon = response.find("C:");
    if (colon != std::string::npos &&
        sscanf(response.c_str() + colon + 2, "%lu", &free_bytes) == 1) {
      ESP_LOGI(TAG, "SIM868 storage %s: free=%lu bytes (FSMEM response: %s)",
               stage, free_bytes, flatten_response(response).c_str());
    } else {
      ESP_LOGI(TAG, "SIM868 storage %s: %s", stage,
               flatten_response(response).c_str());
    }
  } else {
    ESP_LOGW(TAG, "SIM868 storage query failed %s: %s", stage,
             flatten_response(response).c_str());
  }
}

static bool clear_modem_user_files(void) {
  if (!s_state.dte) {
    return false;
  }

  std::string dir_response;
  if (!send_at_command("AT+FSLS\r", &dir_response, 10000)) {
    ESP_LOGW(TAG, "Unable to list modem storage before recording: %s",
             flatten_response(dir_response).c_str());
    ESP_LOGW(TAG, "Continuing with direct filename cleanup because this SIM868 "
                  "firmware does not support FSLS arguments");
    delete_modem_audio_file(SIM868_SOS_AUDIO_FILE);
    delete_modem_audio_file(SIM868_SOS_RAW_FILE);
    return true;
  }

  size_t removed = 0;
  size_t search_from = 0;
  while ((search_from = dir_response.find('"', search_from)) !=
         std::string::npos) {
    const size_t end_quote = dir_response.find('"', search_from + 1);
    if (end_quote == std::string::npos) {
      break;
    }

    std::string listed_path =
        dir_response.substr(search_from + 1, end_quote - search_from - 1);
    search_from = end_quote + 1;
    if (listed_path.empty() || listed_path == "." || listed_path == "..") {
      continue;
    }
    ESP_LOGW(TAG, "Removing stale modem file before SOS recording: %s",
             listed_path.c_str());
    if (delete_modem_audio_file(listed_path)) {
      ++removed;
    }
  }

  ESP_LOGI(
      TAG,
      "Modem storage cleanup before SOS recording complete: %u file(s) removed",
      static_cast<unsigned>(removed));
  return true;
}

static bool record_modem_amr_audio_clip(std::string *path_out, size_t *size_out,
                                        std::string *raw_audio_out) {
  if (!s_state.dte) {
    return false;
  }

  const std::string file_path = SIM868_SOS_AUDIO_FILE;
  s_last_recorded_sos_audio_path = file_path;
  log_modem_storage("before in-call recording");

  std::string memory_response;
  unsigned long free_bytes = 0;
  const bool memory_known =
      send_at_command("AT+FSMEM\r", &memory_response, 10000);
  const size_t memory_colon = memory_response.find("C:");
  if (memory_known && memory_colon != std::string::npos) {
    sscanf(memory_response.c_str() + memory_colon + 2, "%lu", &free_bytes);
  }
  const uint32_t recording_duration_seconds =
      get_call_recording_duration_seconds();
  const unsigned long estimated_recording_bytes =
      static_cast<unsigned long>(recording_duration_seconds) * 600UL;
  if (memory_known && free_bytes > 0 &&
      free_bytes < estimated_recording_bytes) {
    ESP_LOGW(TAG,
             "Not enough SIM868 storage for %u-second AMR recording: free=%lu "
             "bytes, estimated_required=%lu bytes",
             recording_duration_seconds, free_bytes, estimated_recording_bytes);
    ESP_LOGW(TAG, "Reduce SAFETY_BAND_CALL_RECORDING_SECONDS or provide more "
                  "modem filesystem space before retrying");
    s_last_recorded_sos_audio_path.clear();
    return false;
  }

  std::string response;
  if (!send_at_command("AT+CREC=1,\"C:\\User\\sos_audio.amr\",1\r", &response,
                       15000)) {
    ESP_LOGW(TAG, "AT+CREC start failed: %s", trim_response(response).c_str());
    log_modem_storage("after recording start failure");
    s_last_recorded_sos_audio_path.clear();
    return false;
  }

  ESP_LOGI(TAG,
           "Call connection confirmed by +COLP; starting SOS audio recording "
           "for %u seconds to %s",
           recording_duration_seconds, file_path.c_str());
  vTaskDelay(pdMS_TO_TICKS(recording_duration_seconds * 1000UL));

  if (!send_at_command("AT+CREC=0\r", &response, 5000)) {
    ESP_LOGW(TAG, "AT+CREC stop failed: %s", trim_response(response).c_str());
    return false;
  }

  if (!send_at_command("ATH\r", &response, 5000)) {
    ESP_LOGW(TAG,
             "Unable to end voice call before accessing stored SOS audio: %s",
             trim_response(response).c_str());
  } else {
    ESP_LOGI(TAG, "Voice call ended; modem filesystem is now available for SOS "
                  "audio readback and upload");
  }

  log_modem_storage("after SOS audio stored");

  ESP_LOGI(TAG,
           "SOS audio recording completed after %u seconds and stored on modem "
           "storage at %s",
           recording_duration_seconds, file_path.c_str());

  std::string dir_response;
  if (!send_at_command("AT+FSLS\r", &dir_response, 10000)) {
    ESP_LOGW(TAG, "Unable to list modem audio directory: %s",
             trim_response(dir_response).c_str());
    return false;
  }

  size_t file_size = 0;
  const std::string marker = "sos_audio.amr";
  const size_t marker_pos = dir_response.find(marker);
  if (marker_pos != std::string::npos) {
    const size_t comma_pos = dir_response.find(',', marker_pos);
    if (comma_pos != std::string::npos) {
      const std::string size_field =
          trim_response(dir_response.substr(comma_pos + 1));
      file_size = static_cast<size_t>(strtoul(size_field.c_str(), nullptr, 10));
    }
  }

  if (path_out != nullptr) {
    *path_out = file_path;
  }
  if (size_out != nullptr) {
    *size_out = file_size;
  }

  ESP_LOGI(TAG,
           "SOS AMR clip stored on modem at %s (%u bytes); preparing server "
           "transfer",
           file_path.c_str(), static_cast<unsigned>(file_size));

  if (raw_audio_out != nullptr && file_size > 0) {
    const std::string read_cmd = "AT+FSREAD=\"C:\\User\\sos_audio.amr\",0," +
                                 std::to_string(file_size) + ",0\r";
    std::string read_response;
    if (send_at_command(read_cmd, &read_response, 30000)) {
      *raw_audio_out = read_response;
      ESP_LOGI(TAG,
               "Recorded AMR clip at %s with %u bytes; raw modem bytes were "
               "read for the evidence workflow.",
               file_path.c_str(), static_cast<unsigned>(file_size));
    } else {
      ESP_LOGW(TAG, "Failed to read recorded AMR clip from %s: %s",
               file_path.c_str(), trim_response(read_response).c_str());
    }
  }

  return true;
}

static void power_cycle_modem() {
  gpio_set_level(GPIO_NUM_42, 0);
  vTaskDelay(pdMS_TO_TICKS(250));
  gpio_set_level(GPIO_NUM_42, 1);
  vTaskDelay(pdMS_TO_TICKS(2500));
}

static void log_sim_status(void);
static void log_call_activity_status(void);
static const char *get_emergency_call_number(void);
static const char *get_emergency_sms_number(void);
static std::vector<std::string> split_recipients(const std::string &list);
static bool run_at_step(const char *label, const std::string &command,
                        uint32_t timeout_ms,
                        const char *success_marker = nullptr);
static bool enable_gps(void);
static bool get_gps_location(std::string *response, uint32_t timeout_ms = 5000);
static bool is_valid_coordinate(double latitude, double longitude);
static double hdop_to_estimated_meters(double hdop);
static bool fetch_network_location(GpsFixInfo &info, uint32_t timeout_ms);
static double parse_gnss_timestamp_seconds(const std::string &ts);
// Constant-velocity 2D Kalman filter: state = [lat, lon, v_lat, v_lon]
struct KalmanCV2D {
  bool initialized = false;
  double x[4];    // state
  double P[4][4]; // covariance
  double Q[4][4]; // process noise

  void reset() {
    initialized = false;
    for (int i = 0; i < 4; ++i) {
      x[i] = 0.0;
      for (int j = 0; j < 4; ++j) {
        P[i][j] = 0.0;
        Q[i][j] = 0.0;
      }
    }
  }

  void init(double lat, double lon, double meas_var) {
    reset();
    x[0] = lat;
    x[1] = lon;
    x[2] = 0.0;
    x[3] = 0.0;
    // initial covariance: position from measurement var, velocity large
    P[0][0] = meas_var;
    P[1][1] = meas_var;
    P[2][2] = 1.0;
    P[3][3] = 1.0;
    initialized = true;
  }

  void set_process_noise(double pos_q, double vel_q) {
    // Q diagonal approximating position & velocity process noise
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        Q[i][j] = 0.0;
    Q[0][0] = pos_q;
    Q[1][1] = pos_q;
    Q[2][2] = vel_q;
    Q[3][3] = vel_q;
  }

  void predict(double dt) {
    if (!initialized)
      return;
    double F[4][4] = {{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    double xnew[4] = {0};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j)
        xnew[i] += F[i][j] * x[j];
    }
    double Pnew[4][4] = {0};
    // Pnew = F*P*F^T + Q
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k)
          for (int l = 0; l < 4; ++l)
            sum += F[i][k] * P[k][l] * F[j][l];
        Pnew[i][j] = sum + Q[i][j];
      }
    for (int i = 0; i < 4; ++i) {
      x[i] = xnew[i];
      for (int j = 0; j < 4; ++j)
        P[i][j] = Pnew[i][j];
    }
  }

  void update(double meas_lat, double meas_lon, double meas_var) {
    if (!initialized) {
      init(meas_lat, meas_lon, meas_var);
      return;
    }
    // H = [ [1,0,0,0], [0,1,0,0] ]
    double Ht[4][2] = {{1, 0}, {0, 1}, {0, 0}, {0, 0}};
    // S = H*P*Ht + R
    double S[2][2] = {0};
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j) {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k)
          for (int l = 0; l < 4; ++l)
            sum += Ht[k][i] * P[k][l] * Ht[l][j];
        S[i][j] = sum;
      }
    S[0][0] += meas_var;
    S[1][1] += meas_var;
    // Compute Kalman gain K = P * H^T * inv(S)
    // First compute PHT (4x2)
    double PHT[4][2] = {0};
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 2; ++j)
        for (int k = 0; k < 4; ++k)
          PHT[i][j] += P[i][k] * Ht[k][j];
    // invert S (2x2)
    double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    double Sinv[2][2] = {0};
    if (fabs(det) < 1e-12) {
      Sinv[0][0] = 1.0 / (S[0][0] + 1e-12);
      Sinv[1][1] = 1.0 / (S[1][1] + 1e-12);
    } else {
      Sinv[0][0] = S[1][1] / det;
      Sinv[0][1] = -S[0][1] / det;
      Sinv[1][0] = -S[1][0] / det;
      Sinv[1][1] = S[0][0] / det;
    }
    double K[4][2] = {0};
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 2; ++j)
        for (int k = 0; k < 2; ++k)
          K[i][j] += PHT[i][k] * Sinv[k][j];
    // y = z - H*x
    double z[2] = {meas_lat, meas_lon};
    double Hx[2] = {x[0], x[1]};
    double y[2] = {z[0] - Hx[0], z[1] - Hx[1]};
    // x = x + K*y
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 2; ++j)
        x[i] += K[i][j] * y[j];
    // P = (I - K*H) * P
    double KH[4][4] = {0};
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        for (int k = 0; k < 2; ++k)
          KH[i][j] += K[i][k] * Ht[j][k];
    double IminusKH[4][4];
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        IminusKH[i][j] = ((i == j) ? 1.0 : 0.0) - KH[i][j];
    double Pnew[4][4] = {0};
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        for (int k = 0; k < 4; ++k)
          Pnew[i][j] += IminusKH[i][k] * P[k][j];
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        P[i][j] = Pnew[i][j];
  }
};

static KalmanCV2D s_kf;
static double s_last_fix_time = 0.0;

enum MovementProfile {
  MP_STATIONARY = 0,
  MP_WALKING = 1,
  MP_RUNNING = 2,
};

static void set_kalman_profile(int profile) {
  double pos_q = 1e-8;   // degrees^2 process noise for position
  double vel_q = 1e-8;   // degrees^2 process noise for velocity
  double pos_var = 1e-8; // initial position variance (deg^2)
  double vel_var = 1e-6; // initial velocity variance
  if (profile == MP_STATIONARY) {
    pos_q = 1e-10;
    vel_q = 1e-10;
    pos_var = 4e-10;
    vel_var = 1e-8;
  } else if (profile == MP_WALKING) {
    pos_q = 5e-9;
    vel_q = 5e-8;
    pos_var = 8e-9;
    vel_var = 8e-11;
  } else if (profile == MP_RUNNING) {
    pos_q = 2e-8;
    vel_q = 5e-7;
    pos_var = 3e-8;
    vel_var = 7e-10;
  } else {
    pos_q = 5e-9;
    vel_q = 5e-8;
    pos_var = 1e-8;
    vel_var = 1e-10;
  }
  // reset then set process noise and seed covariance diagonals
  s_kf.reset();
  s_kf.set_process_noise(pos_q, vel_q);
  s_kf.P[0][0] = pos_var;
  s_kf.P[1][1] = pos_var;
  s_kf.P[2][2] = vel_var;
  s_kf.P[3][3] = vel_var;
}

extern "C" void gl868_modem_set_kalman_params(double q_lat, double q_lon,
                                              double var_lat, double var_lon) {
  s_kf.reset();
  s_kf.set_process_noise(q_lat, q_lon);
  s_kf.P[0][0] = var_lat;
  s_kf.P[1][1] = var_lon;
  s_kf.P[2][2] = 1.0;
  s_kf.P[3][3] = 1.0;
  ESP_LOGI(TAG,
           "Kalman params set: pos_q=%.6g vel_q=%.6g pos_var=%.6g vel_var=%.6g",
           q_lat, q_lon, var_lat, var_lon);
}
static bool wait_for_sim_ready(uint32_t timeout_ms = 15000);
static bool ensure_apn_configured(void);

static std::string gps_status_line(const std::string &response) {
  const size_t start = response.find("+CGNSINF:");
  if (start == std::string::npos)
    return "no +CGNSINF response";
  const size_t end = response.find_first_of("\r\n", start);
  return trim_response(response.substr(
      start, end == std::string::npos ? std::string::npos : end - start));
}

static __attribute__((unused)) const char *get_geolinker_api_key(void) {
#ifdef CONFIG_SAFETY_BAND_GEOLINKER_API_KEY
  return CONFIG_SAFETY_BAND_GEOLINKER_API_KEY;
#else
  return "";
#endif
}

static __attribute__((unused)) const char *get_geolinker_device_id(void) {
#ifdef CONFIG_SAFETY_BAND_GEOLINKER_DEVICE_ID
  return CONFIG_SAFETY_BAND_GEOLINKER_DEVICE_ID[0] != '\0'
             ? CONFIG_SAFETY_BAND_GEOLINKER_DEVICE_ID
             : "smart_safety_band";
#else
  return "smart_safety_band";
#endif
}

static bool is_sim_ready(std::string *out_status = nullptr) {
  std::string response;
  if (!send_at_command("AT+CPIN?\r", &response, 5000)) {
    if (out_status)
      *out_status = trim_response(response);
    return false;
  }
  const std::string trimmed = trim_response(response);
  if (out_status)
    *out_status = trimmed;
  return trimmed.find("READY") != std::string::npos;
}

static bool wait_for_sim_ready(uint32_t timeout_ms) {
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

static bool ensure_apn_configured(void) {
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

  const std::string command =
      std::string("AT+CGDCONT=1,\"IP\",\"") + apn + "\"\r";
  if (!send_at_command(command, &response, 5000)) {
    ESP_LOGW(TAG, "APN configuration failed: %s",
             trim_response(response).c_str());
    return false;
  }
  ESP_LOGI(TAG, "APN configured: %s", apn);
  return true;
#else
  ESP_LOGW(TAG, "No APN configured in sdkconfig; set SAFETY_BAND_GPRS_APN");
  return false;
#endif
}

bool send_at_command(const std::string &cmd, std::string *response,
                     uint32_t timeout_ms, const char *success_marker) {
  if (!s_state.dte) {
    return false;
  }

  std::string buffer;
  bool marker_seen = false;
  auto result = s_state.dte->command(
      cmd,
      [&buffer, &marker_seen,
       success_marker](uint8_t *data, size_t len) -> esp_modem::command_result {
        buffer.append(reinterpret_cast<char *>(data), len);
        const std::string line(reinterpret_cast<char *>(data), len);
        const std::string trimmed = trim_response(line);
        if (trimmed.find("ERROR") != std::string::npos ||
            trimmed.find("FAIL") != std::string::npos ||
            trimmed.find("+CME ERROR") != std::string::npos ||
            trimmed.find("+CMS ERROR") != std::string::npos) {
          return esp_modem::command_result::FAIL;
        }
        if (success_marker != nullptr &&
            trimmed.find(success_marker) != std::string::npos) {
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
      },
      timeout_ms);

  if (response != nullptr) {
    *response = buffer;
  }
  if (result == esp_modem::command_result::OK) {
    return true;
  }
  if (success_marker != nullptr && marker_seen &&
      buffer.find("ERROR") == std::string::npos &&
      buffer.find("FAIL") == std::string::npos &&
      buffer.find("+CME ERROR") == std::string::npos &&
      buffer.find("+CMS ERROR") == std::string::npos) {
    return true;
  }
  return false;
}

static const char *get_emergency_call_number(void) {
#ifdef CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER
  return CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER[0] != '\0'
             ? CONFIG_SAFETY_BAND_EMERGENCY_CALL_NUMBER
             : DEFAULT_EMERGENCY_CALL_NUMBER;
#else
  return DEFAULT_EMERGENCY_CALL_NUMBER;
#endif
}

static const char *get_emergency_sms_number(void) {
  /* Force emergency SMS recipient to the hard-coded emergency SMS number.
   * Do not honor runtime or build-time SMS overrides for emergency alerts. */
  return DEFAULT_EMERGENCY_SMS_NUMBER;
}

static std::vector<std::string> split_recipients(const std::string &list) {
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
    if (!token.empty())
      out.push_back(token);
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
  double altitude_m = -1.0;
  double speed_kph = -1.0;
  double course_deg = -1.0;
  double hdop = -1.0;
  double pdop = -1.0;
  double vdop = -1.0;
  double accuracy_m = 8.0;
  int satellites_in_view = -1;
  int satellites_used = -1;
  std::string timestamp;
};

static bool parse_gps_fix_info_from_cgnsinf(const std::string &cgnsinf,
                                            GpsFixInfo &info) {
  const size_t pos = cgnsinf.find("+CGNSINF:");
  if (pos == std::string::npos)
    return false;
  const size_t line_end = cgnsinf.find('\n', pos);
  const std::string line = (line_end == std::string::npos)
                               ? cgnsinf.substr(pos)
                               : cgnsinf.substr(pos, line_end - pos);
  size_t colon = line.find(':');
  if (colon == std::string::npos)
    return false;
  std::string rest = line.substr(colon + 1);
  const std::string trimmed = trim_response(rest);

  std::vector<std::string> fields;
  size_t s = 0;
  while (s < trimmed.size()) {
    size_t p = trimmed.find(',', s);
    if (p == std::string::npos) {
      fields.push_back(trimmed.substr(s));
      break;
    }
    fields.push_back(trimmed.substr(s, p - s));
    s = p + 1;
  }

  if (fields.size() < 3)
    return false;
  info.run_status = static_cast<int>(strtol(fields[0].c_str(), nullptr, 10));
  info.fix_status = static_cast<int>(strtol(fields[1].c_str(), nullptr, 10));
  info.timestamp = fields[2];

  if (info.fix_status <= 0) {
    info.valid = false;
    info.accuracy_m = 8.0;
    return true;
  }

  if (fields.size() < 5)
    return false;
  char *endptr = nullptr;
  info.latitude = strtod(fields[3].c_str(), &endptr);
  if (endptr == fields[3].c_str())
    return false;
  info.longitude = strtod(fields[4].c_str(), &endptr);
  if (endptr == fields[4].c_str())
    return false;
  if (!is_valid_coordinate(info.latitude, info.longitude)) {
    return false;
  }

  if (fields.size() > 5)
    info.altitude_m = strtod(fields[5].c_str(), nullptr);
  if (fields.size() > 6)
    info.speed_kph = strtod(fields[6].c_str(), nullptr);
  if (fields.size() > 7)
    info.course_deg = strtod(fields[7].c_str(), nullptr);

  if (fields.size() > 8) {
    info.fix_mode = static_cast<int>(strtol(fields[8].c_str(), nullptr, 10));
  }
  if (fields.size() > 10) {
    info.hdop = strtod(fields[10].c_str(), &endptr);
    if (endptr == fields[10].c_str())
      info.hdop = -1.0;
  }
  info.accuracy_m = info.hdop > 0.0 ? hdop_to_estimated_meters(info.hdop) : 8.0;
  if (fields.size() > 11)
    info.pdop = strtod(fields[11].c_str(), nullptr);
  if (fields.size() > 12)
    info.vdop = strtod(fields[12].c_str(), nullptr);
  if (fields.size() > 14) {
    info.satellites_in_view =
        static_cast<int>(strtol(fields[14].c_str(), nullptr, 10));
  }
  if (fields.size() > 15) {
    info.satellites_used =
        static_cast<int>(strtol(fields[15].c_str(), nullptr, 10));
  }
  info.valid = true;
  return true;
}

static std::string build_smart_band_event_json(const char *event_type,
                                               const GpsFixInfo &fix,
                                               int battery) {
  const std::string band_id = http_upload_device_id();
  const std::string normalized_event_type =
      smart_band_payload_event_name(event_type);
  const std::string occurred_at = utc_rfc3339_now();
  const std::string correlation_id =
      smart_band_correlation_id(band_id, event_type);
  std::string payload;
  std::string evidence;
  std::string event_id = build_event_id(band_id, normalized_event_type);

  if (normalized_event_type == "device.heartbeat") {
    payload = "{\"band_id\":\"" + json_escape(band_id) +
              "\","
              "\"status\":\"online\",\"last_seen\":\"" +
              occurred_at +
              "\","
              "\"battery_percent\":" +
              std::to_string(battery >= 0 ? battery : 0) + "}";
    evidence = "[]";
  } else if (normalized_event_type == "sos.triggered") {
    payload = "{\"band_id\":\"" + json_escape(band_id) +
              "\","
              "\"trigger\":\"button\",\"sos_status\":\"triggered\","
              "\"location_source\":\"gps\",\"altitude_m\":null}";
    evidence = "[]";
  } else {
    payload = "{\"band_id\":\"" + json_escape(band_id) +
              "\","
              "\"sos_status\":\"active\",\"update_type\":\"location\","
              "\"sequence\":1,\"location_source\":\"gps\",\"altitude_m\":null}";
    evidence = "[]";
  }

  std::string event =
      "{"
      "\"schema_version\":\"smart-city-event/1.0\","
      "\"event_id\":\"" +
      json_escape(event_id) +
      "\","
      "\"correlation_id\":\"" +
      json_escape(correlation_id) +
      "\","
      "\"event_type\":\"" +
      json_escape(normalized_event_type) +
      "\","
      "\"severity\":\"" +
      std::string(normalized_event_type == "device.heartbeat" ? "info"
                                                              : "critical") +
      "\","
      "\"occurred_at\":\"" +
      occurred_at +
      "\","
      "\"source\":{\"kind\":\"smart-band\",\"source_id\":\"" +
      json_escape(band_id) +
      "\",\"deployment_id\":null},"
      "\"location\":{\"kind\":\"geo\",\"site_id\":\"hyderabad-demo\",\"zone_"
      "id\":null,"
      "\"latitude\":" +
      std::to_string(fix.latitude) +
      ",\"longitude\":" + std::to_string(fix.longitude) +
      ","
      "\"accuracy_m\":" +
      std::to_string(fix.accuracy_m) +
      "},"
      "\"evidence\":" +
      evidence +
      ","
      "\"payload\":" +
      payload + "}";

  return event;
}

static bool is_valid_coordinate(double latitude, double longitude) {
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

static double hdop_to_estimated_meters(double hdop) {
  if (hdop <= 0.0)
    return 1e6;
  return hdop * GPS_HDOP_TO_METERS;
}

static bool gps_fix_meets_accuracy_requirement(const GpsFixInfo &info) {
  return info.valid && info.hdop > 0.0 &&
         hdop_to_estimated_meters(info.hdop) <= GPS_MAXIMUM_ACCURACY_METERS;
}

static bool wait_for_gps_fix(GpsFixInfo &info, uint32_t timeout_ms) {
  if (!s_state.gps_enabled && !enable_gps()) {
    ESP_LOGW(TAG, "GPS power enable failed before fix attempt");
    return false;
  }

  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  std::string first_timestamp;
  int last_run_status = -1;
  int last_fix_status = -1;
  std::string last_timestamp;
  GpsFixInfo best_acceptable_fix;
  double best_acceptable_accuracy = 1e6;
  bool have_acceptable_fix = false;
  while (xTaskGetTickCount() < deadline) {
    std::string gps_response;
    if (!get_gps_location(&gps_response, 5000)) {
      ESP_LOGI(TAG, "GPS query failed during fix wait: %s",
               gps_status_line(gps_response).c_str());
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (!parse_gps_fix_info_from_cgnsinf(gps_response, info)) {
      ESP_LOGW(TAG, "GPS response parse failed: %s",
               gps_status_line(gps_response).c_str());
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (info.run_status != last_run_status ||
        info.fix_status != last_fix_status ||
        info.timestamp != last_timestamp) {
      if (info.run_status != 1) {
        ESP_LOGW(TAG, "GPS fix unavailable: run=%d fix_status=%d timestamp=%s",
                 info.run_status, info.fix_status, info.timestamp.c_str());
      } else if (info.fix_status <= 0) {
        ESP_LOGW(TAG, "GPS fix unavailable: run=%d fix_status=%d timestamp=%s",
                 info.run_status, info.fix_status, info.timestamp.c_str());
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

    if (!first_timestamp.empty() && !info.timestamp.empty() &&
        info.timestamp == first_timestamp) {
      if (info.timestamp != last_timestamp) {
        ESP_LOGI(TAG, "GPS hot-start fix stale (%s); waiting for current fix",
                 info.timestamp.c_str());
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (first_timestamp.empty() && !info.timestamp.empty()) {
      first_timestamp = info.timestamp;
      ESP_LOGI(TAG, "Hot-start fix (stale): %s — waiting for current",
               info.timestamp.c_str());
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    const double estimated_accuracy = hdop_to_estimated_meters(info.hdop);
    if (!gps_fix_meets_accuracy_requirement(info)) {
      ESP_LOGI(TAG,
               "GPS fix rejected: estimated accuracy %.1f m exceeds %.1f m "
               "(hdop=%.2f)",
               estimated_accuracy, GPS_MAXIMUM_ACCURACY_METERS, info.hdop);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (estimated_accuracy <= GPS_TARGET_ACCURACY_METERS) {
      ESP_LOGI(TAG,
               "GPS fix accepted at preferred accuracy: %.6f, %.6f (estimated "
               "%.1f m; sats=%d; hdop=%.2f)",
               info.latitude, info.longitude, estimated_accuracy,
               info.satellites_used, info.hdop);
      return true;
    }

    if (!have_acceptable_fix || estimated_accuracy < best_acceptable_accuracy) {
      best_acceptable_fix = info;
      best_acceptable_accuracy = estimated_accuracy;
      have_acceptable_fix = true;
      ESP_LOGI(TAG,
               "GPS fix meets the 10 m limit (estimated %.1f m); continuing to "
               "seek the %.1f m target",
               estimated_accuracy, GPS_TARGET_ACCURACY_METERS);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  if (have_acceptable_fix) {
    info = best_acceptable_fix;
    ESP_LOGI(TAG,
             "GPS fix accepted at timeout: %.6f, %.6f (best estimated %.1f m; "
             "target %.1f m)",
             info.latitude, info.longitude, best_acceptable_accuracy,
             GPS_TARGET_ACCURACY_METERS);
    return true;
  }

  ESP_LOGW(TAG, "GPS fix timeout");
  return false;
}

static void log_sim_status(void) {
  std::string response;
  if (send_at_command("AT+CPIN?\r", &response, 6000)) {
    ESP_LOGI(TAG, "SIM status: %s", trim_response(response).c_str());
  } else {
    ESP_LOGW(TAG, "SIM status query failed: %s",
             trim_response(response).c_str());
  }

  if (send_at_command("AT+CCID?\r", &response, 6000)) {
    ESP_LOGI(TAG, "SIM ICCID: %s", trim_response(response).c_str());
  }

  if (send_at_command("AT+CSQ\r", &response, 6000)) {
    ESP_LOGI(TAG, "Signal quality: %s", trim_response(response).c_str());
  }
}

static bool parse_registration_response(const std::string &response) {
  const std::string trimmed = trim_response(response);
  return trimmed.find(",1") != std::string::npos ||
         trimmed.find(",5") != std::string::npos;
}

static void report_registration_query(const char *label,
                                      const std::string &response) {
  const std::string trimmed = trim_response(response);
  ESP_LOGI(TAG, "%s response: %s", label, trimmed.c_str());
}

static __attribute__((unused)) bool log_network_registration(void) {
  std::string creg_response;
  const bool creg_ok = send_at_command("AT+CREG?\r", &creg_response, 5000);
  report_registration_query("AT+CREG?", creg_response);

  std::string cgreg_response;
  const bool cgreg_ok = send_at_command("AT+CGREG?\r", &cgreg_response, 5000);
  report_registration_query("AT+CGREG?", cgreg_response);

  const bool creg_registered =
      creg_ok && parse_registration_response(creg_response);
  const bool cgreg_registered =
      cgreg_ok && parse_registration_response(cgreg_response);
  const bool registered = creg_registered || cgreg_registered;
  ESP_LOGI(TAG, "GSM network registration status: %s (CREG=%s, CGREG=%s)",
           registered ? "READY" : "NOT READY",
           creg_registered ? "READY" : "NOT READY",
           cgreg_registered ? "READY" : "NOT READY");
  return registered;
}

static __attribute__((unused)) bool is_network_registered(void) {
  std::string response;
  if (send_at_command("AT+CREG?\r", &response, 5000)) {
    const std::string result = trim_response(response);
    if (result.find(",1") != std::string::npos ||
        result.find(",5") != std::string::npos) {
      return true;
    }
  }
  if (send_at_command("AT+CGREG?\r", &response, 5000)) {
    const std::string result = trim_response(response);
    if (result.find(",1") != std::string::npos ||
        result.find(",5") != std::string::npos) {
      return true;
    }
  }
  return false;
}

static bool wait_for_network_registration(uint32_t timeout_ms) {
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  bool saw_signal_issue = false;
  while (xTaskGetTickCount() < deadline) {
    std::string creg_response;
    const bool creg_ok = send_at_command("AT+CREG?\r", &creg_response, 5000);
    std::string cgreg_response;
    const bool cgreg_ok = send_at_command("AT+CGREG?\r", &cgreg_response, 5000);

    const bool creg_registered =
        creg_ok && parse_registration_response(creg_response);
    const bool cgreg_registered =
        cgreg_ok && parse_registration_response(cgreg_response);
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
      ESP_LOGW(TAG, "Cellular modem is not registered yet; signal or carrier "
                    "conditions may be poor");
      saw_signal_issue = true;
    }

    if ((xTaskGetTickCount() + pdMS_TO_TICKS(10000)) >= deadline) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }

  ESP_LOGW(TAG, "GSM registration timeout after %u ms; checking SIM/APN state",
           timeout_ms);
  ensure_apn_configured();

  std::string csq_response;
  if (send_at_command("AT+CSQ\r", &csq_response, 5000)) {
    ESP_LOGW(TAG, "Final signal quality after registration timeout: %s",
             trim_response(csq_response).c_str());
  } else {
    ESP_LOGW(TAG, "Unable to query signal quality after registration timeout");
  }
  return false;
}

static void log_call_activity_status(void) {
  std::string response;

  if (send_at_command("AT+CPAS\r", &response, 4000, "+CPAS:")) {
    ESP_LOGI(TAG, "Phone activity status: %s", trim_response(response).c_str());
  } else {
    ESP_LOGI(TAG, "Phone activity status query returned no usable response: %s",
             trim_response(response).c_str());
  }
}

static void log_call_preflight(void) {
  std::string response;
  if (send_at_command("AT+CSQ\r", &response, 5000, "+CSQ:")) {
    ESP_LOGI(TAG, "Signal quality before call: %s",
             trim_response(response).c_str());
  } else {
    ESP_LOGW(TAG, "Unable to query signal quality before call: %s",
             trim_response(response).c_str());
  }
  if (send_at_command("AT+CPAS\r", &response, 4000, "+CPAS:")) {
    ESP_LOGI(TAG, "Phone activity before call: %s",
             trim_response(response).c_str());
  } else {
    ESP_LOGW(TAG, "Unable to query phone activity before call: %s",
             trim_response(response).c_str());
  }
}

static bool clcc_has_connected_call(const std::string &response) {
  size_t pos = 0;
  while ((pos = response.find("+CLCC:", pos)) != std::string::npos) {
    const size_t line_end = response.find('\n', pos);
    const std::string line =
        response.substr(pos, line_end == std::string::npos ? std::string::npos
                                                           : line_end - pos);
    pos = line_end == std::string::npos ? std::string::npos : line_end + 1;

    const size_t colon = line.find(':');
    if (colon == std::string::npos)
      continue;
    std::string rest = line.substr(colon + 1);
    size_t field_start = 0;
    int field_index = 0;
    while (field_start < rest.size()) {
      const size_t comma = rest.find(',', field_start);
      const std::string field = trim_response(rest.substr(
          field_start, comma == std::string::npos ? std::string::npos
                                                  : comma - field_start));
      if (field_index == 2) {
        return atoi(field.c_str()) == 0;
      }
      ++field_index;
      if (comma == std::string::npos)
        break;
      field_start = comma + 1;
    }
  }
  return false;
}

static bool configure_audio_path(void) {
  /*
   * The GeoLinker board exposes its external speaker/microphone on the
   * SIM868 AUX audio path.  Keep every per-channel setting on that same
   * path.  +FMMUTE is an FM-radio command and is not implemented by this
   * firmware.  This firmware also rejects +CMUT while the modem is idle;
   * it is not required because call audio starts unmuted.  +SIDET always
   * needs both the audio channel and the side-tone gain.
   */
  const char *commands[] = {
      "AT+CHFA=1\r",
      "AT+CLVL=90\r",
      "AT+CMIC=1,12\r",
      "AT+SIDET=1,1\r",
  };

  bool configured = true;
  for (const char *command : commands) {
    std::string response;
    if (!send_at_command(command, &response, 3000)) {
      ESP_LOGW(TAG, "Audio command failed (%s): %s",
               trim_response(command).c_str(), trim_response(response).c_str());
      configured = false;
    }
  }
  return configured;
}

static std::string sim868_timestamp_for_filename(void) {
  std::string response;
  if (!send_at_command("AT+CCLK?\r", &response, 5000, "+CCLK:")) {
    ESP_LOGW(TAG, "SIM868 clock unavailable for recording name: %s",
             trim_response(response).c_str());
    return "00000000_000000";
  }

  const size_t marker = response.find("+CCLK:");
  const size_t quote = response.find('"', marker);
  if (marker == std::string::npos || quote == std::string::npos) {
    ESP_LOGW(TAG, "Unexpected SIM868 clock response: %s",
             trim_response(response).c_str());
    return "00000000_000000";
  }

  int yy = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  if (sscanf(response.c_str() + quote + 1, "%2d/%2d/%2d,%2d:%2d:%2d", &yy,
             &month, &day, &hour, &minute, &second) != 6 ||
      yy > 99 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 ||
      minute > 59 || second > 59) {
    ESP_LOGW(TAG, "Unable to parse SIM868 clock response: %s",
             trim_response(response).c_str());
    return "00000000_000000";
  }

  char timestamp[16];
  size_t pos = 0;
  const auto append_two_digits = [&timestamp, &pos](int value) {
    timestamp[pos++] = static_cast<char>('0' + (value / 10));
    timestamp[pos++] = static_cast<char>('0' + (value % 10));
  };
  timestamp[pos++] = '2';
  timestamp[pos++] = '0';
  append_two_digits(yy);
  append_two_digits(month);
  append_two_digits(day);
  timestamp[pos++] = '_';
  append_two_digits(hour);
  append_two_digits(minute);
  append_two_digits(second);
  timestamp[pos] = '\0';
  return timestamp;
}

static void log_call_failure_details(void) {
  std::string response;
  if (send_at_command("AT+CEER\r", &response, 5000)) {
    ESP_LOGI(TAG, "Call extended error report: %s",
             trim_response(response).c_str());
  } else {
    ESP_LOGI(TAG, "Call extended error report not available; call may have "
                  "completed successfully or no extended error was reported");
  }
}

bool wait_for_prompt(const std::string &cmd, const std::string &prompt,
                     std::string *response, uint32_t timeout_ms,
                     char separator) {
  if (!s_state.dte) {
    return false;
  }

  std::string buffer;
  auto result = s_state.dte->command(
      cmd,
      [&buffer, &prompt](uint8_t *data,
                         size_t len) -> esp_modem::command_result {
        buffer.append(reinterpret_cast<char *>(data), len);
        const std::string line(reinterpret_cast<char *>(data), len);
        const std::string trimmed = trim_response(line);
        if (trimmed.find(prompt) != std::string::npos) {
          return esp_modem::command_result::OK;
        }
        if (trimmed.find("ERROR") != std::string::npos ||
            trimmed.find("FAIL") != std::string::npos ||
            trimmed.find("+CME ERROR") != std::string::npos ||
            trimmed.find("+CMS ERROR") != std::string::npos) {
          return esp_modem::command_result::FAIL;
        }
        return esp_modem::command_result::TIMEOUT;
      },
      timeout_ms, separator);

  if (response != nullptr) {
    *response = buffer;
  }
  return result == esp_modem::command_result::OK;
}

static bool is_http_upload_enabled(void) {
#if defined(CONFIG_SAFETY_BAND_HTTP_UPLOAD) && !CONFIG_SAFETY_BAND_HTTP_UPLOAD
  return false;
#else
  return true;
#endif
}

static const char *http_upload_url(void) {
#ifdef CONFIG_SAFETY_BAND_HTTP_URL
  const char *configured_url = CONFIG_SAFETY_BAND_HTTP_URL;
  if (configured_url != nullptr && configured_url[0] != '\0' &&
      std::strncmp(configured_url, "http://127.0.0.1", 16) != 0 &&
      std::strncmp(configured_url, "http://localhost", 16) != 0 &&
      std::strncmp(configured_url, "https://127.0.0.1", 17) != 0 &&
      std::strncmp(configured_url, "https://localhost", 17) != 0) {
    return configured_url;
  }
#endif
  // return "http://my-esp32-test.free.beeceptor.com";
  // return "http://echo.free.beeceptor.com";
  return "http://my-esp32-test.free.beeceptor.com/api/v1/events/normalized";
}

static const char *http_upload_device_id(void) {
#ifdef CONFIG_SAFETY_BAND_HTTP_DEVICE_ID
  return CONFIG_SAFETY_BAND_HTTP_DEVICE_ID;
#else
  return "smart_safety_band_001";
#endif
}

static std::string json_escape(const std::string &value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (char c : value) {
    if (c == '\\' || c == '"')
      escaped.push_back('\\');
    if (c == '\n')
      escaped += "\\n";
    else if (c == '\r')
      escaped += "\\r";
    else if (c != '\n' && c != '\r')
      escaped.push_back(c);
  }
  return escaped;
}

static bool open_gprs_bearer(void) {
  if (!ensure_apn_configured())
    return false;

#ifdef CONFIG_SAFETY_BAND_GPRS_APN
  const char *apn = CONFIG_SAFETY_BAND_GPRS_APN;
#else
  const char *apn = "airtel.in";
#endif
  if (apn == nullptr || apn[0] == '\0')
    return false;

  std::string response;

  /* 1. Ensure GPRS Attached (AT+CGATT=1) with retries */
  uint8_t retries = 0;
  bool gprs_attached = false;
  while (retries < 5) {
    std::string cgatt_resp;
    if (send_at_command("AT+CGATT?\r", &cgatt_resp, 5000, "+CGATT:") &&
        cgatt_resp.find("+CGATT: 1") != std::string::npos) {
      gprs_attached = true;
      break;
    }
    ESP_LOGI(TAG, "Attaching GPRS data context (attempt %d/5)...", retries + 1);
    send_at_command("AT+CGATT=1\r", &response, 15000);
    vTaskDelay(pdMS_TO_TICKS(3000));
    retries++;
  }

  if (!gprs_attached) {
    ESP_LOGE(TAG,
             "[CRITICAL] Failed to attach GPRS. Check SIM data plan & APN "
             "setting (%s)!",
             apn);
    return false;
  }

  /* 2. Network stabilization delay (2 seconds for Airtel IP routing
   * stabilization) */
  vTaskDelay(pdMS_TO_TICKS(2000)); /* 3. Refresh stale GPRS bearer context to
                                      avoid Airtel cell tower NAT expiration */
  if (send_at_command("AT+SAPBR=2,1\r", &response, 5000, "+SAPBR:") &&
      response.find("+SAPBR: 1,1") != std::string::npos) {
    /* Close 2-minute stale PDP bearer context so we get a fresh connection */
    send_at_command("AT+SAPBR=0,1\r", &response, 3000);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  /* 4. Configure SAPBR bearer parameters (Contype and APN) */
  send_at_command("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r", &response, 5000);
  send_at_command(std::string("AT+SAPBR=3,1,\"APN\",\"") + apn + "\"\r",
                  &response, 5000);

  /* 5. Open bearer context (AT+SAPBR=1,1) */
  bool bearer_open = send_at_command("AT+SAPBR=1,1\r", &response, 85000);
  if (!bearer_open) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    bearer_open = send_at_command("AT+SAPBR=1,1\r", &response, 85000);
  }

  /* 6. Confirm GPRS bearer status and assigned IP via AT+SAPBR=2,1 */
  std::string status_resp;
  if (send_at_command("AT+SAPBR=2,1\r", &status_resp, 5000, "+SAPBR:") &&
      status_resp.find("+SAPBR: 1,1") != std::string::npos) {
    ESP_LOGI(TAG, "GPRS bearer ready (APN=%s): %s", apn,
             flatten_response(status_resp).c_str());

    /* Set public DNS servers (Google 8.8.8.8 / 8.8.4.4) to ensure hostname
     * resolution */
    std::string dns_resp;
    send_at_command("AT+DNSSERVER=\"8.8.8.8\",\"8.8.4.4\"\r", &dns_resp, 3000);
    send_at_command("AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"\r", &dns_resp, 3000);

    /* GSM Base Station Location Query (AT+CIPGSMLOC=1,1) */
    std::string loc_resp;
    send_at_command("AT+CIPGSMLOC=1,1\r", &loc_resp, 10000);

    return true;
  }

  ESP_LOGW(
      TAG, "GPRS bearer open failed (APN=%s). SAPBR status: %s, response: %s",
      apn, trim_response(status_resp).c_str(), trim_response(response).c_str());
  return false;
}

static std::string http_read_body(const std::string &response) {
  const size_t marker = response.find("+HTTPREAD:");
  if (marker == std::string::npos)
    return "";
  const size_t body_start = response.find('\n', marker);
  if (body_start == std::string::npos)
    return "";
  const size_t ok_marker = response.rfind("\r\nOK");
  const size_t body_end =
      ok_marker == std::string::npos ? response.size() : ok_marker;
  return trim_response(
      response.substr(body_start + 1, body_end - body_start - 1));
}

static int post_http_json_attempt(const char *url, const std::string &payload) {
  if (!open_gprs_bearer()) {
    ESP_LOGE(TAG, "Failed to open GPRS bearer for HTTP/HTTPS upload");
    return -1;
  }

  std::string response;
  /* Always terminate any lingering HTTP session before starting */
  send_at_command("AT+HTTPTERM\r", &response, 2000);

  bool ok = send_at_command("AT+HTTPINIT\r", &response, 10000);
  if (!ok) {
    send_at_command("AT+HTTPTERM\r", &response, 2000);
    ok = send_at_command("AT+HTTPINIT\r", &response, 10000);
  }

  if (!ok) {
    ESP_LOGW(TAG, "HTTP initialization failed: %s",
             trim_response(response).c_str());
    return -1;
  }

  const bool is_https = std::strncmp(url, "https://", 8) == 0;
  if (is_https) {
    send_at_command("AT+SSLOPT=1,1\r", &response, 3000);
    send_at_command("AT+CSSLCFG=\"sslversion\",0,3\r", &response, 3000);
    send_at_command("AT+CSSLCFG=\"authmode\",0,0\r", &response, 3000);
    if (!send_at_command("AT+HTTPSSL=1\r", &response, 10000)) {
      ok = false;
    }
  } else {
    send_at_command("AT+HTTPSSL=0\r", &response, 5000);
  }

  if (ok)
    ok = send_at_command("AT+HTTPPARA=\"CID\",1\r", &response, 5000);
  if (ok)
    ok = send_at_command(std::string("AT+HTTPPARA=\"URL\",\"") + url + "\"\r",
                         &response, 10000);
  if (ok)
    ok = send_at_command("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r",
                         &response, 5000);
  if (ok)
    ok = send_at_command("AT+HTTPPARA=\"TIMEOUT\",30\r", &response, 5000);
  if (ok)
    ok = send_at_command(
        "AT+HTTPPARA=\"USERDATA\",\"ngrok-skip-browser-warning: 1\"\r",
        &response, 5000);
  if (ok)
    ok = send_at_command("AT+HTTPPARA=\"REDIR\",1\r", &response, 5000);

  if (ok) {
    const std::string data_cmd =
        "AT+HTTPDATA=" + std::to_string(payload.size()) + ",10000\r";
    ok = wait_for_prompt(data_cmd, "DOWNLOAD", &response, 15000);
    if (!ok) {
      ESP_LOGE(TAG, "AT+HTTPDATA prompt 'DOWNLOAD' not received: %s",
               trim_response(response).c_str());
    }
  }

  if (ok) {
    ok = send_at_command(payload, &response, 15000);
  }

  if (ok) {
    ESP_LOGI(TAG, "Sending HTTP POST request (AT+HTTPACTION=1) to %s...", url);
    ok = send_at_command("AT+HTTPACTION=1\r", &response, 60000, "+HTTPACTION:");
  }

  int method = -1;
  int status = -1;
  int body_length = -1;
  const size_t action_pos = response.find("+HTTPACTION:");
  const bool action_parsed =
      (action_pos != std::string::npos) &&
      (sscanf(response.c_str() + action_pos, "+HTTPACTION: %d,%d,%d", &method,
              &status, &body_length) == 3);

  std::string server_body;
  if (action_parsed && body_length > 0) {
    const size_t read_length =
        std::min(static_cast<size_t>(body_length), static_cast<size_t>(1024));
    std::string read_response;
    if (send_at_command("AT+HTTPREAD=0," + std::to_string(read_length) + "\r",
                        &read_response, 30000)) {
      server_body = http_read_body(read_response);
    }
  }

  bool success = (ok && action_parsed && status >= 200 && status < 300);

  if (success) {
    ESP_LOGI(TAG,
             "HTTP POST SUCCESS [HTTP status %d]! Server ACK Received (%d "
             "bytes): %s",
             status, body_length,
             server_body.empty() ? "<empty>" : server_body.c_str());

    if (!s_last_recorded_sos_audio_path.empty()) {
      ESP_LOGI(TAG, "Server ACK confirmed; cleaning up modem SOS audio file %s",
               s_last_recorded_sos_audio_path.c_str());
      if (delete_modem_audio_file(s_last_recorded_sos_audio_path)) {
        log_modem_storage("after server ACK cleanup");
      }
      s_last_recorded_sos_audio_path.clear();
    }
  } else {
    if (status == 429) {
      ESP_LOGW(TAG,
               "HTTP 429 Too Many Requests: Server testing endpoint "
               "rate-limited incoming requests: %s",
               server_body.c_str());
    } else if (status == 601) {
      ESP_LOGW(TAG,
               "SIM868 HTTP status 601: connection to %s failed (cellular "
               "network flicker).",
               url);
    } else if (status == 603) {
      ESP_LOGE(TAG, "SIM868 HTTP status 603: DNS resolution failed for %s.",
               url);
    } else if (status == 606) {
      ESP_LOGE(TAG, "SIM868 HTTP status 606: SSL handshake failed for %s.",
               url);
    } else if (status > 0) {
      ESP_LOGW(TAG, "HTTP POST returned status code %d. Response: %s", status,
               server_body.c_str());
    } else {
      ESP_LOGW(TAG, "HTTP POST execution timed out/failed. Modem response: %s",
               flatten_response(response).c_str());
    }
  }

  /* Terminate HTTP session cleanly after transaction */
  send_at_command("AT+HTTPTERM\r", &response, 5000);
  return action_parsed ? status : -1;
}

static bool post_http_json(const std::string &payload) {
  if (!is_http_upload_enabled()) {
    ESP_LOGI(TAG, "HTTP upload is disabled in configuration");
    return true;
  }

  const char *url = http_upload_url();
  if (url == nullptr || url[0] == '\0') {
    ESP_LOGE(TAG, "HTTP upload URL is empty");
    return false;
  }

  if (std::strncmp(url, "http://127.0.0.1", 16) == 0 ||
      std::strncmp(url, "http://localhost", 16) == 0 ||
      std::strncmp(url, "https://127.0.0.1", 17) == 0 ||
      std::strncmp(url, "https://localhost", 17) == 0) {
    url = "http://my-esp32-test.free.beeceptor.com";
  }

  if (payload.size() > 4096) {
    ESP_LOGW(TAG, "HTTP payload is too large: %u bytes",
             static_cast<unsigned>(payload.size()));
    return false;
  }

  /* Attempt up to 3 retries for high cellular reliability */
  for (int attempt = 1; attempt <= 3; attempt++) {
    if (attempt > 1) {
      ESP_LOGI(TAG, "Retrying HTTP POST telemetry upload (attempt %d/3)...",
               attempt);
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
    int status = post_http_json_attempt(url, payload);
    if (status >= 200 && status < 300) {
      return true;
    }
    if (status == 429) {
      ESP_LOGW(TAG, "Testing endpoint rate-limited (HTTP 429); skipping "
                    "further retries for this packet.");
      return false;
    }
  }

  ESP_LOGE(
      TAG,
      "[CRITICAL] HTTP POST telemetry upload failed after 3 attempts to %s",
      url);
  return false;
}

static std::string telemetry_json_fields(const char *event_type,
                                         const GpsFixInfo &fix, int battery) {
  const std::string payload =
      build_smart_band_event_json(event_type, fix, battery);
  return payload;
}

static bool post_telemetry_packet(const char *event_type, const GpsFixInfo &fix,
                                  int battery) {
  const std::string payload =
      build_smart_band_event_json(event_type, fix, battery);
  ESP_LOGI(TAG, "Posting normalized smart-band event to %s: %s",
           http_upload_url(), payload.c_str());
  return post_http_json(payload);
}

static uint32_t get_call_recording_duration_seconds(void) {
#ifdef CONFIG_SAFETY_BAND_CALL_RECORDING_SECONDS
  return CONFIG_SAFETY_BAND_CALL_RECORDING_SECONDS;
#else
  return 60;
#endif
}

static size_t get_audio_upload_chunk_bytes(void) {
#ifdef CONFIG_SAFETY_BAND_AUDIO_UPLOAD_CHUNK_BYTES
  return CONFIG_SAFETY_BAND_AUDIO_UPLOAD_CHUNK_BYTES;
#else
  return 1024;
#endif
}

static bool record_and_upload_answered_sos_call(void) {
  if (!s_state.dte) {
    return false;
  }

  std::string recorded_path;
  size_t recorded_size = 0;
  std::string raw_audio;
  const bool recorded =
      record_modem_amr_audio_clip(&recorded_path, &recorded_size, &raw_audio);
  if (!recorded) {
    ESP_LOGW(TAG, "Failed to record the modem-local SOS audio clip");
    return false;
  }

  ESP_LOGI(TAG,
           "SIM868 SOS AMR clip recorded to %s (%u bytes); raw bytes were read "
           "from the modem for local evidentiary review.",
           recorded_path.c_str(), static_cast<unsigned>(recorded_size));
  if (!raw_audio.empty()) {
    ESP_LOGI(
        TAG,
        "Read %u raw AMR bytes from the modem storage; the normalized server "
        "payload keeps the evidence URI instead of embedding raw binary bytes.",
        static_cast<unsigned>(raw_audio.size()));
  }
  return true;
}

bool send_sms(const std::string &number, const std::string &message) {
  if (!s_state.dte) {
    return false;
  }

  std::string response;
  if (!send_at_command("AT+CMGF=1\r", &response, 5000)) {
    ESP_LOGW(TAG, "Failed to set SMS text mode: %s",
             trim_response(response).c_str());
    return false;
  }

  std::string sms_cmd = "AT+CMGS=\"" + number + "\"\r";
  if (!wait_for_prompt(sms_cmd, ">", &response, 10000, '>')) {
    ESP_LOGW(TAG, "SMS command failed or timed out waiting for prompt: %s",
             trim_response(response).c_str());
    return false;
  }

  if (response.find('>') == std::string::npos) {
    ESP_LOGW(TAG, "Modem did not return SMS prompt: %s",
             trim_response(response).c_str());
    return false;
  }

  const std::string payload = message + std::string(1, '\x1A');
  std::string final_response;
  auto result = s_state.dte->command(
      payload,
      [&final_response](uint8_t *data,
                        size_t len) -> esp_modem::command_result {
        final_response.append(reinterpret_cast<char *>(data), len);
        const std::string line(reinterpret_cast<char *>(data), len);
        const std::string trimmed = trim_response(line);
        if (trimmed.find("OK") != std::string::npos) {
          return esp_modem::command_result::OK;
        }
        if (trimmed.find("ERROR") != std::string::npos ||
            trimmed.find("FAIL") != std::string::npos ||
            trimmed.find("+CME ERROR") != std::string::npos ||
            trimmed.find("+CMS ERROR") != std::string::npos) {
          return esp_modem::command_result::FAIL;
        }
        return esp_modem::command_result::TIMEOUT;
      },
      15000, '\n');

  if (result != esp_modem::command_result::OK) {
    ESP_LOGW(TAG, "SMS send failed during final write: %d, response: %s",
             static_cast<int>(result), trim_response(final_response).c_str());
    return false;
  }

  const std::string final_trim = trim_response(final_response);
  if (final_trim.find("OK") == std::string::npos) {
    ESP_LOGW(TAG, "SMS send response indicates failure: %s",
             final_trim.c_str());
    return false;
  }

  ESP_LOGI(TAG, "SMS send succeeded: %s", final_trim.c_str());
  return true;
}

bool make_call(const std::string &number) {
  std::string response;
  if (!configure_audio_path()) {
    ESP_LOGW(TAG, "Audio path setup before dialing was not fully acknowledged");
  }

  std::string cmd = "ATD" + number + ";\r";
  ESP_LOGI(TAG, "Issuing emergency call command: %s", cmd.c_str());
  const bool ok = send_at_command(cmd, &response, 15000);
  const std::string trimmed = trim_response(response);
  ESP_LOGI(TAG, "Emergency call raw response: %s", trimmed.c_str());

  bool explicit_error = false;
  if (trimmed.find("ERROR") != std::string::npos ||
      trimmed.find("FAIL") != std::string::npos ||
      trimmed.find("+CME ERROR") != std::string::npos ||
      trimmed.find("+CMS ERROR") != std::string::npos) {
    explicit_error = true;
  }

  if (!ok && explicit_error) {
    ESP_LOGW(TAG, "Emergency call command failed: %s", trimmed.c_str());
    return false;
  }

  if (!ok && !explicit_error) {
    ESP_LOGI(TAG, "Emergency call command completed without explicit OK; "
                  "verifying call state");
  }

  /* Some SIM868 firmware reports an active MO call with the +COLP URC but
   * only returns the abbreviated '+CLCC: 0' form for AT+CLCC?.  +COLP is
   * Connected Line Identification Presentation and is emitted after the
   * called party is connected, so use it as the reliable fallback here. */
  if (response.find("+COLP:") != std::string::npos) {
    ESP_LOGI(
        TAG,
        "Call connection confirmed by +COLP; starting SOS audio recording");
    if (!configure_audio_path()) {
      ESP_LOGW(TAG, "Audio path setup after +COLP was not fully acknowledged");
    }
    return true;
  }

  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(30000);
  while (xTaskGetTickCount() < deadline) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    std::string clcc_response;
    if (send_at_command("AT+CLCC?\r", &clcc_response, 5000)) {
      ESP_LOGI(TAG, "Post-dial call list: %s",
               trim_response(clcc_response).c_str());
      if (clcc_response.find("+COLP:") != std::string::npos) {
        ESP_LOGI(TAG, "Call connection confirmed by +COLP during call-state "
                      "polling; starting SOS audio recording");
        if (!configure_audio_path()) {
          ESP_LOGW(
              TAG,
              "Audio path setup after polled +COLP was not fully acknowledged");
        }
        return true;
      }
      if (clcc_has_connected_call(clcc_response)) {
        ESP_LOGI(
            TAG,
            "Call answered; configuring external speaker and microphone audio");
        if (!configure_audio_path()) {
          ESP_LOGW(TAG, "Audio path setup after call connection was not fully "
                        "acknowledged");
        }
        return true;
      }
    }
  }

  std::string pas_response;
  if (send_at_command("AT+CPAS\r", &pas_response, 5000, "+CPAS:")) {
    ESP_LOGI(TAG, "Phone activity after dial timeout: %s",
             trim_response(pas_response).c_str());
  }

  if (trimmed.find("NO CARRIER") != std::string::npos) {
    ESP_LOGI(TAG, "Call command returned NO CARRIER; confirming active call "
                  "state with AT+CLCC?/AT+CPAS?");
  }

  return false;
}

bool enable_gps(void) {
  std::string response;
  if (!send_at_command("AT+CGNSPWR=1\r", &response, 3000)) {
    ESP_LOGW(TAG, "GPS power on: FAILED -> %s",
             trim_response(response).c_str());
    return false;
  }
  ESP_LOGI(TAG, "GPS power on: SUCCESS");
  // Request richer GNSS outputs (GGA/GSA/GSV/RMC) so HDOP and satellite info
  // are available when the modem supports them. Failure is non-fatal.
  if (!send_at_command("AT+CGNSSEQ=\"GGA\"\r", &response, 3000)) {
    ESP_LOGW(TAG, "GPS sequence configuration failed -> %s",
             trim_response(response).c_str());
  } else {
    ESP_LOGI(TAG, "GPS sequence configured for richer metadata");
  }
  return true;
}

bool get_gps_location(std::string *response, uint32_t timeout_ms) {
  std::string gps_response;
  const bool ok = send_at_command("AT+CGNSINF\r", &gps_response, timeout_ms);
  if (response != nullptr) {
    *response = gps_response;
  }
  if (!ok)
    ESP_LOGW(TAG, "GPS query: FAILED (%s)",
             gps_status_line(gps_response).c_str());
  return ok;
}

static bool fetch_network_location(GpsFixInfo &info, uint32_t timeout_ms) {
  const char *cmds[] = {"AT+CIPGSMLOC=1,1\r", "AT+CLBS=1,1\r", "AT+CLBS?\r"};
  for (const char *cmd : cmds) {
    std::string resp;
    if (!send_at_command(cmd, &resp, timeout_ms))
      continue;
    std::string r = trim_response(resp);
    size_t i = 0;
    std::vector<double> nums;
    while (i < r.size()) {
      while (i < r.size() &&
             !((r[i] >= '0' && r[i] <= '9') || r[i] == '+' || r[i] == '-'))
        i++;
      if (i >= r.size())
        break;
      char *endp = nullptr;
      double v = strtod(r.c_str() + i, &endp);
      if (endp != nullptr && (r.c_str() + i) != endp) {
        nums.push_back(v);
        i = static_cast<size_t>(endp - r.c_str());
      } else {
        i++;
      }
      if (nums.size() >= 3)
        break;
    }
    if (nums.size() >= 2) {
      GpsFixInfo net;
      net.latitude = nums[0];
      net.longitude = nums[1];
      net.valid = is_valid_coordinate(net.latitude, net.longitude);
      if (!net.valid)
        continue;
      if (nums.size() >= 3) {
        double acc = nums[2];
        if (acc > 0.0)
          net.hdop = acc / GPS_HDOP_TO_METERS;
      }
      info.latitude = net.latitude;
      info.longitude = net.longitude;
      if (net.hdop > 0.0)
        info.hdop = net.hdop;
      info.valid = true;
      return true;
    }
  }
  return false;
}

static double parse_gnss_timestamp_seconds(const std::string &ts) {
  if (ts.empty())
    return 0.0;
  // Expected format: YYYYMMDDhhmmss(.sss) or YYYY/MM/DD hh:mm:ss
  // Try to extract year,month,day,hour,min,sec,fraction
  int year = 0, mon = 0, day = 0, hour = 0, min = 0;
  double sec_f = 0.0;
  // compact format YYYYMMDDhhmmss.sss
  if (ts.size() >= 14 && isdigit(static_cast<unsigned char>(ts[0]))) {
    year = atoi(ts.substr(0, 4).c_str());
    mon = atoi(ts.substr(4, 2).c_str());
    day = atoi(ts.substr(6, 2).c_str());
    hour = atoi(ts.substr(8, 2).c_str());
    min = atoi(ts.substr(10, 2).c_str());
    sec_f = atof(ts.substr(12).c_str());
  } else {
    // fallback: try to parse tokens for hh:mm:ss or digits
    int scanned = sscanf(ts.c_str(), "%d-%d-%d %d:%d:%lf", &year, &mon, &day,
                         &hour, &min, &sec_f);
    if (scanned < 6)
      scanned = sscanf(ts.c_str(), "%d/%d/%d %d:%d:%lf", &year, &mon, &day,
                       &hour, &min, &sec_f);
    if (scanned < 6)
      return 0.0;
  }

  struct tm t;
  std::memset(&t, 0, sizeof(t));
  t.tm_year = year - 1900;
  t.tm_mon = mon - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = min;
  t.tm_sec = static_cast<int>(floor(sec_f));
  // Use timegm if available to get UTC seconds; fall back to mktime (localtime)
  // if not.
#if defined(__USE_MISC) || defined(_GNU_SOURCE)
  time_t s = timegm(&t);
#else
  time_t s = mktime(&t);
#endif
  if (s == (time_t)-1)
    return 0.0;
  double frac = sec_f - floor(sec_f);
  return static_cast<double>(s) + frac;
}

__attribute__((unused)) bool run_at_step(const char *label,
                                         const std::string &command,
                                         uint32_t timeout_ms,
                                         const char *success_marker) {
  std::string response;
  const bool ok =
      send_at_command(command, &response, timeout_ms, success_marker);
  ESP_LOGI(TAG, "%s: %s%s", label, ok ? "SUCCESS" : "FAILED",
           response.empty()
               ? ""
               : (std::string(" -> ") + trim_response(response)).c_str());
  return ok;
}

} // namespace

extern "C" bool gl868_modem_send_at_command(const char *command, char *response,
                                            size_t response_len,
                                            uint32_t timeout_ms) {
  if (!s_state.initialized || command == nullptr || response == nullptr ||
      response_len == 0) {
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
  const size_t copy_len =
      resp.size() < (response_len - 1) ? resp.size() : (response_len - 1);
  std::memcpy(response, resp.c_str(), copy_len);
  response[copy_len] = '\0';
  return ok;
}

extern "C" void gl868_modem_run_diagnostics(void) {
  if (!s_state.initialized) {
    ESP_LOGW(TAG, "Modem not initialized; skipping diagnostics");
    return;
  }

  static const char *diagnostic_commands[] = {
      "AT",       "ATI",          "AT+CSQ",     "AT+COPS?",
      "AT+CREG?", "AT+CGNSPWR=1", "AT+CGNSINF", "AT+CBC"};

  char response[256];
  for (size_t i = 0;
       i < sizeof(diagnostic_commands) / sizeof(diagnostic_commands[0]); ++i) {
    const bool ok = gl868_modem_send_at_command(
        diagnostic_commands[i], response, sizeof(response), 4000);
    ESP_LOGI(TAG, "AT[%zu] %s => %s", i, diagnostic_commands[i],
             ok ? response : "timeout/no-response");
  }
}

extern "C" void gl868_modem_run_full_diagnostics(void) {
  if (!s_state.initialized) {
    ESP_LOGW(TAG, "Modem not initialized; skipping full diagnostics");
    return;
  }

  char response[512];
  const auto run_command = [&](const char *command, const char *label,
                               uint32_t timeout_ms = 5000) {
    const bool ok = gl868_modem_send_at_command(command, response,
                                                sizeof(response), timeout_ms);
    const std::string value =
        ok ? trim_response(response) : std::string("timeout/no-response");
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
}

extern "C" bool gl868_modem_send_test_sms(const char *message) {
  if (!s_state.initialized) {
    ESP_LOGW(
        TAG,
        "SIM868 modem bridge is not initialized; cannot send diagnostic SMS");
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

  const std::string sms_message =
      message != nullptr ? message : "[ALERT]: SMART_SAETY_BAND_001";
  const bool sent = send_sms(number, sms_message);
  ESP_LOGI(TAG, "Diagnostic SMS to %s -> %s", number, sent ? "sent" : "failed");
  return sent;
}

extern "C" bool gl868_modem_init(void) {
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
        ESP_LOGW(TAG, "Modem AT handshake: FAILED after 3 attempts -> %s",
                 trim_response(response).c_str());
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
    ESP_LOGW(TAG, "SIM card did not become ready during init; check SIM "
                  "presence and PIN state");
    s_state.dte.reset();
    gpio_set_level(GPIO_NUM_42, 0);
    return false;
  }

  ensure_apn_configured();

  /* Wait for basic GSM network registration to complete during init so
   * that subsequent operations can assume network availability. */
  if (!wait_for_network_registration(45000)) {
    ESP_LOGW(TAG, "GSM network registration did not complete during init; poor "
                  "signal, carrier, or SIM may be preventing registration");
  }

  std::string response;
  if (!send_at_command("AT+CMEE=2\r", &response, 3000)) {
    ESP_LOGW(TAG, "Failed to enable verbose modem errors: %s",
             trim_response(response).c_str());
  }
  if (!send_at_command("AT+CIURC=0\r", &response, 3000)) {
    ESP_LOGW(TAG, "Initial readiness URCs disable: FAILED -> %s",
             trim_response(response).c_str());
  } else {
    ESP_LOGI(TAG, "Initial readiness URCs disabled (Call Ready/SMS Ready)");
  }

  s_state.gps_enabled = enable_gps();
  s_state.initialized = true;
  ESP_LOGI(TAG, "SIM868 modem bridge initialized (GPS=%d)",
           s_state.gps_enabled);

  ESP_LOGI(TAG, "Running full SIM868 diagnostics after modem initialization");
  gl868_modem_run_full_diagnostics();
  return true;
}

extern "C" void gl868_modem_update(void) {
  if (!s_state.initialized) {
    return;
  }
}

extern "C" void gl868_modem_trigger_emergency(const char *source,
                                              int32_t value) {
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
  if (!wait_for_network_registration(30000)) {
    ESP_LOGW(TAG, "GSM network registration failed after 30 seconds; emergency "
                  "alert may not be delivered");
  }

  GpsFixInfo fix_info;
  const bool gps_ok = wait_for_gps_fix(fix_info, SOS_GPS_QUALITY_TIMEOUT_MS);
  if (gps_ok) {
    ESP_LOGI(TAG,
             "SOS GNSS fix valid: sending emergency alerts with coordinates (%.6f, %.6f)",
             fix_info.latitude, fix_info.longitude);
  } else {
    ESP_LOGW(TAG,
             "No GNSS fix within %u sec; sending SOS alert with fallback "
             "coordinates (0.0, 0.0)",
             static_cast<unsigned>(SOS_GPS_QUALITY_TIMEOUT_MS / 1000U));
    fix_info.valid = false;
    fix_info.latitude = 0.0;
    fix_info.longitude = 0.0;
    gl868_modem_request_deferred_gps_upload();
  }

  // build message: include battery percent if available and Google Maps link
  // when possible
  int batt = -1;
  {
    char cbc[128] = {0};
    if (gl868_modem_send_at_command("AT+CBC", cbc, sizeof(cbc), 3000)) {
      const std::string cbcs = trim_response(std::string(cbc));
      int bcs = 0, bcl = 0, volt = 0;
      if (sscanf(cbcs.c_str(), "+CBC: %d,%d,%d", &bcs, &bcl, &volt) >= 2)
        batt = bcl;
    }
  }
  char message[320];
  if (gps_ok) {
    snprintf(message, sizeof(message),
             "ALERT: SOS activated! Loc: %.6f,%.6f. Map: "
             "[https://maps.google.com/?q=%.6f,%.6f] "
             "(https://maps.google.com/?q=%.6f,%.6f) Batt: %d%%",
             fix_info.latitude, fix_info.longitude, fix_info.latitude,
             fix_info.longitude, fix_info.latitude, fix_info.longitude,
             batt >= 0 ? batt : 0);
  } else {
    snprintf(
        message, sizeof(message),
        "ALERT: SOS activated! Loc: 0.000000,0.000000. Map: "
        "[https://maps.google.com/?q=0.000000,0.000000] "
        "(https://maps.google.com/?q=0.000000,0.000000) Batt: %d%%",
        batt >= 0 ? batt : 0);
  }

  /* STEP 1: Send Emergency SMS */
  ESP_LOGI(TAG, "[SOS STEP 1/3] Sending emergency SMS to %s", sms_number);
  const std::vector<std::string> recipients =
      split_recipients(std::string(sms_number));
  bool any_sent = false;
  for (const auto &r : recipients) {
    const bool sms_ok = send_sms(r, std::string(message));
    ESP_LOGI(TAG, "Emergency SMS to %s -> %s", r.c_str(),
             sms_ok ? "sent" : "failed");
    if (sms_ok)
      any_sent = true;
  }
  if (any_sent) {
    ESP_LOGI(
        TAG,
        "Emergency SMS succeeded; waiting before initiating emergency call");
  } else {
    ESP_LOGW(TAG, "Emergency SMS failed for all recipients; continuing with "
                  "independent call attempt");
  }
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* STEP 2: Initiate Emergency Call */
  log_modem_storage("before pre-call cleanup");
  clear_modem_user_files();
  log_modem_storage("after pre-call cleanup");

  ESP_LOGI(TAG, "[SOS STEP 2/3] Initiating emergency call to %s", call_number);
  log_call_preflight();
  const bool call_ok = make_call(call_number);
  if (!call_ok) {
    ESP_LOGW(
        TAG,
        "Emergency call not confirmed for %s; it may still have been placed",
        call_number);
    log_call_failure_details();
  } else {
    ESP_LOGI(TAG, "Emergency call initiated to %s", call_number);
    vTaskDelay(pdMS_TO_TICKS(2000));
    log_call_activity_status();
    if (!record_and_upload_answered_sos_call()) {
      ESP_LOGW(TAG, "Answered SOS call audio upload step was skipped because "
                    "modem-local recording failed; check modem filesystem "
                    "storage and available space");
    }
  }

  /* STEP 3: Post HTTP JSON Telemetry Packet */
  ESP_LOGI(TAG, "[SOS STEP 3/3] Posting SOS HTTP JSON telemetry packet over GPRS...");
  if (!post_telemetry_packet("sos", fix_info, batt)) {
    ESP_LOGW(TAG, "SOS telemetry HTTP upload failed or rate-limited");
  }
}

extern "C" bool gl868_modem_send_sms_to(const char *number,
                                        const char *message) {
  if (!s_state.initialized || number == nullptr || message == nullptr)
    return false;
  return send_sms(std::string(number), std::string(message));
}

extern "C" bool gl868_modem_make_call_to(const char *number) {
  if (!s_state.initialized || number == nullptr)
    return false;
  return make_call(std::string(number));
}

extern "C" bool gl868_modem_hang_up_call(void) {
  if (!s_state.initialized)
    return false;
  std::string response;
  const bool ok = send_at_command("ATH\r", &response, 5000);
  ESP_LOGI(TAG, "Call hang-up: %s (%s)", ok ? "success" : "failed",
           trim_response(response).c_str());
  return ok;
}

extern "C" bool gl868_modem_upload_telemetry(const char *event_type) {
  if (!s_state.initialized)
    return false;

  double latitude = 0.0;
  double longitude = 0.0;
  bool has_gps_fix = gl868_modem_get_gps_coordinates(&latitude, &longitude);

  if (has_gps_fix) {
    ESP_LOGI(TAG,
             "GNSS fix valid: uploading HTTP telemetry with coordinates (%.6f, "
             "%.6f)",
             latitude, longitude);
  } else {
    ESP_LOGW(
        TAG,
        "GNSS fix pending/unavailable; sending HTTP telemetry with fallback "
        "coordinates (0.0, 0.0)");
    latitude = 0.0;
    longitude = 0.0;
  }

  /* Query for optional telemetry metadata; retain corrected/fallback
   * coordinates. */
  GpsFixInfo fix;
  std::string gps_response;
  if (!get_gps_location(&gps_response, 5000) ||
      !parse_gps_fix_info_from_cgnsinf(gps_response, fix)) {
    fix = GpsFixInfo{};
  }
  fix.valid = has_gps_fix;
  fix.latitude = latitude;
  fix.longitude = longitude;
  return post_telemetry_packet(event_type, fix,
                               gl868_modem_get_battery_percent());
}

extern "C" bool gl868_modem_send_live_location(void) {
  if (!s_state.initialized)
    return false;

  double latitude = 0.0;
  double longitude = 0.0;
  if (!gl868_modem_get_gps_coordinates(&latitude, &longitude)) {
    ESP_LOGW(TAG, "Live tracking: no valid GPS fix; SMS not sent");
    return false;
  }

  char message[96];
  snprintf(message, sizeof(message), "https://maps.google.com/?q=%.6f,%.6f",
           latitude, longitude);
  const char *number = get_emergency_sms_number();
  const bool sent = send_sms(number, std::string(message));
  ESP_LOGI(TAG, "Live tracking SMS to %s -> %s: %s", number,
           sent ? "sent" : "failed", message);
  return sent;
}

extern "C" bool gl868_modem_get_gps_now(char *buf, size_t buf_len) {
  if (!s_state.initialized || buf == nullptr || buf_len == 0)
    return false;
  std::string gps;
  const bool ok = get_gps_location(&gps);
  const size_t copy_len =
      gps.size() < (buf_len - 1) ? gps.size() : (buf_len - 1);
  if (copy_len > 0)
    memcpy(buf, gps.c_str(), copy_len);
  buf[copy_len] = '\0';
  return ok;
}

extern "C" bool gl868_modem_get_gps_coordinates(double *latitude,
                                                double *longitude) {
  if (!s_state.initialized || latitude == nullptr || longitude == nullptr)
    return false;

  std::string gps_response;
  GpsFixInfo fix;
  if (!get_gps_location(&gps_response, 10000)) {
    ESP_LOGW(TAG, "GPS location query failed: %s",
             gps_status_line(gps_response).c_str());
    return false;
  }
  if (!parse_gps_fix_info_from_cgnsinf(gps_response, fix)) {
    ESP_LOGW(TAG, "GPS response parse failed: %s",
             gps_status_line(gps_response).c_str());
    return false;
  }
  if (fix.fix_status <= 0) {
    ESP_LOGW(TAG, "GPS fix unavailable: run=%d fix_status=%d timestamp=%s",
             fix.run_status, fix.fix_status, fix.timestamp.c_str());
    return false;
  }

  bool poor_metadata = (fix.hdop <= 0.0 || fix.hdop > GPS_ACCEPTABLE_HDOP ||
                        fix.satellites_used < GPS_MIN_SATELLITES);

  if (poor_metadata && GPS_ENABLE_SMOOTHING_FALLBACK) {
    // use global Kalman state so tuning persists across calls
    KalmanCV2D &kf = s_kf;

    // 1) Try network-based coarse location and accept if its estimated accuracy
    // is reasonable.
    GpsFixInfo netinfo;
    if (fetch_network_location(netinfo, 3000)) {
      double net_acc_m = (netinfo.hdop > 0.0)
                             ? hdop_to_estimated_meters(netinfo.hdop)
                             : NETWORK_FALLBACK_MAX_ACCURACY_METERS;
      if (net_acc_m <= NETWORK_FALLBACK_MAX_ACCURACY_METERS) {
        double meas_std_deg = net_acc_m / 111320.0;
        double meas_var = meas_std_deg * meas_std_deg;
        // compute variable dt from last GNSS fix timestamp when available
        double dt = 0.5;
        double curt = parse_gnss_timestamp_seconds(fix.timestamp);
        if (curt > 0.0 && s_last_fix_time > 0.0) {
          double d = curt - s_last_fix_time;
          if (d > 0.0 && d < 60.0)
            dt = d;
        }
        if (curt > 0.0)
          s_last_fix_time = curt;
        kf.predict(dt);
        kf.update(netinfo.latitude, netinfo.longitude, meas_var);
        *latitude = kf.x[0];
        *longitude = kf.x[1];
        ESP_LOGW(TAG,
                 "Using network fallback location (est acc=%.1fm): %.6f,%.6f",
                 net_acc_m, *latitude, *longitude);
        return true;
      }
    }

    // 2) Collect multiple GNSS samples and compute a robust estimate (weighted
    // average by 1/hdop^2)
    const int sample_count = 6;
    std::vector<GpsFixInfo> samples;
    for (int i = 0; i < sample_count; ++i) {
      std::string resp;
      if (!get_gps_location(&resp, 3000)) {
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      GpsFixInfo s;
      if (!parse_gps_fix_info_from_cgnsinf(resp, s)) {
        vTaskDelay(pdMS_TO_TICKS(300));
        continue;
      }
      if (s.fix_status <= 0 || !is_valid_coordinate(s.latitude, s.longitude))
        continue;
      samples.push_back(s);
      vTaskDelay(pdMS_TO_TICKS(400));
    }

    if (!samples.empty()) {
      double wsum = 0.0, lat_sum = 0.0, lon_sum = 0.0;
      double hdop_sum = 0.0;
      for (const auto &si : samples) {
        double w = 1.0;
        if (si.hdop > 0.0)
          w = 1.0 / (si.hdop * si.hdop);
        lat_sum += si.latitude * w;
        lon_sum += si.longitude * w;
        wsum += w;
        hdop_sum += (si.hdop > 0.0) ? si.hdop : GPS_ACCEPTABLE_HDOP;
      }
      double avg_lat = lat_sum / wsum;
      double avg_lon = lon_sum / wsum;
      double avg_hdop = hdop_sum / static_cast<double>(samples.size());

      double meas_m = hdop_to_estimated_meters(avg_hdop);
      double meas_std_deg = meas_m / 111320.0;
      double meas_var = meas_std_deg * meas_std_deg;
      // compute dt from last sample timestamp when available
      double dt = 0.5;
      if (!samples.empty()) {
        double curt = parse_gnss_timestamp_seconds(samples.back().timestamp);
        if (curt > 0.0 && s_last_fix_time > 0.0) {
          double d = curt - s_last_fix_time;
          if (d > 0.0 && d < 60.0)
            dt = d;
        }
        if (curt > 0.0)
          s_last_fix_time = curt;
      }
      kf.predict(dt);
      kf.update(avg_lat, avg_lon, meas_var);
      *latitude = kf.x[0];
      *longitude = kf.x[1];
      ESP_LOGW(TAG,
               "GPS poor metadata: returning multi-sample Kalman estimate "
               "(samples=%d avg_hdop=%.2f est_m=%.1fm)",
               (int)samples.size(), avg_hdop, meas_m);
      ESP_LOGI(
          TAG,
          "GPS fix: SUCCESS -> %.6f,%.6f (smoothed, est_m=%.1fm, samples=%d)",
          *latitude, *longitude, meas_m, (int)samples.size());
      return true;
    }

    // 3) No good alternate samples — fall back to using the single measurement
    // in Kalman
    double meas_m = hdop_to_estimated_meters(fix.hdop);
    if (meas_m > 1e5)
      meas_m = 200.0;
    double meas_std_deg = meas_m / 111320.0;
    double meas_var = meas_std_deg * meas_std_deg;
    double dt = 0.5;
    double curt = parse_gnss_timestamp_seconds(fix.timestamp);
    if (curt > 0.0 && s_last_fix_time > 0.0) {
      double d = curt - s_last_fix_time;
      if (d > 0.0 && d < 60.0)
        dt = d;
    }
    if (curt > 0.0)
      s_last_fix_time = curt;
    kf.predict(dt);
    kf.update(fix.latitude, fix.longitude, meas_var);
    *latitude = kf.x[0];
    *longitude = kf.x[1];
    ESP_LOGW(TAG,
             "GPS metadata poor (hdop=%.2f sats=%d); returning Kalman-smoothed "
             "coord (meas_m=%.1fm)",
             fix.hdop, fix.satellites_used, meas_m);
    ESP_LOGI(TAG,
             "GPS fix: SUCCESS -> %.6f,%.6f (smoothed, hdop=%.2f, sats=%d)",
             *latitude, *longitude, fix.hdop, fix.satellites_used);
    return true;
  }

  *latitude = fix.latitude;
  *longitude = fix.longitude;
  ESP_LOGI(TAG, "GPS fix: SUCCESS -> %.6f,%.6f (hdop=%.2f, sats=%d)",
           fix.latitude, fix.longitude, fix.hdop, fix.satellites_used);
  return true;
}

extern "C" void gl868_modem_set_movement_profile(int profile) {
  set_kalman_profile(profile);
  ESP_LOGI(TAG, "Movement profile set: %d", profile);
}

extern "C" const char *gl868_modem_get_emergency_call_number(void) {
  return get_emergency_call_number();
}

extern "C" const char *gl868_modem_get_emergency_sms_number(void) {
  return get_emergency_sms_number();
}

extern "C" int gl868_modem_get_battery_percent(void) {
  if (!s_state.initialized)
    return -1;
  char buf[128] = {0};
  if (!gl868_modem_send_at_command("AT+CBC", buf, sizeof(buf), 3000))
    return -1;
  const std::string resp(buf);
  const std::string trimmed = trim_response(resp);
  // Expected: +CBC: <bcs>,<bcl>,<voltage>
  int bcs = 0, bcl = 0, volt = 0;
  if (sscanf(trimmed.c_str(), "+CBC: %d,%d,%d", &bcs, &bcl, &volt) >= 2) {
    return bcl; // battery percent
  }
  return -1;
}
