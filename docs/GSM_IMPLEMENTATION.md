# GSM Telephony & Voice: Application Usage, Working & Implementation Procedure

This document details the practical application usage, step-by-step operational flow, C++ source code implementation procedure, and testing verification for GSM telephony, emergency voice calls, SMS dispatches, and HTTP JSON telemetry sequencing on the ESP32-S3 Mini-1 and SIM868 platform.

---

## 1. Application Usage & Real-World Use Cases

The GSM Telephony subsystem provides crucial emergency voice and SMS communication channels:

1. **Emergency SOS Alert SMS Dispatch (`[SOS STEP 1/3]`)**:
   * **Trigger**: Physical hardware button press (GPIO 4) or automatic fall detection trigger.
   * **Usage**: Transmits an emergency SMS text containing Google Maps coordinates and battery percentage to predefined emergency numbers (`CONFIG_SAFETY_BAND_EMERGENCY_SMS_NUMBER`).
2. **Emergency Voice Call Dispatch (`[SOS STEP 2/3]`)**:
   * **Trigger**: Post-SMS dispatch during active SOS state.
   * **Usage**: Automatically dials the emergency recipient's phone number (`ATD`), establishes two-way voice communication, and records 60 seconds of ambient audio for command-center verification.
3. **HTTP JSON Telemetry Delivery (`[SOS STEP 3/3]`)**:
   * **Trigger**: Post-Call completion.
   * **Usage**: Publishes a normalized `smart-city-event/1.0` JSON payload via GPRS HTTP POST to the backend command center.

---

## 2. Step-by-Step Working Mechanism

```mermaid
sequenceDiagram
    participant User as User Press (GPIO 4)
    participant Task as communication_task (ESP32-S3)
    participant Modem as SIM868 UART
    participant Phone as Emergency Recipient Phone
    participant Server as HTTP Backend Server

    User->>Task: SOS Button Trigger Interrupt
    
    Note over Task, Phone: STEP 1: Emergency SMS Dispatch
    Task->>Modem: AT+CMGF=1\r (Set SMS Text Mode)
    Modem-->>Task: OK
    Task->>Modem: AT+CMGS="+91XXXXXXXXXX"\r
    Modem-->>Task: > (Prompt)
    Task->>Modem: ALERT: SOS activated! Loc: lat,lon. Map: https://maps.google.com/?q=lat,lon <Ctrl+Z>
    Modem-->>Phone: Deliver Emergency SMS
    Modem-->>Task: +CMGS: <mr> OK
    
    Note over Task, Phone: STEP 2: Emergency Voice Call & Recording
    Task->>Modem: ATD+91XXXXXXXXXX;\r (Initiate Call)
    Modem-->>Phone: Ring Recipient
    Phone-->>Modem: Call Answered
    Modem-->>Task: VOICE CALL: BEGIN
    Task->>Modem: Start 60s Local Audio Recording
    Task->>Modem: ATH\r (Hangup Call after 60s)
    Modem-->>Phone: Call Ended
    
    Note over Task, Server: STEP 3: HTTP JSON Telemetry Packet
    Task->>Modem: AT+HTTPINIT & AT+HTTPACTION=1
    Modem-->>Server: HTTP POST /api/v1/events/normalized (JSON Payload)
    Server-->>Modem: HTTP 200 OK
```

---

## 3. Firmware Implementation Procedure

### 3.1 Serialized Thread Safety (`communication_task`)

In `main/safety_band_main.c`, dedicated tasks communicate with the modem through `communication_task` via a FreeRTOS Queue (`s_communication_events`) to prevent UART collisions:

```c
void app_main(void) {
    /* Create FreeRTOS Queue for modem communication events */
    s_communication_events = xQueueCreate(COMMUNICATION_QUEUE_DEPTH, sizeof(communication_event_t));

    /* Start dedicated communication task on Core 0 */
    xTaskCreatePinnedToCore(communication_task, "communication_task", 6144, NULL, 5, NULL, 0);
    
    /* Start SOS button interrupt handler */
    xTaskCreate(sos_button_task, "sos_button_task", 3072, NULL, 10, NULL);
}
```

### 3.2 Sequential SOS Trigger Implementation (`gl868_modem.cpp`)

```cpp
extern "C" void gl868_modem_trigger_sos(void) {
  if (!s_state.initialized) return;

  const char *sms_number = get_emergency_sms_number();
  const char *call_number = get_emergency_call_number();

  GpsFixInfo fix_info;
  const bool gps_ok = wait_for_gps_fix(fix_info, SOS_GPS_QUALITY_TIMEOUT_MS);
  if (gps_ok) {
    ESP_LOGI(TAG, "SOS GNSS fix valid: coordinates (%.6f, %.6f)", fix_info.latitude, fix_info.longitude);
  } else {
    ESP_LOGW(TAG, "No GNSS fix within timeout; using fallback coordinates (0.0, 0.0)");
    fix_info.valid = false;
    fix_info.latitude = 0.0;
    fix_info.longitude = 0.0;
  }

  int batt = gl868_modem_get_battery_percent();
  char message[320];
  snprintf(message, sizeof(message),
           "ALERT: SOS activated! Loc: %.6f,%.6f. Map: [https://maps.google.com/?q=%.6f,%.6f] Batt: %d%%",
           fix_info.latitude, fix_info.longitude, fix_info.latitude, fix_info.longitude, batt);

  /* STEP 1: Send Emergency SMS */
  ESP_LOGI(TAG, "[SOS STEP 1/3] Sending emergency SMS to %s", sms_number);
  const std::vector<std::string> recipients = split_recipients(std::string(sms_number));
  for (const auto &r : recipients) {
    send_sms(r, std::string(message));
  }
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* STEP 2: Initiate Emergency Call & AMR Recording */
  clear_modem_user_files();
  ESP_LOGI(TAG, "[SOS STEP 2/3] Initiating emergency call to %s", call_number);
  if (make_call(call_number)) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    record_and_upload_answered_sos_call();
  }

  /* STEP 3: Post HTTP JSON Telemetry Packet */
  ESP_LOGI(TAG, "[SOS STEP 3/3] Posting SOS HTTP JSON telemetry packet over GPRS...");
  post_telemetry_packet("sos", fix_info, batt);
}
```

---

## 4. Implementation Testing & Verification

### 4.1 Prerequisites
* Valid SIM card with SMS and voice call balance inserted into SIM868 holder.
* Emergency phone number configured via `idf.py menuconfig` -> **Smart Safety Band Configuration → Emergency SMS Number**.

### 4.2 Manual Test Procedure
1. Flash firmware and open monitor: `idf.py flash monitor`.
2. Press the SOS hardware button (GPIO 4).
3. Observe serial logs for the 3 sequential execution steps (`[SOS STEP 1/3]`, `[SOS STEP 2/3]`, and `[SOS STEP 3/3]`).

### 4.3 Expected Log Output
```text
I (31000) SMART_SAFETY_BAND_001: SOS button pressed! Initiating emergency alerts.
I (31050) sim868_bridge: [SOS STEP 1/3] Sending emergency SMS to +916309538622
I (32150) sim868_bridge: Emergency SMS to +916309538622 -> sent
I (34150) sim868_bridge: [SOS STEP 2/3] Initiating emergency call to +916309538622
I (34200) sim868_bridge: ATD+916309538622; -> OK
I (94200) sim868_bridge: Call hang-up: success (OK)
I (96200) sim868_bridge: [SOS STEP 3/3] Posting SOS HTTP JSON telemetry packet over GPRS...
I (99500) sim868_bridge: HTTP POST SUCCESS [HTTP status 200]! Server ACK Received.
```
