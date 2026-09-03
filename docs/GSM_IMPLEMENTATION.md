# GSM Telephony & Voice: Application Usage, Working & Implementation Procedure

This document details the practical application usage, step-by-step operational flow, C++ source code implementation procedure, and testing verification for GSM telephony, emergency voice calls, and SMS dispatches on the ESP32-S3 Mini-1 and SIM868 platform.

---

## 1. Application Usage & Real-World Use Cases

The GSM Telephony subsystem provides crucial emergency voice and SMS communication channels:

1. **Emergency SOS Alert SMS Dispatch**:
   * **Trigger**: Physical hardware button press or automatic fall detection trigger.
   * **Usage**: Transmits an emergency SMS text containing Google Maps coordinates and battery percentage to predefined emergency numbers (`CONFIG_SAFETY_BAND_EMERGENCY_SMS_NUMBER`).
2. **Emergency Voice Call Dispatch**:
   * **Trigger**: Post-SMS dispatch during active SOS state.
   * **Usage**: Automatically dials the emergency recipient's phone number (`ATD`), establishes two-way voice communication, and records 60 seconds of ambient audio for command-center verification.

---

## 2. Step-by-Step Working Mechanism

```mermaid
sequenceDiagram
    participant User as User Press (GPIO 4)
    participant Task as communication_task (ESP32-S3)
    participant Modem as SIM868 UART
    participant Phone as Emergency Recipient Phone

    User->>Task: SOS Button Trigger Interrupt
    Task->>Modem: AT+CMGF=1\r (Set SMS Text Mode)
    Modem-->>Task: OK
    Task->>Modem: AT+CMGS="+91XXXXXXXXXX"\r
    Modem-->>Task: > (Prompt)
    Task->>Modem: ALERT: SOS activated! Loc: lat,lon. Map: https://maps.google.com/?q=lat,lon <Ctrl+Z>
    Modem-->>Phone: Deliver Emergency SMS
    Modem-->>Task: +CMGS: <mr> OK
    
    Task->>Modem: ATD+91XXXXXXXXXX;\r (Initiate Call)
    Modem-->>Phone: Ring Recipient
    Phone-->>Modem: Call Answered
    Modem-->>Task: VOICE CALL: BEGIN
    Task->>Modem: Start 60s Local Audio Recording
    Task->>Modem: ATH\r (Hangup Call after 60s)
    Modem-->>Phone: Call Ended
```

---

## 3. Firmware Implementation Procedure

### 3.1 Serialized Thread Safety (`communication_task`)

In `main/safety_band_main.c`, dedicated tasks communicate with the modem through `communication_task` via a FreeRTOS Queue to prevent UART collision:

```c
void app_main(void) {
    /* Create FreeRTOS Queue for modem requests */
    s_modem_queue = xQueueCreate(10, sizeof(modem_request_t));

    /* Start dedicated communication task on Core 0 */
    xTaskCreatePinnedToCore(communication_task, "communication_task", 4096, NULL, 5, NULL, 0);
    
    /* Start SOS button interrupt handler */
    xTaskCreate(sos_button_task, "sos_button_task", 3072, NULL, 10, NULL);
}
```

### 3.2 Key C++ Source Implementation Code

#### Step 1: Emergency SMS Dispatch (`send_sms`)
```cpp
static bool send_sms(const std::string &number, const std::string &message) {
  std::string response;

  /* 1. Set SMS text mode */
  if (!send_at_command("AT+CMGF=1\r", &response, 5000)) return false;

  /* 2. Initiate CMGS command with phone number */
  std::string cmgs_cmd = "AT+CMGS=\"" + number + "\"\r";
  if (!wait_for_prompt(cmgs_cmd, ">", &response, 10000)) return false;

  /* 3. Send message payload followed by Ctrl+Z (0x1A) */
  std::string full_payload = message + "\x1A";
  send_raw_bytes(full_payload.data(), full_payload.size(), 15000);

  /* 4. Verify transmission response */
  bool ok = wait_for_response("+CMGS:", &response, 20000);
  ESP_LOGI(TAG, "SMS dispatch to %s: %s", number.c_str(), ok ? "SUCCESS" : "FAILED");
  return ok;
}
```

#### Step 2: Emergency Voice Calling (`make_call`)
```cpp
static bool make_call(const std::string &number) {
  std::string response;
  
  /* 1. Format ATD dial command */
  std::string dial_cmd = "ATD" + number + ";\r";
  ESP_LOGI(TAG, "Dialing emergency number: %s", number.c_str());

  /* 2. Issue dial command and monitor connection URCs */
  if (!send_at_command(dial_cmd, &response, 10000)) return false;

  /* 3. Wait for call establishment or termination */
  vTaskDelay(pdMS_TO_TICKS(5000));
  
  /* 4. Clean hangup using ATH */
  send_at_command("ATH\r", &response, 5000);
  return true;
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
3. Observe serial logs for SMS dispatch and incoming voice call on recipient phone.

### 4.3 Expected Log Output
```text
I (31000) SMART_SAFETY_BAND_001: SOS button pressed! Initiating emergency alerts.
I (31050) sim868_bridge: SMS dispatch to +916309538622: SUCCESS
I (35000) sim868_bridge: Dialing emergency number: +916309538622
I (35100) sim868_bridge: ATD+916309538622; -> OK
I (95100) sim868_bridge: Call hang-up: success (OK)
```
