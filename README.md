# Smart Safety Band Firmware

ESP-IDF / FreeRTOS firmware for an ESP32-S3 Mini-1 based GeoLinker GL868 safety band. It provides a safety-event pipeline, manual SOS handling, fall detection with LSM6DSOX (and legacy LIS3DH support), and a dedicated SIM868 communication bridge for SMS and voice emergency dispatch.

The current firmware is focused on local emergency communication. Cloud reporting and remote telemetry are not included in this branch; the modem is powered on only when needed and otherwise remains off to save battery.

Optional I2C devices are probed without preventing boot: BME680/BME688, SCD40/SCD41, VEML6075, AS3935, MAX86141/MAX86176, MAX30208, and MAX30009. Their measurement/compensation drivers remain hardware-specific integration points; development-only simulated values are always labelled `SIM`.

## Real-time development without modules

The FreeRTOS runtime includes a 20 ms motion/safety loop, event-driven communication tasks, and periodic health/environment monitoring. With `SAFETY_BAND_SIMULATION` enabled, missing optional sensor modules emit explicitly labelled `SIM` telemetry. This lets you develop task timing, queues, logging, and emergency logic before hardware arrives. It is disabled by default, so normal firmware never presents simulated values as live measurements.

## Reference wiring

| Function | ESP32-S3 GPIO | Notes |
|---|---:|---|
| I2C SDA / SCL | 8 / 9 | Shared sensor bus |
| Status LED | 47 | Emergency flashes rapidly |
| SIM868 UART TX / RX | 17 / 18 | 115200 baud |
| SIM868 power | 42 | Board power-control line |
| SOS button | 41 / 4| Active-low; configurable in `menuconfig` |
| Motion interrupt | 2 by default | Reserved for sensor-driver integration |

## Build

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
```

## GitHub repository setup

This project is ready to be published as a GitHub repository for team collaboration.
The included helper script can create the remote repo, configure the `origin` remote, push the `main` branch, and apply repository metadata.

```bash
./create_github_repo.sh
```

Before running the script, authenticate `gh` locally:

```bash
gh auth login
```

If you want to use a personal access token instead, set it securely and authenticate with:

```bash
gh auth login --with-token < /path/to/token
```

## NVS configuration

The modem and emergency route settings are stored in NVS under the `safety` namespace.

- `emergency_numbers`: comma- or semicolon-separated phone numbers used for voice calls, with the first number dialed first.
- `emergency_number`: legacy fallback single phone number when `emergency_numbers` is not present.
- `emergency_sms_numbers`: comma- or semicolon-separated phone numbers used for emergency SMS delivery.
- `emergency_sms_number`: legacy fallback single SMS number when `emergency_sms_numbers` is not present.
- `cellular_apn`: operator APN used during modem attach; defaults to `internet`.
- `dispatch_channels`: bitmask defining emergency dispatch routes: SMS `1`, voice `2`, or both `3`.

### Writing NVS values

You can write these values from application code using ESP-IDF's NVS APIs or with a Python helper.

Example C helper using `nvs_flash`:

```c
#include "nvs.h"
#include "nvs_flash.h"

void write_safety_nvs(void) {
    nvs_handle_t handle;
    if (nvs_open("safety", NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_str(handle, "emergency_numbers", "+911234567890,+919876543210");
    nvs_set_str(handle, "emergency_sms_numbers", "+911234567890");
    nvs_set_str(handle, "cellular_apn", "internet");
    nvs_set_u8(handle, "dispatch_channels", 3);
    nvs_commit(handle);
    nvs_close(handle);
}
```

Example IDF Python command using `idf.py` and a simple script:

```bash
cat <<'EOF' > write_nvs.py
from espidf import nvs

with nvs.open('safety', 'rw') as handle:
    handle.set_str('emergency_numbers', '+911234567890,+919876543210')
    handle.set_str('emergency_sms_numbers', '+911234567890')
    handle.set_str('cellular_apn', 'internet')
    handle.set_u8('dispatch_channels', 3)
EOF

idf.py -p PORT python write_nvs.py
```

## Runtime behavior

- The SIM868 modem is kept powered off until `gl868_modem_init()` is called.
- `gl868_modem_shutdown()` powers the modem down gracefully when the modem bridge is no longer needed.
- During an emergency, `emergency_dispatch_task` sets `BIT_EMERGENCY`, wakes GNSS, and dispatches SMS and voice events.
- The `live_location_task` runs while emergency mode is active, queries GNSS at ~1 Hz, applies a simple Kalman filter, and enqueues live-location SMS fallback every 60 seconds.
- SMS and voice recipients are read dynamically from NVS at runtime, so contact lists can be updated without recompiling.

## Emergency dispatch

- SMS payloads include an event description, UTC timestamp, GPS location or fallback text, and battery state.
- Voice calls are dialed from `emergency_numbers` sequentially with a retry delay between attempts.
- If NVS contact lists are missing, the firmware falls back to built-in default numbers.

## Modem and diagnostics

The SIM868 bridge is implemented in `main/gl868_modem.cpp` and exposes C APIs via `main/gl868_modem.h`:

- `gl868_modem_init()` / `gl868_modem_shutdown()`
- `gl868_modem_update()` for periodic state progression
- `gl868_modem_trigger_emergency()` for emergency dispatch
- `gl868_modem_send_at_command()` and diagnostics helpers
- `gl868_modem_send_test_sms()`
- `gl868_modem_send_sms_to()`
- `gl868_modem_make_call_to()` / `gl868_modem_hangup_call()`
- `gl868_modem_enable_gnss()` / `gl868_modem_disable_gnss()`
- `gl868_modem_get_gps_now()`
- `gl868_modem_get_emergency_call_number()` / `gl868_modem_get_emergency_sms_number()`
- `gl868_modem_get_battery_percent()`

## Notes for modification

- Change the modem power GPIO or UART pins in `main/gl868_modem.cpp` if your hardware uses different wiring.
- Update the APN by writing `cellular_apn` into the `safety` NVS namespace.
- Update call and SMS recipients by changing `emergency_numbers` / `emergency_sms_numbers` in NVS.
- Use `dispatch_channels` to enable only SMS, only voice, or both.

## Current implementation status

This firmware branch includes:
- dynamic emergency contact configuration via NVS
- on-demand modem initialization and shutdown for battery savings
- SMS and voice emergency alert routing
- live-location SMS fallback during active emergency
- direct SIM868 control over UART and power GPIO

This branch does not include remote cloud telemetry, MQTT reporting, or OTA update support.
