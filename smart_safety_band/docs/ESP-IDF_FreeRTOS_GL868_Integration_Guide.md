# Native ESP-IDF SIM868 integration guide
### ESP32-S3 + SIM868 — native modem bridge and safety firmware

---

## 0. Current project architecture

This repository no longer uses the older Arduino `GL868_ESP32` wrapper integration.

The current firmware is built as a native ESP-IDF project using:
- `main/safety_band_main.c` for safety and communication task logic
- `main/gl868_modem.cpp` for the SIM868 UART modem bridge
- `main/gl868_modem.h` for modem API exports
- `main/idf_component.yml` to depend on `espressif/esp_modem`

That means the effective integration is:
- not using Arduino `HardwareSerial`, `Wire`, or `Preferences`
- not depending on ArduinoJson
- not calling `GeoLinker` or `GL868_ESP32` APIs
- using direct AT command handling through ESP-IDF `esp_modem`

---

## 1. Repository structure

```
smart_safety_band/
├── CMakeLists.txt
├── sdkconfig
├── docs/
│   └── ESP-IDF_FreeRTOS_GL868_Integration_Guide.md
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml
    ├── safety_band_main.c
    ├── gl868_modem.cpp
    └── gl868_modem.h
```

The current integration is centered on a native modem bridge and FreeRTOS tasks.

---

## 2. Build and dependency files

### `main/idf_component.yml`

```yml
dependencies:
  espressif/esp_modem: ^1.4.3
```

### `main/CMakeLists.txt`

```cmake
idf_component_register(SRCS "safety_band_main.c" "gl868_modem.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_uart esp_modem)
```

This is the correct native build path for the current codebase.

---

## 3. Native SIM868 bridge behavior

`main/gl868_modem.cpp` now implements the modem directly in ESP-IDF.

It does the following:
- controls SIM868 power with GPIO42
- initializes UART1 on TX=17 and RX=18
- sends AT commands through `esp_modem::DTE`
- retries modem initialization when `AT` does not respond
- power-cycles the modem between retries
- checks SIM status with `AT+CPIN?`
- skips SIM-only AT checks until the SIM reports `READY`
- only sends diagnostic SMS when the SIM is ready
- triggers emergency SMS and call alerts after initialization

This is more robust than attempting every SIM command on boot unconditionally.

---

## 4. Safety and communication task architecture

The repo currently runs two main task flows:

- `communication_task` in `main/safety_band_main.c`
  - initializes the SIM868 modem bridge
  - runs modem diagnostics
  - sends a diagnostic SMS if the SIM is ready
  - keeps the modem bridge alive

- `safety_manager_task`
  - receives safety events from the app
  - triggers emergency SMS/call through the modem bridge

The forced emergency path is handled by `gl868_modem_trigger_emergency()`.

---

## 5. Why the reported error happened

Your earlier log showed a pattern where the modem worked after a reset but failed on a subsequent flash with:
- `SIM868 did not respond to AT:`
- `SIM busy`
- `SIM not inserted`
- `operation not allowed`
- `SIM PIN required`

That usually means the SIM868 was still in a partially initialized or busy state after the previous boot.

The updated bridge now:
- performs a modem power cycle on initialization
- retries `AT` up to three times
- recreates the UART DTE after each power cycle
- checks `AT+CPIN?` before running SIM-specific diagnostics

This reduces the chance of repeated false negatives caused by modem firmware state or a non-ready SIM.

---

## 6. What diagnostics now do

`gl868_modem_run_full_diagnostics()` now:
- always runs modem-level checks like `AT`, `ATI`, `AT+COPS?`, `AT+CSQ`, `AT+CREG?`, and GPS commands
- reads SIM status first with `AT+CPIN?`
- only executes SIM-dependent commands like `AT+CIMI`, `AT+CCID`, `AT+CNUM?`, and `AT+CGATT?` when the SIM is actually READY

That prevents noisy logs like `+CME ERROR: operation not allowed` when the SIM is still locked, busy, or missing.

---

## 7. What the firmware now expects

The current code expects one of these paths:

- healthy SIM state:
  - `SIM868 modem bridge initialized (GPS=0 or 1)`
  - `SIM868 modem bridge ready`
  - `Running full SIM868 diagnostics...`
  - `[SIM status] READY`
  - diagnostic SMS proceeds

- SIM not ready:
  - modem initializes successfully
  - diagnostic logs show the SIM state
  - SIM-dependent checks are skipped until the SIM becomes READY
  - no SMS is attempted until the SIM is ready

This makes the firmware resilient to a modem that is powered but not yet fully registered.

---

## 8. Flash and monitor

```bash
cd /home/eyashwanth/smart_safety_band
source /home/eyashwanth/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

If you still see repeated `SIM busy` or `SIM not inserted` messages, verify the hardware and SIM card:
- SIM is seated correctly
- SIM socket contacts are clean
- modem power is being cycled properly
- no other serial tool is holding the UART port

---

## 9. Notes on the old Arduino guide content

The previous file content described an Arduino-based integration that is not what this repo currently uses.

The older guide is outdated for this project because the repo now implements a native ESP-IDF modem bridge rather than the Arduino `GL868_ESP32` component.

---

## 10. Recommended next step

Use the current `main/gl868_modem.cpp` implementation and `idf.py flash monitor` to verify the modem starts cleanly.

If the SIM still reports `BUSY` or `NOT INSERTED`, the likely issue is physical SIM state rather than the code itself.
