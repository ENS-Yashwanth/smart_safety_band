# Smart Safety Band firmware

ESP-IDF / FreeRTOS firmware for an ESP32-S3 Mini-1 based GeoLinker GL868 safety band. It provides modem initialization, manual SOS SMS/call alerts, GPS acquisition, and a single-owner SIM868 communication task so no two tasks issue AT commands on the UART at the same time.

Optional I2C devices are probed without preventing boot: BME680/BME688, SCD40/SCD41, VEML6075, AS3935, MAX86141/MAX86176, MAX30208, and MAX30009. Their measurement/compensation drivers remain hardware-specific integration points; development-only simulated values are always marked `SIM`.

## Runtime tasks

All application task entry points are in `main/safety_band_main.c`:

- `communication_task` powers on and initializes the modem, then serializes emergency calls/SMS, GPS reads, GPRS, and HTTP.
- `gps_task` queues an HTTPS live-location upload every two minutes.
- `sos_button_task` debounces the active-low SOS button and queues one emergency request per press.

The SMS alert format is `ALERT: SOS activated! Loc: <lat>,<lon>. Map: [<url>] (<url>) Batt: <percent>%`.

## HTTP/HTTPS telemetry and SOS audio

The band attaches GPRS using the configured `iot.com` APN, opens SIM868 bearer profile 1, and POSTs JSON to the configured endpoint. The temporary default is `http://my-esp32-test.free.beeceptor.com`; use HTTPS for deployment. Live location uploads run every two minutes; an SOS additionally posts its location before SMS/call handling. Configure a server endpoint and device ID in **Smart Safety Band configuration**.

After an SOS call is answered, the SIM868 records 60 seconds of AMR audio, ends the call so its class-B GPRS data service can operate, copies the recording to RAM, deletes the temporary modem file, then uploads Base64 JSON chunks over HTTPS. The server reconstructs chunks using `file_name`, such as `smart_safety_band_001*20260901_123456.amr`, plus `audio_chunk_index` and `audio_chunk_count`.

Example payload:

```json
{"device_id":"smart_safety_band_001","event":"sos","timestamp":"20260831_143015","latitude":12.971599,"longitude":77.594566,"altitude_m":920.3,"speed_kph":4.5,"course_deg":180.0,"hdop":0.8,"pdop":1.1,"vdop":0.9,"satellites_in_view":12,"satellites_used":8,"battery_percent":85}
```

The firmware sets `AT+HTTPSSL=0` for the temporary HTTP endpoint and enables `AT+HTTPSSL=1` automatically for an HTTPS URL. Verify the deployed endpoint during field testing.

## GeoLinker setup

In `idf.py menuconfig`, set **Smart Safety Band configuration → GeoLinker API key** and choose a device ID. The default GPRS APN is `iot.com`; change it to the APN supplied by your SIM provider if needed. GPS coordinates are POSTed to CircuitDigest's [GeoLinker API](https://www.circuitdigest.cloud/geolinker/overview) every two minutes, where they appear under the configured device ID.

## Reference wiring

| Function | ESP32-S3 GPIO | Notes |
|---|---:|---|
| I2C SDA / SCL | 8 / 9 | Shared sensor bus |
| Status LED | 47 | Emergency flashes rapidly |
| SIM868 UART TX / RX | 17 / 18 | 115200 baud |
| SIM868 power | 42 | Board power-control line |
| SOS button | 4 by default | Active-low; configurable in `menuconfig` |
| Motion interrupt | 2 by default | Reserved for sensor-driver integration |

## Build

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
```

Before enabling real SMS/call transmission, set the emergency call and SMS numbers in `menuconfig`, then complete cellular compliance and field testing.
