# Smart Safety Band firmware

ESP-IDF / FreeRTOS firmware for an ESP32-S3 Mini-1 based GeoLinker GL868 safety band. It provides modem initialization, manual SOS SMS/call alerts, GPS acquisition, and a single-owner SIM868 communication task so no two tasks issue AT commands on the UART at the same time.

Optional I2C devices are probed without preventing boot: BME680/BME688, SCD40/SCD41, VEML6075, AS3935, MAX86141/MAX86176, MAX30208, and MAX30009. Their measurement/compensation drivers remain hardware-specific integration points; development-only simulated values are always marked `SIM`.

## Runtime tasks

All application task entry points are in `main/safety_band_main.c`:

- `communication_task` powers on and initializes the modem, then serializes emergency calls/SMS, GPS reads, GPRS, and HTTP.
- `gps_task` queues a GeoLinker location update immediately after modem readiness and every two minutes thereafter.
- `sos_button_task` debounces the active-low SOS button and queues one emergency request per press.

The SMS alert format is `ALERT: SOS activated! Loc: <lat>,<lon>. Map: [<url>] (<url>) Batt: <percent>%`.

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
