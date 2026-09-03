# Smart Safety Band Firmware

ESP-IDF / FreeRTOS firmware for an ESP32-S3 Mini-1 based GeoLinker GL868 safety band. It provides modem initialization, manual SOS SMS/call alerts, GPS acquisition, and a single-owner SIM868 communication task so no two tasks issue AT commands on the UART simultaneously.

Optional I2C devices are probed without preventing boot: BME680/BME688, SCD40/SCD41, VEML6075, AS3935, MAX86141/MAX86176, MAX30208, and MAX30009. Their measurement/compensation drivers remain hardware-specific integration points; development-only simulated values are always marked `SIM`.

---

## Runtime Tasks

All application task entry points are located in `main/safety_band_main.c`:

* **`communication_task`**: Powers on and initializes the modem, then serializes emergency calls/SMS, GPS reads, GPRS bearer context, and HTTP telemetry POST requests.
* **`gps_task`**: Queues live-location telemetry uploads every two minutes (and every 5 seconds during active SOS).
* **`sos_button_task`**: Debounces the active-low SOS button and queues one emergency request per press.

The default emergency SMS alert format is:
`ALERT: SOS activated! Loc: <lat>,<lon>. Map: https://maps.google.com/?q=<lat>,<lon> Batt: <percent>%`

---

## HTTP Telemetry & Smart City Event Schema

The band attaches GPRS using the configured APN (e.g., `airtel.in`), initializes PDP bearer context 1, and submits normalized JSON events via HTTP POST.

### Normalized Event Payload Example (`smart-city-event/1.0`)

```json
{
  "schema_version": "smart-city-event/1.0",
  "event_id": "smart_safety_band_001-device.live_location-1",
  "correlation_id": "device:smart_safety_band_001",
  "event_type": "device.live_location",
  "severity": "critical",
  "occurred_at": "2026-09-03T10:00:00Z",
  "source": {
    "kind": "smart-band",
    "source_id": "smart_safety_band_001",
    "deployment_id": null
  },
  "location": {
    "kind": "geo",
    "site_id": "hyderabad-demo",
    "zone_id": null,
    "latitude": 17.385044,
    "longitude": 78.486671,
    "accuracy_m": 8
  },
  "evidence": [],
  "payload": {
    "band_id": "smart_safety_band_001",
    "sos_status": "active",
    "update_type": "location",
    "sequence": 1,
    "location_source": "gps",
    "altitude_m": null
  }
}
```

---

## Hardware Limitations & Networking Architecture

### 1. SSL/TLS SNI Limitation (Error 606)
* **SIM868 Modem Firmware (R14.18)** lacks support for Server Name Indication (SNI) extensions during TLS handshakes.
* Direct HTTPS connections to SNI-enforced servers (e.g., Cloudflare, Ngrok, or HTTPS Beeceptor) trigger modem status code **`606`** (SSL Handshake Failure).
* **Recommended Setup**: Use plain **HTTP (`http://`)** endpoints or a local reverse proxy / Beeceptor Local Tunnel to bridge incoming HTTP traffic to backend HTTPS ingestion services (`/api/v1/events/normalized`).

### 2. Rate-Limit Handling (HTTP 429 Protection)
* If the testing endpoint returns **HTTP Status 429 (Too Many Requests)**, the modem bridge logs a warning and **suppresses redundant retries** for that packet to prevent server IP blacklisting.

### 3. GPRS PDP Bearer Refresh Strategy
* Cellular NAT connections on 2G networks (such as Airtel) can expire during idle intervals.
* The modem bridge verifies and proactively refreshes the PDP context (`AT+SAPBR=2,1`) before initiating POST transactions to prevent connection timeouts (modem status `601/603`).

### 4. GNSS Fix & Fallback Coordinates
* **Valid Satellite Fix**: When a valid GNSS fix is acquired, true coordinates are embedded in the telemetry event.
* **Pending GNSS Fix**: If satellite fix is unavailable during startup or indoors, the system falls back to `(0.0, 0.0)` coordinates, allowing telemetry event delivery to proceed without blocking emergency updates.

---

## Reference Wiring

| Function | ESP32-S3 GPIO | Notes |
|---|---:|---|
| I2C SDA / SCL | 8 / 9 | Shared sensor bus |
| Status LED | 47 | Emergency flashes rapidly |
| SIM868 UART TX / RX | 17 / 18 | 115200 baud |
| SIM868 Power | 42 | Board power-control line |
| SOS Button | 4 (default) | Active-low; configurable in `menuconfig` |
| Motion Interrupt | 2 (default) | Reserved for sensor-driver integration |

---

## Build & Flash Instructions

```bash
# Set target chip
idf.py set-target esp32s3

# Configure credentials, APN, and telemetry endpoint
idf.py menuconfig

# Build and flash
idf.py build
idf.py -p PORT flash monitor
```
