# GPS Power Management in Smart Safety Band

## Overview
The GPS (GNSS) is **enabled on demand** and can be **powered off** to save energy. The power state is controlled by configuration flags and operational triggers.

---

## When GPS Powers OFF or Enters Sleep

### 1. **After Initial GPS Fix Attempt (During Initialization)**
**Location:** `gl868_modem_init()` (line ~1298)

```c
if (acquire_gps_fix_with_retries(initial_fix, GPS_FIX_RETRY_COUNT, ...)) {
    cache_gps_fix(initial_fix);
    ESP_LOGI(TAG, "Initial GPS fix cached: %.6f,%.6f", ...);
} else {
    ESP_LOGW(TAG, "No initial GPS fix after %d attempts; will retry when needed", ...);
}
disable_gps();  // <-- GPS POWERED OFF HERE
ESP_LOGI(TAG, "GNSS powered off after initial fix attempt; GPS will restart on demand");
```

**Condition:** `#if defined(CONFIG_SAFETY_BAND_GNSS_POWER_SAVE) && CONFIG_SAFETY_BAND_GNSS_POWER_SAVE`

**When:** 
- At the end of modem initialization
- Whether or not a fix was successfully cached
- GPS will restart on demand (SOS button or periodic uploads)

**Log:** `"GNSS powered off after initial fix attempt; GPS will restart on demand"`

---

### 2. **After SOS Emergency Handling**
**Location:** `gl868_modem_trigger_emergency()` (line ~1422)

```c
bool gps_ok = false;
if (cached_fix_is_fresh()) {
    fix_info.latitude = s_state.cached_lat;
    fix_info.longitude = s_state.cached_lon;
    // ... use cached fix
} else {
    // Attempt fresh GPS fix acquisition
    if (acquire_gps_fix_with_retries(fix_info, GPS_FIX_RETRY_COUNT, ...)) {
        gps_ok = true;
    }
}

// Send SMS and make call with location
send_sms(..., location_string);
make_call(call_number);

#if defined(CONFIG_SAFETY_BAND_GNSS_POWER_SAVE) && CONFIG_SAFETY_BAND_GNSS_POWER_SAVE
    disable_gps();  // <-- GPS POWERED OFF HERE
    ESP_LOGI(TAG, "GNSS powered off after SOS handling");
#endif
```

**Condition:** `#if defined(CONFIG_SAFETY_BAND_GNSS_POWER_SAVE) && CONFIG_SAFETY_BAND_GNSS_POWER_SAVE`

**When:** 
- After emergency call and SMS are sent
- Regardless of whether a fresh GPS fix was obtained

**Log:** `"GNSS powered off after SOS handling"`

---

### 3. **Manual Control via Public API**
**Function:** `gl868_modem_set_gnss_power(bool enabled)` (line ~1629)

```c
extern "C" bool gl868_modem_set_gnss_power(bool enabled) {
    if (!s_state.initialized) {
        return false;
    }
    return enabled ? enable_gps() : disable_gps();
}
```

**How to use:**
```c
gl868_modem_set_gnss_power(0);  // Powers off GPS
gl868_modem_set_gnss_power(1);  // Powers on GPS
```

---

## Current GPS Power State Management

| Condition | GPS State |
|-----------|-----------|
| At boot | OFF |
| During modem init | Powered ON for ~5 attempts (40 seconds) to get initial fix |
| After init (power-save enabled) | **OFF** (runs on demand) |
| After init (power-save disabled) | **ON** (always running) |
| On SOS trigger | Powered ON to get fresh fix OR uses cached fix |
| After SOS (power-save enabled) | **OFF** (until next SOS or upload) |
| Periodic uploads | GPS powered on only during upload window |

---

## Power Save Configuration

The GPS power save behavior is controlled by:

```c
#if defined(CONFIG_SAFETY_BAND_GNSS_POWER_SAVE) && CONFIG_SAFETY_BAND_GNSS_POWER_SAVE
```

**Default:** NOT DEFINED (GPS stays ON after init)

**To enable GPS power save:**
Add to sdkconfig:
```
CONFIG_SAFETY_BAND_GNSS_POWER_SAVE=y
```

Or define in Kconfig.projbuild:
```
config SAFETY_BAND_GNSS_POWER_SAVE
    bool "Enable GNSS power saving after init and SOS"
    default n
    help
        Powers off GNSS after initial fix attempt and after SOS handling
        to reduce power consumption. GPS restarts on demand.
```

---

## GPS Enable/Disable Functions

### `disable_gps()` - Powers off GNSS (line ~909)
```c
bool disable_gps(void) {
    if (!s_state.gps_enabled) {
        return true;  // Already off
    }
    
    // Send: AT+CGNSPWR=0
    if (!send_at_command("AT+CGNSPWR=0\r", &response, 3000)) {
        ESP_LOGW(TAG, "GPS power off: FAILED -> %s", ...);
        return false;
    }
    s_state.gps_enabled = false;
    ESP_LOGI(TAG, "GPS power off: SUCCESS");
    return true;
}
```

**AT Command:** `AT+CGNSPWR=0` → Disables GNSS power

**Logs:**
- Success: `"GPS power off: SUCCESS"`
- Failure: `"GPS power off: FAILED -> <error>"`

---

### `enable_gps()` - Powers on GNSS (line ~879)
```c
bool enable_gps(void) {
    if (s_state.gps_enabled) {
        return true;  // Already on
    }
    
    // Send: AT+CGNSPWR=1
    if (!send_at_command("AT+CGNSPWR=1\r", &response, 3000)) {
        ESP_LOGW(TAG, "GPS power on: FAILED -> %s", ...);
        return false;
    }
    
    // Configure sentence output to RMC
    if (!send_at_command("AT+CGNSSEQ=\"RMC\"\r", &response, 3000)) {
        ESP_LOGW(TAG, "GPS sentence configuration failed -> %s", ...);
        s_state.gps_enabled = false;
        return false;
    }
    
    ESP_LOGI(TAG, "GPS sentence configuration: SUCCESS");
    s_state.gps_enabled = true;
    return true;
}
```

**AT Commands:**
- `AT+CGNSPWR=1` → Enables GNSS power
- `AT+CGNSSEQ="RMC"` → Sets sentence output format (RMC = essential fix + satellite data)

**Logs:**
- Success: `"GPS sentence configuration: SUCCESS"`
- Failure: `"GPS power on: FAILED -> <error>"`

---

## Key Points

1. **GPS is not in "sleep state"** - it's either fully powered ON or fully powered OFF
2. **Power-save disabled (default):** GPS stays ON after initialization
3. **Power-save enabled:** GPS powers off after init and after SOS, restarts on demand
4. **Cached fixes:** After first acquisition, recent fixes are cached and reused if still fresh (< 10 minutes)
5. **On-demand restart:** GPS powers back on when needed (SOS, periodic upload tasks)
6. **Manual control:** Use `gl868_modem_set_gnss_power()` to control GPS state from application code

---

## Recommendation

To reduce power consumption:
1. Enable `CONFIG_SAFETY_BAND_GNSS_POWER_SAVE=y`
2. GPS will power down after init and SOS events
3. Implement periodic location updates (e.g., every 5 minutes) to minimize GPS uptime
4. Use cached fixes when possible (less than 10 minutes old)
