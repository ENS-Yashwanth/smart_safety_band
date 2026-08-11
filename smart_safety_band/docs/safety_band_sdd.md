# Safety Band Software Design Document (SDD)

## Document Information
- Project: Smart Safety Band
- Platform: ESP32-S3
- RTOS: FreeRTOS via ESP-IDF
- Primary Language: C
- Target Firmware: Embedded safety monitoring and communication application
- Status: Initial software design document aligned with the current repository implementation

---

# Chapter 1 – Introduction

## 1.1 Purpose
This document defines the software architecture, task model, resource management strategy, and implementation approach for the Smart Safety Band firmware. The goal is to provide a professional embedded-software engineering foundation for the system and to support future expansion from the current proof-of-concept implementation into a production-grade safety monitoring platform.

## 1.2 Scope
This document covers:
- Embedded firmware architecture on ESP32-S3
- Real-time task structure and scheduling
- Hardware resource mapping
- Inter-task communication and synchronization
- Memory and timing assumptions
- Error handling and future extensibility

The current repository already includes:
- ESP-IDF application entry point in the main application
- GPIO, I2C, UART, and ADC usage
- A LIS3DH accelerometer interface
- Battery monitoring via ADC
- GSM/serial modem diagnostics using UART

## 1.3 System Overview
The Smart Safety Band is an embedded safety-monitoring device intended to detect motion, monitor health-related conditions, assess battery state, and communicate alarms or diagnostic information through a wireless modem. The firmware is designed around FreeRTOS and ESP-IDF to provide deterministic task scheduling, modularity, and safe access to shared hardware resources.

## 1.4 Features
Current implementation features:
- ESP32-S3-based firmware execution
- Shared-I2C sensor discovery for the requested environmental and health devices
- LSM6DSOX fall-monitor path, with legacy LIS3DH bring-up compatibility
- Battery voltage monitoring using ADC and a low-battery event
- GPIO-based LED and debounced manual SOS handling
- UART single-owner SIM868 readiness check and emergency notification path
- Queue, mutex, event-group, task-notification, and ISR semaphore usage

Target future features:
- Heart rate and temperature monitoring
- Environmental sensing
- Emergency alert communication
- GPS and cellular data handling
- Event-based safety decision logic

## 1.5 Hardware Overview
The present implementation is based on the following hardware concepts:
- ESP32-S3 as the main controller
- LIS3DH accelerometer on I2C
- Battery voltage divider connected to ADC channel 0
- Status LED on a dedicated GPIO
- UART-connected modem interface for diagnostics and communication

## 1.6 Software Overview
The firmware is structured around ESP-IDF components and FreeRTOS tasks. The current design uses:
- GPIO driver for LEDs and modem power control
- I2C driver for sensor access
- ADC oneshot driver for battery measurement
- UART driver for modem interaction
- FreeRTOS tasks for periodic monitoring and diagnostics

## 1.7 Functional Requirements
The firmware shall:
- initialize hardware peripherals at startup
- detect and initialize the LIS3DH sensor
- read acceleration data and report it
- measure battery state through ADC
- expose modem diagnostics via UART AT command probing
- maintain a non-blocking and reliable runtime behavior on ESP32-S3

## 1.8 Non-Functional Requirements
The firmware shall:
- be deterministic under real-time constraints
- use bounded task execution time
- protect shared hardware resources from concurrent access
- remain modular and extensible for future sensors and communication modules
- be portable across ESP-IDF versions with minimal modifications

---

# Chapter 2 – System Architecture

## 2.1 High-Level Architecture

```text
                    +-----------------------------------+
                    |         Smart Safety Band         |
                    +-----------------------------------+
                                 |
         ---------------------------------------------------------
         |            |            |            |              |
         |            |            |            |              |
    Motion/Health  Environment  Communication    User/Power
      Sensing        Sensing       Interface      Interface
         |            |            |            |              |
         ---------------------------------------------------------
                                 |
                           FreeRTOS Kernel
                                 |
                 --------------------------------------------
                 |              |             |              |
              Drivers      Middleware       ESP-IDF HAL
```

## 2.2 Layered Architecture

```text
+-------------------------------------------------------------+
|                    Application Layer                        |
|-------------------------------------------------------------|
| Safety Manager                                              |
| Communication Manager                                       |
| Health Manager                                              |
| Environment Manager                                         |
+-------------------------------------------------------------+

+-------------------------------------------------------------+
|                      Service Layer                          |
|-------------------------------------------------------------|
| Sensor Service                                              |
| Queue Manager                                               |
| Battery Service                                             |
| Logger Service                                              |
+-------------------------------------------------------------+

+-------------------------------------------------------------+
|                      Driver Layer                           |
|-------------------------------------------------------------|
| LIS3DH Driver                                              |
| ADC Driver                                                 |
| UART Driver                                                |
| GPIO Driver                                                |
| I2C Driver
	spi driver
	i2s driver
	gl868_esp32  
                                               |
+-------------------------------------------------------------+

+-------------------------------------------------------------+
|                    ESP-IDF / FreeRTOS                      |
+-------------------------------------------------------------+
```

## 2.3 Current Repository Mapping
The current repository structure maps to this architecture as follows:
- Application entry point: main/safety_band_main.c
- Shared peripheral helper component: components/common_peripherals
- Future sensor-specific components: components/sensors
- Future communication functionality: components/communication
- Future safety policy logic: components/safety

---

# Chapter 3 – Hardware Resource Mapping

| Resource | Interface | Shared | Protection | Notes |
|---|---|---:|---|---|
| LSM6DSOX / BMI270 | I2C | Yes | Mutex | Fall detection; LSM6DSOX is configured by this firmware |
| BME680 / BME688 | I2C | Yes | Mutex | Presence-detected environment sensor |
| SCD40 / SCD41 | I2C | Yes | Mutex | Presence-detected CO2 sensor |
| VEML6075 | I2C | Yes | Mutex | Presence-detected UV sensor |
| AS3935 | I2C/SPI | Yes | Mutex | Presence-detected storm sensor (I2C reference wiring) |
| MAX86141 / MAX86176 | I2C | Yes | Mutex | Presence-detected pulse-ox sensor |
| MAX30208 / MAX30009 | I2C | Yes | Mutex | Presence-detected skin-temperature/GSR sensors |
| Battery ADC | ADC | No | None | Read from ADC1 channel 0 |
| Status LED | GPIO | No | None | Used for simple health indication |
| Modem power control | GPIO | No | None | Controlled by application logic |
| Modem UART | UART | No | Single owner | Used for AT command probing and future communication |
| I2C bus | I2C | Yes | Mutex | Must be protected when multiple sensor tasks are introduced |
| Serial console | UART/USB | No | Single owner | Logging output |

### Current Hardware Mapping
| Function | Pin / Resource | Notes |
|---|---|---|
| I2C SDA | GPIO8 | Used by LIS3DH |
| I2C SCL | GPIO9 | Used by LIS3DH |
| Motion interrupt | GPIO2 | Reserved for sensor-driver interrupt integration |
| SOS push button | GPIO4 default | Active-low; configurable in menuconfig |
| Status LED | GPIO47 | Toggle-based activity indicator |
| Battery ADC | ADC_CHANNEL_0 / GPIO1 | Analog battery sensing |
| Modem TX | GPIO17 | UART1 TX |
| Modem RX | GPIO18 | UART1 RX |
| Modem power | GPIO42 | Power switching for modem |

---

# Chapter 4 – Task Design

## 4.1 Task Model Overview
The firmware follows a lightweight FreeRTOS task model. The current implementation uses two main tasks, while the architecture is extensible to a fully modular safety-monitoring system.

## 4.2 Current Tasks

### Peripheral Test Task
- Purpose: Periodically report sensor and battery state.
- Period: 1000 ms
- Priority: 5
- Stack: 3072 bytes
- Synchronization: none beyond shared sensor access and logging
- Flow:
  1. Toggle status LED
  2. Read battery ADC
  3. Read accelerometer data
  4. Clear interrupt source if available
  5. Log state to serial console

### Modem Diagnostics Task
- Purpose: Probe the modem over UART, verify AT command response, and report modem state.
- Period: One-shot startup diagnostic followed by periodic keepalive loop
- Priority: 5
- Stack: 4096 bytes
- Synchronization: UART ownership and power-control sequencing
- Flow:
  1. Delay for startup
  2. Initialize modem power GPIO
  3. Probe UART baud rates and pins
  4. Send AT commands
  5. Log response or failure

## 4.3 Target Task Architecture
The following task structure is recommended for the full implementation.

### Motion Task
- Purpose: Read acceleration data and detect motion events.
- Period: 20 ms
- Priority: 9
- Stack: 4096 bytes
- Synchronization: I2C mutex, binary semaphore for interrupt-driven wakeup

### Temperature Task
- Purpose: Read temperature from the health sensor subsystem.
- Period: 1000 ms
- Priority: 6
- Stack: 3072 bytes
- Synchronization: I2C mutex

### Heart Rate Task
- Purpose: Process heart-rate sensor samples and compute rate.
- Period: 100 ms
- Priority: 7
- Stack: 4096 bytes
- Synchronization: I2C mutex

### Environment Task
- Purpose: Acquire environmental data such as temperature, humidity, and air quality.
- Period: 2000 ms
- Priority: 5
- Stack: 4096 bytes
- Synchronization: I2C/SPI mutex

### Communication Task
- Purpose: Handle modem state, AT commands, SMS/call logic, and network status.
- Period: Event-driven
- Priority: 7
- Stack: 6144 bytes
- Synchronization: UART ownership, event groups

### Logger Task
- Purpose: Buffer and log system events and sensor data.
- Period: Event-driven
- Priority: 4
- Stack: 4096 bytes
- Synchronization: Queue and mutex

### Safety Manager
- Purpose: Analyze incoming sensor events and determine emergency state.
- Period: Event-driven
- Priority: 8
- Stack: 4096 bytes
- Synchronization: Queues and task notifications

### LED Task
- Purpose: Control visual status and alarms.
- Period: 500 ms
- Priority: 3
- Stack: 2048 bytes
- Synchronization: None

### Battery Task
- Purpose: Sample and evaluate battery level.
- Period: 5000 ms
- Priority: 4
- Stack: 3072 bytes
- Synchronization: ADC access control

---

# Chapter 5 – FreeRTOS Synchronization

## 5.1 Queue
Queues are used to transfer sensor data and events between producer and consumer tasks. Example data flow:

```text
Motion Task -> Queue -> Safety Manager
Temperature Task -> Queue -> Safety Manager
Heart Rate Task -> Queue -> Safety Manager
```

Typical queue APIs:
- xQueueCreate()
- xQueueSend()
- xQueueReceive()

## 5.2 Mutex
A mutex protects the shared I2C bus so that sensor tasks do not corrupt each other’s transactions. The design should use priority inheritance so that a lower-priority task cannot block a higher-priority task indefinitely.

```text
Motion Task
  -> Take Mutex
  -> Access I2C
  -> Release Mutex
```

## 5.3 Binary Semaphore
Binary semaphores are suitable for interrupt-driven wakeups. In this design, motion interrupts can trigger a semaphore that wakes a sensor task.

```text
Accelerometer Interrupt -> ISR -> Give Semaphore -> Motion Task
```

## 5.4 Counting Semaphore
Counting semaphores can be used for audio or DMA-related buffering if the system later adds voice or streaming features.

## 5.5 Task Notification
Task notifications are useful when only one task needs to be notified. They are lightweight and faster than queues for simple event signaling.

Example:
```text
Safety Manager -> Notify -> Communication Task
```

## 5.6 Event Groups
Event groups are appropriate for coordinated state changes such as modem readiness, GPS readiness, or network availability.

```text
GPS Ready + GSM Ready + Internet Ready -> All Ready -> Upload
```

---

# Chapter 6 – Shared Resources

| Resource | Owner | Protection | Rationale |
|---|---|---|---|
| I2C bus | Sensor tasks | Mutex | Prevents interleaved sensor transactions |
| UART modem port | Communication task | Single owner | Prevents command corruption |
| ADC channel | Battery task | None or lightweight lock | Simple periodic sampling |
| Queue buffers | Sensor producers / safety manager | Queue API | Ensures bounded message passing |
| Event groups | Communication and safety logic | Event group API | Coordinated state synchronization |

---

# Chapter 7 – Timing Analysis

## 7.1 Execution Periods
The firmware timing model is based on periodic sensing and event-driven handling.

| Function | Period |
|---|---:|
| Motion sensing | 20 ms |
| Heart rate monitoring | 100 ms |
| Temperature monitoring | 1000 ms |
| Environment monitoring | 2000 ms |
| Battery monitoring | 5000 ms |
| LED update | 500 ms |

## 7.2 Worst-Case Timing
The current implementation is lightweight and low duty cycle. The dominant timing cost is the repeated sensor and UART operations. The firmware should ensure that each task completes within its period and that no blocking call exceeds the planned deadline.

## 7.3 CPU Utilization
With the current implementation, CPU utilization remains low because the system performs short sensor reads and logging bursts. As the feature set expands, timing should be reassessed using ESP-IDF tracing and profiling tools.

---

# Chapter 8 – Scheduling

## 8.1 Priority Assignment
A practical priority strategy is:
- Motion / Safety logic: highest priority
- Communication: high priority
- Temperature / Heart rate: medium priority
- Logging / LED / battery: lower priority

## 8.2 Preemption
FreeRTOS preempts lower-priority tasks whenever higher-priority tasks become ready. This ensures that safety-critical functions are serviced promptly.

## 8.3 Priority Inversion
Priority inversion is addressed by using mutex priority inheritance. This is especially important when sensor access is shared across multiple tasks.

## 8.4 ESP32-S3 Scheduling Notes
The ESP32-S3 uses FreeRTOS scheduling with support for dual-core execution. The design should avoid shared resource races and should prefer deterministic task placement during later integration.

---

# Chapter 9 – Memory Design

## 9.1 Task Stacks
Recommended stack sizes:
- Motion Task: 4096 bytes
- Communication Task: 6144 bytes
- Safety Manager: 4096 bytes
- Logger Task: 4096 bytes
- Temperature/Heart Rate Tasks: 3072–4096 bytes
- LED/Battery Tasks: 2048–3072 bytes

## 9.2 Heap Usage
Heap consumption should remain bounded by:
- Static task stacks
- Queue storage
- Logging buffers
- Sensor data buffers

## 9.3 Buffer Strategy
The firmware should use fixed-size buffers for UART, logging, and queue messages to avoid heap fragmentation and runtime uncertainties.

---

# Chapter 10 – Sequence Diagrams

## 10.1 Emergency Detection Flow
```text
Motion Task
  -> Queue Send
  -> Safety Manager
  -> Determine Emergency Condition
  -> Notify Communication Task
  -> Modem / SMS / Alert Handling
```

## 10.2 Battery Monitoring Flow
```text
Battery Task
  -> ADC Read
  -> Evaluate Threshold
  -> Log / Notify Safety Manager
```

---

# Chapter 11 – State Machines

## 11.1 System State Machine
```text
INIT -> IDLE -> MONITOR -> EMERGENCY -> COMMUNICATION -> RECOVERY -> MONITOR
```

## 11.2 Communication State Machine
```text
POWER_ON -> REGISTER_NETWORK -> GPS_READY -> ALERT_SEND -> SLEEP/WAIT
```

---

# Chapter 12 – Folder Structure

```text
main/
├── safety_band_main.c
├── CMakeLists.txt
├── idf_component.yml
├── Kconfig.projbuild
│
components/
├── common_peripherals/
│   ├── include/
│   │   └── periph.h
│   └── src/
│       └── periph.c
├── sensors/
├── communication/
└── safety/
```

## 12.1 Recommended Future Expansion
The current structure is already suitable for modular growth:
- sensors/ for sensor drivers and services
- communication/ for modem and transport logic
- safety/ for safety state and alarm policy

---

# Chapter 13 – Error Handling

## 13.1 Sensor Timeout
If a sensor transaction times out:
- retry once or twice
- log the failure
- report degraded mode

## 13.2 Queue Full
If a queue is full:
- discard oldest data or newest data depending on policy
- prevent system deadlock

## 13.3 Modem Failure
If modem initialization or AT probing fails:
- retry with alternate baud rate and UART pin mapping
- log the error clearly
- continue without blocking the rest of the system

## 13.4 Battery Low
If the battery is below a threshold:
- reduce activity
- shift to a low-power mode
- prepare for emergency communication

---

# Chapter 14 – Future Enhancements

The present design is intentionally modular so that the following enhancements can be added cleanly:
- BLE communication
- Wi-Fi connectivity
- MQTT-based cloud reporting
- OTA firmware updates
- GPS integration
- AI-assisted fall or anomaly detection
- Voice command or audio alert features

---

# Summary
The Smart Safety Band firmware is currently at an early but structured stage of implementation. The existing ESP-IDF project already demonstrates the core building blocks of an embedded safety system: sensor access, ADC measurement, GPIO control, and modem diagnostics. The design in this document establishes a professional path forward by introducing a realistic task architecture, synchronization strategy, resource mapping, and modular software organization suitable for further development into a production-ready safety device.
