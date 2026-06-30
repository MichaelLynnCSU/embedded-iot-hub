# Smart Home IoT System

A fault-tolerant distributed IoT system across 10+ MCUs — sensing, control,
persistence, networking, inference, and display each run as isolated processes
or dedicated hardware nodes for independent failure recovery.

## Architecture
![System Architecture](docs/Arch%20diagram2.png)

## Hardware

| Node | MCU | Role |
|------|-----|------|
| BLE Hub | ESP32 (WROOM) | BLE scan, TCP forwarding, UART bridge, PIR occupancy window |
| Dashboard | STM32F411 BlackPill | LVGL display, ILI9341 240x320 |
| Temp/UART | STM32F103 BluePill | DHT11 avg_temp → UART to ESP32 |
| Motor | ESP32-C3 | PWM motor control, TCP server |
| Reed x2 | nRF52840 | Door/window state + battery SOC via BLE adv (up to 6) |
| PIR x5 | ESP32-C3 (Zephyr) | Motion count + battery SOC via BLE adv |
| Smart Lock | nRF52840 | Lock state + battery SOC via BLE adv |
| Smart Light | nRF52840 | Light state via BLE adv + GATT write |
| Temp Sensor | nRF52840 | Temperature + battery SOC via BLE adv (TempSensor1/2) |
| Security CAM x3 | ESP32-S3 | PIR-triggered JPEG clip → BeagleBone TCP 9090 |
| Doorbell CAM x4 | ESP32 (AI-Thinker, classic D0WD) | Button-triggered JPEG/MJPEG → BeagleBone TCP 9091/9093 |
| BeagleBone | AM335x | Linux pipeline: parse, persist, infer, display |

## BLE Protocol

All nRF52840 nodes broadcast state passively in manufacturer advertisement
data — zero connection overhead for monitoring. The ESP32 hub tracks device
age and battery from scan data alone.

| Device | Company ID | Payload |
|--------|------------|---------|
| Reed Sensors | 0xAB | `[state, batt_soc]` |
| Smart Lock | 0xAC | `[state, batt_soc]` |
| Smart Light | 0xAD | `[state]` |
| PIR (x5) | 0xFF 0xFF | `[count BE 4 bytes, batt_soc]` |

GATT connections established only for write commands (lock/light control).
Device disconnects immediately after write confirmation.

## PIR Occupancy and Camera Trigger Pipeline

The hub maintains a per-slot sliding window occupancy state for each of the
5 PIR sensors and publishes occupancy to the BeagleBone. The BeagleBone
detects 0→1 transitions and dispatches CAPTURE triggers directly to the
security camera over UDP. The hub is transport only — it does not decide
when to capture.

```
PIR BLE advertisement
    → ESP32 Hub BLE scan
    → pir_window_update(slot)
    → bus_publish_pir() → BeagleBone controller (TCP)

BeagleBone controller
    → sensor_dispatch detects PIR 0→1 occupancy transition
    → CamTriggerRequest → UNIX socket → camera_manager

camera_manager
    → lookup camera IP from heartbeat registry
    → verify camera online
    → UDP CAPTURE:event_id=...,zone=... → ESP32-CAM:9091

ESP32-CAM
    → captures JPEG clip (10s, ~20 frames at 500ms intervals)
    → TCP connect → BeagleBone:9090
    → inference_daemon receives clip
    → TFLite person detection per frame
    → best result saved to /data/clips/ as .avi
```

Camera IPs are learned dynamically from UDP heartbeat source addresses
(recvfrom) — no static IP configuration required.

## Doorbell Pipeline

The doorbell camera is an independent subsystem with its own Wi-Fi
connection to the BeagleBone. It does not depend on the hub.

The doorbell firmware is compiled in one of two modes set at build time:

- **SNAPSHOT mode** (default): button press → single JPEG → BeagleBone TCP 9091
  → `doorbell_daemon` → TFLite inference → POSIX SHM → STM32 display
- **STREAM mode**: button press toggles MJPEG stream → BeagleBone TCP 9093
  → `doorbell_stream_daemon` → ffmpeg → mediamtx → RTSP

Up to 4 doorbell cameras share each port, identified by `device_id` (0–3)
in the TCP packet header.

```
Doorbell button press
    → TCP connect → BeagleBone:9091 (SNAPSHOT) or :9093 (STREAM)
    → doorbell_daemon / doorbell_stream_daemon
    → inference + SHM publish (SNAPSHOT)
    → RTSP stream via mediamtx (STREAM)
```

## Camera Port Reference

| Port | Protocol | Direction | Service | Purpose |
|------|----------|-----------|---------|---------|
| 9090 | TCP | inbound | inference_daemon | Security CAM JPEG clip receive |
| 9091 | TCP | inbound | doorbell_daemon | Doorbell SNAPSHOT JPEG receive |
| 9091 | UDP | outbound | camera_manager | CAPTURE trigger → ESP32-CAM |
| 9093 | TCP | inbound | doorbell_stream_daemon | Doorbell MJPEG stream receive |
| 9094 | UDP | inbound | camera_manager | Heartbeats from all camera devices |

## Dynamic Reed Discovery

Reed sensors are auto-discovered by BLE name prefix `ReedSensor*`. Adding a
new sensor requires no code changes anywhere in the stack:

1. Flash nRF52840 with `CONFIG_BT_DEVICE_NAME="ReedSensor3"`
2. Power on — ESP32 assigns a slot by MAC address
3. BeagleBone receives new slot in JSON, forwards via UART
4. STM32 receives `REED_COUNT:3`, calls `UI_Reflow(3)` — new tile appears

Up to 6 reed sensors supported (`ReedSensor1`–`ReedSensor6`). Currently 2
are deployed.

**Slot state machine:**
```
SLOT_ACTIVE  — advertising within 150s  → green dot
SLOT_OFFLINE — unseen > 150s            → red dot, tile stays
SLOT_EMPTY   — unseen > 3600s           → tile hidden, layout reflows
```

Cooldown table prevents immediate slot re-allocation after removal. Generation
counter increments on device swap — detectable across the full stack.

## BeagleBone Pipeline

Ten-process architecture, each an independent systemd service:

```
esp01-tcp-server        — AT command setup for ESP-01 WiFi module (oneshot)
sensor-server           — UART rx from ESP32, JSON parse, named pipe write
camera-manager          — UDP ingress for camera + doorbell devices, IP registry,
                          CAPTURE trigger dispatch, liveness pipe write
data-controller         — pipe read, SQLite write, SHM update, PIR cam trigger
inference               — TCP server for ESP32-CAM JPEG clips, TFLite detection
doorbell                — TCP server for doorbell JPEG, TFLite detection, SHM publish
doorbell_stream         — TCP server for doorbell MJPEG → ffmpeg → mediamtx
mediamtx                — RTSP server for doorbell live stream
api-server              — REST API (Python) over POSIX SHM
lcd-display             — thermostat + sensor readout on I2C LCD
```

**Service boot order:**
```
network-online.target
    → esp01-tcp-server  (oneshot, configures ESP-01 AT server)
    → data-controller
    → camera-manager    (waits for /tmp/cam_pipe before start)
    → sensor-server
    → inference
    → doorbell
    → doorbell_stream
    → mediamtx
    → api-server
    → lcd-display
```

UART push format to STM32:
```
STATE:tmp,pir,lgt,lck,age_pir,age_lgt,age_lck,reed_count
PIR:total_count
PIR_COUNT:n
PIR1:count,batt,age
PIR2:count,batt,age
...PIRn
OCC1:occupied
OCC2:occupied
...OCCn
REED_COUNT:n
DR1:state,batt,age
DR2:state,batt,age
LGT:state
LCK:state,batt
MTR:online
```

## Crash Logging

All nodes persist crash state across hard power loss:

| Node | Storage | Data |
|------|---------|------|
| nRF52840 | Zephyr `flash_area` circular log | PC/LR on HardFault |
| ESP32 | NVS + coredump | Boot reason, reset cause |
| STM32F411 | RTC backup registers | PC/LR, fault type |
| STM32F103 | FRAM (16KB partition) | PC/LR, fault type, magic 0xA5 |

## Build & Flash

### ESP32 Hub
```bash
cd esp32-hub
idf.py build flash monitor
```

### ESP32-S3 Security CAM
```bash
cd esp32-cam
idf.py set-target esp32s3
idf.py -DCAM_SLOT=0 build flash   # cam 0
idf.py -DCAM_SLOT=1 build flash   # cam 1
idf.py -DCAM_SLOT=2 build flash   # cam 2
```

### ESP32 Doorbell CAM
```bash
cd esp32-doorbell
# SNAPSHOT mode (default)
idf.py -DDOORBELL_ID=0 -DDOORBELL_MODE=SNAPSHOT build
# STREAM mode
idf.py -DDOORBELL_ID=1 -DDOORBELL_MODE=STREAM build
idf.py -p /dev/ttyUSB0 flash
```

### BeagleBone
```bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

# Sensor Server
cd beaglebone/server
${CROSS_COMPILE}gcc -g -O0 \
  sensor_server.c uart_io.c json_parser.c pipe_writer.c build_info.c \
  -o sensor_server -I../include -ljson-c

# Camera Manager
cd ../camera_manager
arm-linux-gnueabihf-gcc -g -O0 \
  camera_manager.c build_info.c \
  -o camera_manager -I../include -lpthread

# Data Controller
cd ../controller
${CROSS_COMPILE}gcc -g -O0 \
  data_controller.c \
  pipeline/pipe_reader.c pipeline/cam_pipe_reader.c \
  pipeline/sensor_dispatch.c pipeline/cam_dispatch.c \
  state/state_registry.c state/shm_updater.c state/db_persist.c \
  uart/uart_staging.c uart/uart_transport.c uart/uart_controller.c uart/uart_lock.c \
  doorbell/doorbell_pending.c doorbell/doorbell_result_reader.c \
  build_info.c db_manager.c heartbeat.c \
  -I./include -I. -I../inference/app -I../include \
  -o data_controller -lpthread -lrt -lm -lsqlite3

sudo systemctl restart data-controller sensor-server camera-manager inference
```

### STM32F411 / STM32F103
Open CubeIDE project, build and flash via ST-Link.

### nRF52840 (Zephyr)
```bash
cd nrf52840/reed-sensor
west build -- -DCONFIG_BT_DEVICE_NAME=\"ReedSensor1\" && west flash
```

### ESP32-C3 Motor
```bash
cd esp32c3/idf/motor/
idf.py build flash monitor
```

### ESP32-C3 PIR
```bash
cd esp32c3/zephyr/pir/
west build && west flash
```

## Project Structure

```
embedded-iot-hub/
├── esp32-hub/                  # ESP-IDF BLE central + TCP forwarder
│   └── main/
│       ├── ble_pir.c           # BLE PIR handler, occupancy window, publish
│       ├── pir_window.c/h      # per-slot occupancy sliding window
│       ├── tcp_manager.c       # BeagleBone state machine
│       └── motor_server.c/h    # TCP server for ESP32-C3 motor
│
├── esp32-cam/                  # ESP32-S3 security camera (PIR-triggered clip)
│   └── main/
│       ├── main.c              # UDP trigger rx, JPEG clip capture, TCP push
│       └── cam_logic.h         # port config, trigger parse, clip constants
│
├── esp32-doorbell/             # ESP32 doorbell camera (AI-Thinker, OV2640)
│   └── main/
│       ├── main.c              # button ISR, SNAPSHOT/STREAM modes
│       └── cam_logic.h         # port config, header pack/unpack, mode flags
│
├── beaglebone/                 # Embedded Linux pipeline
│   ├── include/                # Shared ABI headers
│   │   ├── ipc_proto.h         # Wire-format structs (pipe/SHM ABI)
│   │   ├── shared_data.h       # Shared state structures
│   │   └── cam_trigger_ipc.h   # CamTriggerRequest IPC contract
│   ├── controller/             # Data controller — event processing pipeline
│   │   ├── pipeline/           # Pipe readers + event dispatchers
│   │   ├── state/              # State registry, SHM updater, DB persistence
│   │   ├── uart/               # UART transport, controller, lock, staging
│   │   └── doorbell/           # Doorbell pending + result reader
│   ├── server/                 # sensor_server, json_parser
│   ├── camera_manager/         # UDP ingress, IP registry, CAPTURE dispatch
│   ├── inference/              # TFLite inference_daemon, doorbell_daemon,
│   │   └── app/                # doorbell_stream_daemon, detect.tflite
│   ├── api/                    # REST API — api_server.py, shm_reader.py
│   ├── motor-thermostat-lcd/   # I2C LCD thermostat display
│   └── wifi/                   # ESP-01 AT setup script
│
├── esp32c3/                    # ESP32-C3 targets
│   ├── idf/motor/              # Motor PWM controller (ESP-IDF)
│   └── zephyr/pir/             # PIR sensor x5 (Zephyr)
│
├── nrf52840/                   # Zephyr BLE peripheral nodes
│   ├── reed-sensor/            # Door/window reed sensor (up to 6)
│   ├── temp-sensor/            # BLE temperature sensor (TempSensor1/2)
│   ├── smart-light/
│   └── smart-lock/
│
├── stm32-blackpill/            # STM32F411 LVGL dashboard
├── stm32-bluepill/             # STM32F103 DHT11 + UART bridge
├── docs/
├── README.md
└── ENVIRONMENT.md
```

## Testing

### CI/CD — Jenkins + Cloudflare Tunnel

Every push to `master` triggers a full automated test run via Jenkins. The
Jenkins server runs on the local network and is exposed externally through a
Cloudflare Tunnel — no port forwarding or static IP required.

The pipeline runs all test suites in sequence, collects JUnit XML results, and
publishes per-module coverage reports via the Coverage plugin:

| Stage | Framework | Coverage |
|-------|-----------|----------|
| BeagleBone Controller | CUnit | gcovr → Cobertura XML |
| BeagleBone Server | CUnit | gcovr → Cobertura XML |
| BeagleBone Inference | CUnit | gcovr → Cobertura XML |
| ESP32 Hub | Unity (FetchContent) | gcovr → Cobertura XML |
| ESP32-CAM | Unity (FetchContent) | gcovr → Cobertura XML |
| ESP32-C3 Motor | Unity (FetchContent) | gcovr → Cobertura XML |
| STM32 BlackPill | Unity (FetchContent) | gcovr → Cobertura XML |
| STM32 BluePill | Unity (FetchContent) | gcovr → Cobertura XML |
| nRF52840 + PIR (Zephyr) | ztest · native_sim | gcovr → Cobertura XML |

The Zephyr stage runs with `catchError(buildResult: 'UNSTABLE')` — a missing
toolchain marks the build unstable rather than failed so other stages still run.

### Manual testing

All unit tests run on-host — no hardware required.

### ESP32 Hub
```bash
cd esp32-hub/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_hub
```

### ESP32-CAM
```bash
cd esp32-cam/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_cam
```

### ESP32-C3 Motor
```bash
cd esp32c3/idf/motor/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_motor
```

### nRF52840 (native_sim — all nodes)
```bash
# Run from any node directory: reed-sensor, temp-sensor, smart-lock, smart-light
rm -rf build_test
west build -b native_sim tests/unit --build-dir build_test
west build -t run --build-dir build_test
```

### STM32 BlackPill
```bash
cd stm32-blackpill/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_blackpill
```

### STM32 BluePill
```bash
cd stm32-bluepill/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_bluepill
```

### BeagleBone — Controller
```bash
cd beaglebone/controller/tests/unit
rm -rf build && mkdir build && cd build
cmake .. && make && ./test_controller && ./test_db_tx
```

### BeagleBone — Server
```bash
cd beaglebone/server/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_server
```

### BeagleBone — Inference
```bash
cd beaglebone/inference/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_inference
```

## Key Design Decisions

**Why scan-based BLE?** Passive advertisement monitoring gives the hub
continuous device age and state with zero connection overhead. Battery life
on coin/AA nodes is measured in months.

**Why BeagleBone?** AM335x gives a full Linux environment for SQLite, systemd
service isolation, TFLite inference, and future HTTP/AWS expansion without
the constraints of an RTOS.

**Why FRAM for crash logging on STM32F103?** Unlimited write endurance and
byte-addressable writes survive hard power loss. No wear leveling needed, no
erase cycles, no filesystem overhead.

**Why dedicated ESP32-C3 for motor?** Oscilloscope analysis confirmed DHT11
timing violations under shared STM32 interrupt load. Dedicated silicon
eliminates the constraint entirely.

**Why per-trigger TCP for ESP32-CAM?** A persistent connection from the CAM
to the BeagleBone timed out during long idle intervals between PIR triggers.
Connecting per-trigger eliminates the timeout entirely with negligible latency
overhead given the infrequent capture rate.

**Why does the BeagleBone send the camera trigger rather than the hub?**
The hub is a BLE transport node — it scans, computes occupancy, and forwards
events. Trigger authority belongs to the BeagleBone, which owns the inference
pipeline, the camera registry, and occupancy policy. This keeps all camera
networking in `camera_manager`: IP learning, liveness, rate limiting, and
dispatch are all in one place. The hub needs no knowledge of camera addresses.

**Why dynamic IP learning for cameras?** The `camera_manager` learns each
camera's IP from the UDP heartbeat source address (`recvfrom`). No static IP
configuration is required, and the registry updates automatically if DHCP
reassigns an address.

**Why static PIR slots?** The static slot system gives predictable memory
layout, zero heap fragmentation, and FreeRTOS-friendly fixed allocation.
Adding sensors beyond MAX_PIRS requires a recompile and reflash. See issue
#38 for the dynamic discovery roadmap if sensor count grows.

**Why separate SNAPSHOT and STREAM modes for the doorbell?** The two use
cases have different requirements. SNAPSHOT delivers a single high-quality
JPEG per button press with full inference and event correlation. STREAM
delivers continuous low-latency MJPEG for live viewing via RTSP. Combining
them in one firmware path would require runtime mode switching and complicate
the TCP framing. Compile-time selection keeps each path simple and testable.
