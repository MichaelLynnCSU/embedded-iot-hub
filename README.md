# Smart Home IoT System

A fault-tolerant distributed IoT system across 10+ MCUs — sensing, control,
persistence, networking, inference, and display each run as isolated processes
or dedicated hardware nodes for independent failure recovery.

## Architecture
![System Architecture](docs/Arch%20diagram2.png)

## Hardware

| Node | MCU | Role |
|------|-----|------|
| BLE Hub | ESP32 | BLE scan, TCP forwarding, UART bridge, PIR→CAM trigger |
| Dashboard | STM32F411 BlackPill | LVGL display, ILI9341 240x320 |
| Temp/UART | STM32F103 BluePill | DHT11 avg_temp → UART to ESP32 |
| Motor | ESP32-C3 | PWM motor control, TCP server |
| Reed x2 | nRF52840 | Door state + battery SOC via BLE adv |
| PIR x5 | ESP32-C3 (Zephyr) | Motion count + battery SOC via BLE adv |
| Smart Lock | nRF52840 | Lock state + battery SOC via BLE adv |
| Smart Light | nRF52840 | Light state via BLE adv + GATT write |
| ESP32-CAM | ESP32-S3 | PIR-triggered JPEG capture → BeagleBone |
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
5 PIR sensors. On a 0→1 occupied transition the hub fires a UDP `CAPTURE`
packet to the ESP32-CAM. The CAM connects to the BeagleBone inference daemon,
sends a JPEG, and closes the connection.

```
PIR motion event
    → hub BLE scan
    → pir_window_update(slot)
    → drain_queues() detects 0->1 transition
    → send_cam_trigger() UDP CAPTURE → 10.0.0.222:9091
    → ESP32-CAM wakes, captures 320x240 JPEG
    → TCP connect → BeagleBone 10.0.0.206:9090
    → inference_daemon receives JPEG
    → TFLite person detection
    → result saved to /data/pending/
```

The ESP32-CAM connects per-trigger — no persistent TCP connection. A counting
semaphore (max=5) prevents burst capture loss when multiple PIR zones fire
simultaneously.

## Dynamic Reed Discovery

Reed sensors are auto-discovered by BLE name prefix `ReedSensor*`. Adding a
new sensor requires no code changes anywhere in the stack:

1. Flash nRF52840 with `CONFIG_BT_DEVICE_NAME="ReedSensor3"`
2. Power on — ESP32 assigns a slot by MAC address
3. BeagleBone receives new slot in JSON, forwards via UART
4. STM32 receives `REED_COUNT:3`, calls `UI_Reflow(3)` — new tile appears

**Slot state machine:**
```
SLOT_ACTIVE  — advertising within 150s  → green dot
SLOT_OFFLINE — unseen > 150s            → red dot, tile stays
SLOT_EMPTY   — unseen > 3600s           → tile hidden, layout reflows
```

Cooldown table prevents immediate slot re-allocation after removal. Generation
counter increments on device swap — detectable across the full stack.

## BeagleBone Pipeline

Seven-process architecture, each an independent systemd service:

```
esp01-tcp-server  — AT command setup for ESP-01 WiFi module (oneshot)
sensor_server     — UART rx from ESP32, JSON parse, named pipe write
camera_manager    — UDP ingress for camera + doorbell devices, pipe write
data_controller   — pipe read, SQLite write, shared memory update
uart_controller   — UART tx to STM32F411 every 5s
heartbeat         — device online/offline tracking
db_manager        — SQLite persistence
inference_daemon  — TCP server for ESP32-CAM JPEG, TFLite person detection
```

**Service boot order:**
```
network-online.target
    → esp01-tcp-server  (oneshot, configures ESP-01 AT server)
    → data-controller
    → sensor-server     (wrapper polls for named pipe before exec)
    → inference_daemon
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

### ESP32-CAM
```bash
cd esp32-cam
idf.py build flash monitor
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
west build && west flash
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
│   ├── main/                   # aws, ble, tcp, uart, wifi managers
│   │   ├── tcp_manager.c       # BB state machine, drain_queues, g_state
│   │   ├── motor_server.c/h    # TCP server for ESP32-C3 motor
│   │   ├── cam_trigger.c/h     # UDP CAPTURE trigger for ESP32-CAM
│   │   └── pir_window.c/h      # per-slot occupancy sliding window
│   ├── managed_components/
│   ├── tests/
│   ├── CMakeLists.txt
│   ├── partitions.csv
│   └── sdkconfig
│
├── esp32-cam/                  # ESP32-S3-CAM PIR-triggered capture
│   └── main/
│       └── main.c              # UDP trigger rx, JPEG capture, TCP push
│
├── beaglebone/                 # Embedded Linux pipeline
│   ├── include/                # Shared ABI headers (ipc_proto.h, shared_data.h)
│   ├── controller/             # Data controller — event processing pipeline
│   │   ├── pipeline/           # Pipe readers + event dispatchers
│   │   ├── state/              # State registry, SHM updater, DB persistence
│   │   ├── uart/               # UART transport, controller, lock, staging
│   │   └── doorbell/           # Doorbell pending + result reader
│   ├── server/                 # sensor_server, json_parser
│   ├── camera_manager/         # UDP ingress for camera + doorbell devices
│   ├── inference/              # TFLite inference_daemon, detect.tflite
│   └── wifi/                   # ESP-01 AT setup script
│
├── esp32c3/                    # ESP32-C3 targets
│   ├── idf/motor/              # Motor PWM controller (ESP-IDF)
│   └── zephyr/pir/             # PIR sensor x5 (Zephyr)
│
├── nrf52840/                   # Zephyr BLE peripheral nodes
│   ├── reed-sensor/
│   ├── smart-light/
│   └── smart-lock/
│
├── stm32-blackpill/            # STM32F411 LVGL dashboard
│   ├── Core/
│   ├── Drivers/
│   ├── Middlewares/lvgl/
│   └── User/
│
├── stm32-bluepill/             # STM32F103 DHT11 + UART bridge
│   ├── Core/
│   ├── Drivers/
│   └── UserCore/
│
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
| ESP32 Hub | Unity (FetchContent) | gcovr → Cobertura XML |
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

### ESP32-C3 Motor
```bash
cd esp32c3/idf/motor/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_motor
```

### nRF52840 (native_sim — all nodes)
```bash
# Run from any node directory: reed-sensor, smart-lock, smart-light, pir
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
mkdir -p build && cd build
cmake .. && make && ./test_controller
```

### BeagleBone — Server
```bash
cd beaglebone/server/tests/unit
mkdir -p build && cd build
cmake .. && make && ./test_server
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

**Why static PIR slots?** The static slot system gives predictable memory
layout, zero heap fragmentation, and FreeRTOS-friendly fixed allocation.
Adding sensors beyond MAX_PIRS requires a recompile and reflash. See issue
#38 for the dynamic discovery roadmap if sensor count grows.
