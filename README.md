# Smart Home IoT System

A fault-tolerant distributed IoT system across 8 MCUs — sensing, control, persistence, networking, and display each run as isolated processes or dedicated hardware nodes for independent failure recovery.

## Architecture

```
nRF52840 x4 (Zephyr RTOS)
  Reed Sensors ──┐
  PIR Motion ────┤  BLE advertisements
  Smart Lock ────┤  (manufacturer data)
  Smart Light ───┘
                 │
                 ▼
          ESP32 Hub (ESP-IDF)
          ├── BLE scanner + slot manager
          ├── UART rx ← STM32F103 avg_temp
          ├── TCP → BeagleBone Black
          ├── TCP → ESP32-C3 motor
          └── UART tx → BeagleBone Black
                 │
        ┌────────┴────────┐
        ▼                 ▼
BeagleBone Black     ESP32-C3 Motor
(Embedded Linux)     Controller
├── sensor_server    └── PWM + DHT11
├── data_controller
├── SQLite DB
└── UART tx → STM32F411
                 │
                 ▼
        STM32F411 BlackPill
        LVGL v8 Dashboard
        ILI9341 240x320
```

## Hardware

| Node | MCU | Role |
|------|-----|------|
| BLE Hub | ESP32 | BLE scan, TCP forwarding, UART bridge |
| Dashboard | STM32F411 BlackPill | LVGL display, ILI9341 240x320 |
| Temp/UART | STM32F103 Blue Pill | DHT11 avg_temp → UART to ESP32 |
| Motor | ESP32-C3 | PWM motor control, TCP server |
| Reed x2 | nRF52840 | Door state + battery SOC via BLE adv |
| PIR | nRF52840 | Motion count + battery SOC via BLE adv |
| Smart Lock | nRF52840 | Lock state + battery SOC via BLE adv |
| Smart Light | nRF52840 | Light state via BLE adv + GATT write |

## BLE Protocol

All nRF52840 nodes broadcast state passively in manufacturer advertisement data — zero connection overhead for monitoring. The ESP32 hub tracks device age and battery from scan data alone.

| Device | Company ID | Payload |
|--------|-----------|---------|
| Reed Sensors | 0xAB | `[state, batt_soc]` |
| Smart Lock | 0xAC | `[state, batt_soc]` |
| Smart Light | 0xAD | `[state]` |
| PIR | 0xFF 0xFF | `[count BE 4 bytes, batt_soc]` |

GATT connections established only for write commands (lock/light control). Device disconnects immediately after write confirmation.

## Dynamic Reed Discovery

Reed sensors are auto-discovered by BLE name prefix `ReedSensor*`. Adding a new sensor requires no code changes anywhere in the stack:

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

Cooldown table prevents immediate slot re-allocation after removal. Generation counter increments on device swap — detectable across the full stack.

## BeagleBone Pipeline

Six-process architecture, each an independent systemd service:

```
sensor_server   — UART rx from ESP32, JSON parse, named pipe write
data_controller — pipe read, SQLite write, shared memory update
uart_controller — UART tx to STM32F411 every 5s
heartbeat       — device online/offline tracking
cmd_handler     — IPC command dispatch
db_manager      — SQLite persistence
```

UART push format to STM32:
```
STATE:tmp,pir,lgt,lck,age_pir,age_lgt,age_lck,reed_count
REED_COUNT:n
DR1:state,batt,age
DR2:state,batt,age
PIR:count,batt
LGT:state
LCK:state,batt
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

### BeagleBone
```bash
cd beaglebone/controller
gcc data_controller.c commands.c db_manager.c heartbeat.c \
    uart_controller.c cmd_handler.c \
    -o data_controller -lpthread -lrt -lsqlite3

cd ../server
gcc sensor_server.c -o sensor_server -ljson-c

sudo systemctl restart data-controller sensor-server
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
cd esp32-c3-motor
idf.py build flash monitor
```

## Project Structure

```
smart-home-iot/
├── esp32-hub/          ESP-IDF BLE hub + TCP forwarder
├── beaglebone/         Embedded Linux pipeline (6 services)
│   ├── controller/     data_controller, cmd_handler, uart_controller
│   └── server/         sensor_server
├── stm32-blackpill/    STM32F411 LVGL dashboard
├── stm32-bluepill/     STM32F103 DHT11 + UART bridge
├── nrf52840/           Zephyr peripheral nodes
│   ├── reed-sensor/
│   ├── pir/
│   ├── smart-lock/
│   └── smart-light/
└── esp32-c3-motor/     Motor PWM controller
```

## Key Design Decisions

**Why scan-based BLE?** Passive advertisement monitoring gives the hub continuous device age and state with zero connection overhead. Battery life on coin/AA nodes is measured in months.

**Why BeagleBone?** AM335x gives a full Linux environment for SQLite, systemd service isolation, and future HTTP/AWS expansion without the constraints of an RTOS.

**Why FRAM for crash logging on STM32F103?** Unlimited write endurance and byte-addressable writes survive hard power loss. No wear leveling needed, no erase cycles, no filesystem overhead.

**Why dedicated ESP32-C3 for motor?** Oscilloscope analysis confirmed DHT11 timing violations under shared STM32 interrupt load. Dedicated silicon eliminates the constraint entirely.
