# Smart Home IoT System

A fault-tolerant distributed IoT system across 10+ MCUs — sensing, control,
persistence, networking, inference, and display each run as isolated processes
or dedicated hardware nodes for independent failure recovery.

## Quick Navigation

- Architecture → Architecture
- Runtime flows → PIR + Camera + Doorbell Pipelines
- Logging & tracing → Logging & Event Correlation
- System guarantees → System Invariants
- IPC + boot ordering → BeagleBone Pipeline / IPC Reference
- Camera system → Camera Port Reference
- Testing → Testing

## Architecture
![System Architecture](docs/Arch%20diagram3.png)

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
| Security CAM x3 | ESP32-S3 | PIR-triggered JPEG clip → BeagleBone TCP 9090 (zones 0–2: indoor/front/back) |
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

Only PIR zones 0–2 have a camera behind them. `zone` in
`CamTriggerRequest` maps directly to `CAM_SLOT` on the ESP32-CAM side
(zone 0 → CAM_SLOT=0, and so on up to `MAX_CAMS-1`). `sensor_dispatch.c`
fires a trigger for every PIR 0→1 transition across all 5 configured
PIR slots unconditionally — it has no knowledge of camera topology.
`camera_manager` is the single enforcement point: triggers for
`zone >= MAX_CAMS` are dropped and logged
(`[TRIGGER] zone=%u out of range`). PIR slots 3–4 still feed occupancy
data to `SensorData.pir_slots[]` for other consumers (state registry,
UART push to STM32, etc.) — they just don't drive camera capture.

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
| 9091 | TCP (doorbell) | inbound | doorbell_daemon | Doorbell SNAPSHOT JPEG receive |
| 9091 | UDP (security cameras) | outbound | camera_manager | CAPTURE trigger → ESP32-CAM |
| 9093 | TCP | inbound | doorbell_stream_daemon | Doorbell MJPEG stream receive |
| 9094 | UDP | inbound | camera_manager | Heartbeats from all camera devices |

## IPC Reference (local sockets / pipes)

| Path | Type | Direction | Purpose |
|------|------|-----------|---------|
| `/tmp/sensor_pipe` | Named FIFO | sensor-server → data-controller | Sensor frame data |
| `/tmp/cam_pipe` | Named FIFO | camera-manager → data-controller | Camera/doorbell liveness frames |
| `/tmp/cam_trigger.sock` | UNIX datagram | data-controller → camera-manager | PIR-triggered capture requests |

See the chicken-and-egg startup note above — `/tmp/sensor_pipe` and
`/tmp/cam_pipe` are FIFOs requiring `ExecStartPre` ordering on the
writer side; `/tmp/cam_trigger.sock` is a datagram socket with no such
requirement since failed sends do not block.

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
hostapd-ap              — cams_2.4 WiFi AP for camera/doorbell devices
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
thermostat-lcd          — thermostat + sensor readout on I2C LCD
```

**Service boot order:**
```
network-online.target
    → hostapd-ap        (cams_2.4 WiFi AP for camera/doorbell devices;
                         ExecStartPre waits for the wlu1 USB WiFi adapter
                         to enumerate before starting hostapd — same
                         wait-for-resource pattern used for /tmp/cam_pipe
                         and /tmp/sensor_pipe below, applied to a network
                         interface instead of a named pipe)
    → dnsmasq           (generic DNS/DHCP forwarder, base image service —
                         loads /etc/dnsmasq.d/cams.conf, serves DHCP on
                         the camera AP subnet; not the dedicated
                         dnsmasq-ap.service, which exists but is disabled
                         and currently unused)
    → esp01-tcp-server  (oneshot, configures ESP-01 AT server)
    → data-controller
    → camera-manager    (waits for /tmp/cam_pipe before start)
    → sensor-server     (waits for /tmp/sensor_pipe before start)
    → inference
    → doorbell
    → doorbell_stream
    → mediamtx
    → api-server
    → thermostat-lcd
```

**Named pipe (FIFO) startup ordering — chicken-and-egg note:**

Two pipes exist between camera-manager and data-controller, in opposite
directions, with different IPC mechanisms:

- `/tmp/cam_pipe` (camera-manager writes → data-controller reads):
  a named FIFO `open(O_RDONLY)` blocks until a writer opens the other
  end; `open(O_WRONLY|O_NONBLOCK)` returns `ENXIO` if no reader is
  present yet. To avoid deadlock, the reader
  (`cam_pipe_reader_thread` in data-controller) opens with
  `O_RDONLY|O_NONBLOCK` and polls with `poll()`, while
  `camera-manager.service` uses `ExecStartPre` to wait until
  `/tmp/cam_pipe` exists before starting — guaranteeing the reader is
  already past its non-blocking open before the writer's first
  heartbeat. The writer keeps a persistent fd open for the process
  lifetime, reopening only on `EPIPE`/`ENXIO`; `EAGAIN`/`EINTR` drop the
  frame but keep the fd open. The same pattern is used by
  `sensor-server` → `/tmp/sensor_pipe` → `data-controller`.

- `/tmp/cam_trigger.sock` (data-controller writes → camera-manager
  reads): a UNIX datagram socket, not a FIFO. `sendto()` to a socket
  path that does not exist yet fails immediately with `ENOENT` — it
  does not block and does not deadlock. `data-controller.service` has
  no startup dependency on `camera-manager.service` for this reason;
  any PIR-triggered capture events that fire before camera-manager
  has bound the socket are dropped and logged
  (`[DISPATCH] cam_trigger_failed`), not retried.


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

## Logging & Event Correlation

Cross-process log correlation on the BBB uses **two distinct, unrelated
keys** — do not join across them, they answer different questions.

### `event_id` — sensor/camera/inference event correlation

Identifies a single physical event (a PIR trip, a REED state change, a
camera capture, etc.) across every process that touches it. Generated
once per event in `esp32-hub/main/wroom_bus.c` via
`wroom_event_id_generate()` — a `uint64_t` monotonic counter guarded by
a critical section (`taskENTER_CRITICAL`/`taskEXIT_CRITICAL` +
`portMUX_TYPE`), matching the mutex pattern already used in
`ble_light.c`/`ble_lock.c`. Concurrent `bus_publish_*` calls from
different FreeRTOS tasks previously raced on this counter and could
produce duplicate event_ids; the critical section fixes this.

Same underlying value, two textual formats on the BBB side:

| Domain                          | Format                      | Example                     |
|----------------------------------|------------------------------|------------------------------|
| controller (dispatch/SHM/UART)  | decimal, `event_id=%llu`    | `event_id=45512`            |
| camera_manager                  | decimal, `event_id=%llu`    | `event_id=49`               |
| inference / doorbell daemons     | 16-hex-digit, `EVENT_ID_FMT`| `event_id=0000000000000031` |

Convert decimal → hex to join against inference/doorbell logs:
```bash
printf '%016x\n' 49
# 0000000000000031
```

`EVENT_ID_FMT`/`EVENT_ID_ARG` are defined independently (same text) in
`inference_core.h` and `doorbell_result_shm.h` — not centralized; keep
both in sync if the format changes.

Example end-to-end trace (security-cam path):
```
camera_manager.log:  [TRIGGER] sent zone=0 event_id=49 cam_tx_id=1 boot_epoch=... ip=10.0.1.166
inference.log:       [INDOOR] clip_start event_id=0000000000000031 device_id=0
inference.log:       [INDOOR] rx event_id=0000000000000031 frame=1 bytes=17922
inference.log:       [INDOOR] clip_done event_id=0000000000000031 frames=20 duration_ms=10654
inference.log:       [INDOOR] infer_start/infer_done event_id=0000000000000031 frame=1..20
inference.log:       [INDOOR] clip_saved / db_insert event_id=0000000000000031 path=...
```

Controller-domain example (`event_id=8` threaded across six log prefixes):
```
[BLE_LOCK] / [WROOM] / [TCP] / [DISPATCH] event_id=8
[SHM]  transport=sensor_shm write src=pipe_ingress device=LOCK event_id=8
[UART] transport=ttyS1 write dst=blackpill_lcd device=LOCK event_id=8
```

The controller domain used `eid=` instead of `event_id=` until unified
across `uart_controller.c`, `shm_updater.c`, `sensor_dispatch.c`, and
`state_registry.c`. Any `eid=` seen going forward indicates a stale
binary that needs rebuilding.

**`cam_tx_id` / `boot_epoch`** — a related but separate, session-scoped
pair, not a standalone event correlation key. `cam_tx_id` correlates a
UDP trigger to the ESP32-cam's capture within one `camera_manager`
process lifetime only, and resets to 1 on every restart. `boot_epoch`
(process start time) is logged alongside it on every `[TRIGGER] sent`
line so `(boot_epoch, cam_tx_id)` pairs are unique across restarts.
`boot_epoch` is BBB-log-only and is never sent over the wire to the
ESP32-cam.

### `frame_seq` — server telemetry/event correlation

A separate join key used only in the `sensor_server`/telemetry
pipeline (`json_parser.c`, `pipe_writer.c`, `sensor_server.c`). Not
interchangeable with `event_id`.

- `telemetry.log` — high-volume, one line per tick.
- `events.log` — low-volume, one line only on a true state
  transition, written via a dedicated `log_event()` function to its
  own file handle.

`frame_seq` is the documented join key between these two files.
`event_id` can appear as a payload field on individual `events.log`
lines, but `frame_seq` — not `event_id` — is what correlates a line
in `events.log` back to its corresponding tick in `telemetry.log`.

### Trace commands

A helper script (`trace_event.sh`) traces either key across the
relevant logs, handling the decimal↔hex conversion for `event_id`
automatically:
```bash
./trace_event.sh event 49        # camera/controller/inference domains
./trace_event.sh seq   18809     # server telemetry/events domains
```
Run on the BeagleBone — both modes read directly from `/var/log/*.log`.

### Bridging event_id and frame_seq at trigger time

As of 2026-07-04, the security-cam (indoor) trigger pipeline threads
`frame_seq` alongside `event_id` from `sensor_dispatch.c` through
`camera_manager.c`'s wire trigger, the ESP32-CAM header (`cam_header_t`
bumped v1→v2, 20→24 bytes), and into `inference_daemon.c`'s log lines.
This does not merge the two schemes — `event_id` (causal identity) and
`frame_seq` (telemetry snapshot) remain conceptually distinct — but a
camera/inference event can now be joined back to the exact sensor
telemetry tick active at capture time.

**Not yet covered:** `esp32-doorbell`'s `cam_logic.h` shares the same
`cam_header_t` layout and needs the identical change to stay
compatible; tracked separately, not yet applied.

### Known gaps

- `esp32-doorbell` does not yet carry `frame_seq` in its header (see
  above) — the doorbell path is unaffected by the trigger-time bridge.

## System Invariants

Reflects verified source/behavior as of 2026-07-04. These are
assertions to check against, not aspirational design goals — several
of these were false until fixes landed today; see the linked commits.

- `wroom_event_id_generate()` (`wroom_bus.c`) is the single generator
  for PIR/REED/LOCK/LIGHT/TEMP `event_id`s, guarded by a critical
  section (`taskENTER_CRITICAL`/`taskEXIT_CRITICAL` + `portMUX_TYPE`)
  as of `85a2736`. Before that fix, concurrent `bus_publish_*` calls
  from different FreeRTOS tasks could race and produce duplicate
  `event_id`s — confirmed in production (`event_id=9999` assigned to
  both a PIR and a REED event with no restart involved).
- doorbell's `event_id` source is **not** confirmed to go through
  `wroom_event_id_generate()` — `bus_publish_doorbell()` generates and
  discards an `eid` via that function, then uses an externally-passed
  `event_id` parameter instead. Unresolved; treat doorbell `event_id`
  provenance as unverified until this is chased down.
- `event_id` resumes from its last NVS-checkpointed value on hub
  reboot (checkpointed every `WROOM_SEQ_SAVE_EVERY`=50 increments,
  commit `c6e08ba`). A restart can produce a bounded overlap of up to
  ~50 `event_id`s with pre-restart values — this is **not** a full
  reset to 0, and **not** perfectly gapless continuity either.
- `frame_seq` is only a valid join key within the
  `sensor_server`/telemetry pipeline (`events.log` ↔ `telemetry.log`);
  as of `69dde21` it is also threaded through the security-cam
  (indoor) trigger pipeline to bridge with `event_id` at trigger time
  (see above) — doorbell not yet included.
- `cam_tx_id` alone is valid only within one `camera_manager` process
  lifetime (resets to 1 on restart); pair with `boot_epoch` (logged on
  every `[TRIGGER] sent` line) for cross-restart disambiguation.
- `camera_manager` is the only component that resolves camera IPs
  (learned dynamically from UDP heartbeat source addresses); the hub
  is transport-only for camera triggers and has no capture authority.
- UDP camera triggers (`camera_manager` → ESP32-CAM) are fire-and-forget
  with no retry/acknowledgment. As of `f64f5e5`, a dropped trigger
  (zone out of range, camera offline, no IP known) is now reported to
  `data_controller` via a status message, so drops are visible in
  `data_controller`'s log — the trigger itself is still not retried.
- Hub reboot is detected on the BBB via a `boot_marker` field sent on
  every `telemetry{}` tick (0 on the first frame after hub boot, 1
  thereafter; commit `8725a4f`) — a direct signal, not inferred from
  sensor-state symptoms. Old hub firmware without this field defaults
  the BBB's interpretation to `1` (no false reboot detection).
- `/tmp/cam_pipe` and `/tmp/sensor_pipe` are named FIFOs requiring
  writer/reader coexistence at startup (see chicken-and-egg note under
  BeagleBone Pipeline); `/tmp/cam_trigger.sock` is a UNIX datagram
  socket and does not have this requirement.

## Crash Logging

All nodes persist crash state across hard power loss:

| Node | Storage | Data |
|------|---------|------|
| nRF52840 | Zephyr `flash_area` circular log | PC/LR on HardFault |
| ESP32 | NVS + coredump | Boot reason, reset cause |
| STM32F411 | RTC backup registers | PC/LR, fault type |
| STM32F103 | FRAM (16KB partition) | PC/LR, fault type, magic 0xA5 |

## Hardware-Emulation Validation Node

`tiny85-dht-sensor/` (standalone subproject) is a from-scratch software
implementation of the DHT11 single-wire protocol on an ATtiny85, used as a
compatibility test node against the BluePill's DHT11 driver. It occupies
PA7 alongside three physical DHT11 sensors on PA4–PA6; the STM32 driver
issues the same trigger/read sequence to all four and does not distinguish
the emulated node from real hardware at the driver interface level.

**Design constraint.** The engineering problem is bus timing, not sensor
data. A physical DHT11 begins its response within roughly 20–40µs of the
host releasing the bus; the BluePill driver's timeout on that window is on
the order of 1ms. This is a tight timing window imposed by the DHT11
protocol and the driver's polling behavior, not a formally specified
hardware guarantee — it had to be met in software.

**Implementation choices.** Landing a bit-banged response inside that
window required:

- caching sensor data ahead of the trigger, refreshed off the response
  path rather than read synchronously on demand
- excluding all non-deterministic or slow operations (ADC, UART, logging)
  from the response function itself — only pin writes and `_delay_us()`
  are permitted between trigger and response
- matching DHT11's specific byte encoding (not DHT22's), since the two
  protocols are timing-compatible but not encoding-compatible, and the
  host driver validates against DHT11's checksum format

**Observed failure modes.** Early revisions blew this budget by one to two
orders of magnitude — ADC conversion (>20ms) and UART logging (~31ms/line)
both ran ahead of the response and caused the STM32 driver to time out
before the pin ever moved. See
`tiny85-dht-sensor/src/tiny85-dht-sensor.c` for the full record of
timing-violation bugs found and fixed during bring-up.

This validates an implicit assumption in the BluePill driver: that DHT11
responses behave like a hardware-bound contract rather than a
timing-sensitive software protocol — true for this driver's ~1ms tolerance
and this implementation path, not a universal property of the DHT11 protocol.

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

### ESP32 Doorbell CAM (AI-Thinker ESP32)
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
│   ├── idf/motor/               # Motor PWM controller (ESP-IDF)
│   └── zephyr/pir/              # PIR sensor x5 (Zephyr)
│
├── nrf52840/                    # Zephyr BLE peripheral nodes
│   ├── reed-sensor/            # Door/window reed sensor (up to 6)
│   ├── temp-sensor/            # BLE temperature sensor (TempSensor1/2)
│   ├── smart-light/
│   └── smart-lock/
│
├── stm32-blackpill/             # STM32F411 LVGL dashboard
├── stm32-bluepill/               # STM32F103 DHT11 + UART bridge
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

### Hardware-in-the-Loop Validation

#### ATtiny85 DHT11 Protocol Emulator

The following section documents a hardware-in-the-loop validation case that informed driver-level timing assumptions.

*This component is not part of the runtime system architecture; it exists
solely for validation of driver timing assumptions.*

`tiny85-dht-sensor/` (standalone repo) is a from-scratch software
implementation of the DHT11 single-wire protocol on an ATtiny85, used as a
compatibility test node against the BluePill's DHT11 driver. It occupies
PA7 alongside three physical DHT11 sensors on PA4–PA6; the STM32 driver
issues the same trigger/read sequence to all four and does not distinguish
the emulated node from real hardware at the driver interface level.

**Design constraint.** The engineering problem is bus timing, not sensor
data. A physical DHT11 begins its response within roughly 20–40µs of the
host releasing the bus; the BluePill driver's timeout on that window is on
the order of 1ms. This is a tight timing window imposed by the DHT11
protocol and the driver's polling behavior, not a formally specified
hardware guarantee — it had to be met in software.

**Implementation choices.** Landing a bit-banged response inside that
window required:

- caching sensor data ahead of the trigger, refreshed off the response
  path rather than read synchronously on demand
- excluding all non-deterministic or slow operations (ADC, UART, logging)
  from the response function itself — only pin writes and `_delay_us()`
  are permitted between trigger and response
- matching DHT11's specific byte encoding (not DHT22's), since the two
  protocols are timing-compatible but not encoding-compatible, and the
  host driver validates against DHT11's checksum format

**Observed failure modes.** Early revisions blew this budget by one to two
orders of magnitude — ADC conversion (>20ms) and UART logging (~31ms/line)
both ran ahead of the response and caused the STM32 driver to time out
before the pin ever moved. See the `tiny85-dht-sensor` repo for the full
record of timing-violation bugs found and fixed during bring-up.

This validates an implicit assumption in the BluePill driver: that DHT11
responses behave like a hardware-bound contract rather than a
timing-sensitive software protocol — true for this driver's ~1ms tolerance
and this implementation path, not a universal property of the DHT11 protocol.

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

**Why does trigger authority for camera capture live on the BeagleBone
instead of the hub?** The hub is a pure BLE/PIR transport node — it
scans, computes occupancy, and forwards events; it makes no capture
decisions itself. Trigger authority belongs to the BeagleBone for two
compounding reasons. First, ownership: the BeagleBone owns the
inference pipeline, the camera heartbeat registry, and occupancy
policy, so keeping dispatch in `camera_manager` puts IP learning,
liveness, rate limiting, and CAPTURE dispatch in one place, with no
duplicated state on the hub. Second, visibility: camera and doorbell
devices live on a BeagleBone-hosted Wi-Fi AP (`cams_2.4`) that the hub
has no access to at all — it cannot resolve camera IPs, liveness, or
reachability on that subnet, so it is not even capable of dispatching
a trigger even if it wanted to. `camera_manager` is the only process
positioned to verify a camera is online and send it a CAPTURE packet.

**Why dynamic IP learning for cameras?** The `camera_manager` learns each
camera's IP from the UDP heartbeat source address (`recvfrom`). No static IP
configuration is required, and the registry updates automatically if DHCP
reassigns an address.

**Why static PIR slots?** The static slot system gives predictable memory
layout, zero heap fragmentation, and FreeRTOS-friendly fixed allocation.
Adding sensors beyond MAX_PIRS requires a recompile and reflash. See issue
#38 for the dynamic discovery roadmap if sensor count grows.

**Why two different IPC mechanisms between camera-manager and
data-controller?** The two directions have different timing and
reliability requirements. Liveness data (`/tmp/cam_pipe`,
camera-manager → data-controller) is continuous and order-sensitive —
a named FIFO with a persistent writer fd gives ordered delivery and
natural backpressure. PIR-triggered capture requests
(`/tmp/cam_trigger.sock`, data-controller → camera-manager) are rare,
latency-sensitive, fire-and-forget events where blocking on a missing
reader would stall the entire sensor dispatch pipeline — a UNIX
datagram socket fails fast with `ENOENT` instead of blocking, so a
camera-manager restart never stalls sensor processing.

**Why separate SNAPSHOT and STREAM modes for the doorbell?** The two use
cases have different requirements. SNAPSHOT delivers a single high-quality
JPEG per button press with full inference and event correlation. STREAM
delivers continuous low-latency MJPEG for live viewing via RTSP. Combining
them in one firmware path would require runtime mode switching and complicate
the TCP framing. Compile-time selection keeps each path simple and testable.
