# Unit Test Stubs

This directory contains the stub and mock infrastructure for the reed sensor
unit test suite. The suite runs on `native_sim` (host Linux) with no real
hardware or Zephyr RTOS kernel — every external dependency is either stubbed
or FFF-mocked.

## Overview

| Type | Count | Location |
|------|-------|----------|
| FFF mocks | 9 | `src/test_reed_main.c` |
| Stub files | 6 | `tests/unit/stubs/` |
| **Total tests** | **50** | across 9 suites |

---

## Stub Files

### `stub_device.c` + `stub_overrides.h`

Replaces the Zephyr device model for `battery.c`.

`battery.c` calls `DEVICE_DT_GET_ANY(maxim_max17048)` to obtain the fuel
gauge device pointer, then calls `device_is_ready()` on it. On `native_sim`
there is no devicetree and no real driver, so these are replaced:

- `DEVICE_DT_GET_ANY` is redefined (via `stub_overrides.h`) to return
  `g_stub_device_ptr`, a statically allocated `struct device`.
- `stub_set_device_ready(bool)` switches the device state between ready and
  not-ready by swapping the `device_state` pointer inside the stub device.
  This lets tests simulate MAX17048 present vs absent on the I2C bus.
- `batt_before_null` calls `stub_set_device_ready(false)` then
  `battery_init()` to force `g_fg = NULL` through the driver's own
  `device_is_ready()` failure path — no internal state is exposed.

### `zephyr/bluetooth/bluetooth.h`

Replaces the Zephyr BLE stack headers for `ble_adv.c`.

Provides the minimal types and macros `ble_adv.c` needs to compile:

- `struct bt_data` (type, data_len, data pointer)
- `struct bt_le_adv_param`
- `BT_DATA`, `BT_DATA_BYTES`, `BT_DATA_FLAGS`, `BT_DATA_MANUFACTURER_DATA`,
  `BT_DATA_NAME_COMPLETE` macros
- `BT_LE_ADV_OPT_*` constants
- Declarations for `bt_enable`, `bt_le_adv_start`, `bt_le_adv_update_data`,
  `bt_le_adv_stop` — the latter two are FFF-mocked in the test file.

### `zephyr/bluetooth/hci.h`

Empty stub. `ble_adv.c` includes this transitively for HCI event types, none
of which are needed for the functions under test.

### `zephyr/drivers/fuel_gauge.h`

Replaces the Zephyr fuel gauge driver API for `battery.c`.

Provides:
- `enum fuel_gauge_property` (`FUEL_GAUGE_VOLTAGE`,
  `FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE`)
- `union fuel_gauge_prop_val` (voltage in uV, relative_state_of_charge)
- Declaration for `fuel_gauge_get_prop` — FFF-mocked in the test file.

### `zephyr/nrf.h`

Replaces Nordic nRF register definitions.

`main.c` reads and clears `NRF_POWER->RESETREAS` at startup.
`trinity_fault.c` reads SCB registers. Neither file is compiled into the
unit test build, but the header is pulled in transitively. This stub provides
the minimal `NRF_POWER_Type` struct, the `NRF_POWER` macro, and an empty
`NVIC_SystemReset()` so the build is clean.

---

## FFF Mocks

All mocks are declared in `src/test_reed_main.c` using the FFF framework
(`FAKE_VOID_FUNC` / `FAKE_VALUE_FUNC`). They are reset before every test
via `reset_all()`.

| Mock | Replaces | Notes |
|------|----------|-------|
| `trinity_log_event` | `trinity_flash.c` | Has a custom capture fake (`fake_trinity_log_event_capture`) that copies the message into `g_log_event_buf` before the stack frame is released. Tests read `g_log_event_buf`, not `arg0_val`. |
| `trinity_wdt_kick` | `trinity_wdt.c` | Void, no return value needed. |
| `trinity_wdt_init` | `trinity_wdt.c` | Void. |
| `trinity_log_init` | `trinity_log.c` | Returns `int`; default return 0. |
| `trinity_log_dump_previous` | `trinity_log.c` | Void. |
| `trinity_log_dump_previous_deferred` | `trinity_log.c` | Void. |
| `fuel_gauge_get_prop` | Zephyr fuel gauge API | Two custom fakes: `fuel_gauge_inject_voltage` (sets `val.voltage` in uV) and `fuel_gauge_inject_soc` (sets `val.relative_state_of_charge`). Used by voltage and SOC path tests respectively. |
| `bt_le_adv_update_data` | Zephyr BLE API | Captures `arg0_val` (the `bt_data` array pointer). The payload test walks this array to find the manufacturer data entry and reads `data[2]` (the battery SOC byte) directly. |
| `bt_le_adv_stop` | Zephyr BLE API | Asserted to never be called — the stop/start race condition regression. |

---

## Suite Structure

| Suite | Before-hook | What it tests |
|-------|-------------|---------------|
| `reed_wdt` | `reset_all` | Compile-time WDT timing constants |
| `reed_mfg` | `reset_all` | BLE manufacturer data layout constants |
| `reed_batt_mv_to_soc` | `reset_all` | `mv_to_soc()` pure arithmetic, no mocks |
| `reed_batt_fuel_gauge` | `batt_before` | `battery_init/read_mv/read_soc/print_status` happy and error paths |
| `reed_batt_fuel_gauge_null` | `batt_before_null` | Same functions with `g_fg == NULL` |
| `reed_ble` | `reset_all` | `ble_broadcast()`, `ble_adv_set_batt()`, payload byte |
| `reed_resetreas` | `resetreas_before` | `trinity_classify_reset()` all 7 paths, `trinity_log_boot_reason()` all labels |
| `reed_canary` | `reset_all` | `trinity_canary_set_booted()` brownout detection |
| `reed_log_stack` | `reset_all` | Kconfig log thread stack size guard |

### Before-hook design

- **`reset_all`** — resets all FFF fakes, restores stub device to ready,
  clears noinit sentinels. Used by suites that need no driver initialisation.
- **`batt_before`** — calls `reset_all`, wires the log capture custom fake,
  calls `battery_init()`. Tests in `reed_batt_fuel_gauge` start with a live
  driver and focus only on the behaviour under test.
- **`batt_before_null`** — calls `reset_all`, marks device not-ready, calls
  `battery_init()` (which sets `g_fg = NULL` via the driver's own failure
  path), then resets fakes so null-guard tests start with a clean call count.
- **`resetreas_before`** — calls `reset_all`, wires the log capture custom
  fake. Identical to `batt_before` minus `battery_init()`.
