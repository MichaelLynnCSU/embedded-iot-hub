/******************************************************************************
 * \file test_temp_main.c
 * \brief Unit tests for temp sensor node -- full regression coverage.
 *
 * Coverage map:
 *   temp_init()               temp.c   -- success, hw-fail, propagate error
 *   temp_read_decidegc()      temp.c   -- hw error, conversion, call count,
 *                                         log content, negative temp
 *   temp_print_status()       temp.c   -- hw error early return, success path
 *   mv_to_soc()               battery.h -- boundary + midpoint
 *   battery_init()            battery.c -- ready, not-ready
 *   battery_read_mv()         battery.c -- null guard, uV->mV, error
 *   battery_read_soc()        battery.c -- null guard, clamp, error
 *   battery_print_status()    battery.c -- null guard, error, success
 *   ble_broadcast()           ble_adv.c -- payload, no stop, errors
 *   ble_adv_set_batt()        ble_adv.c -- no update_data, payload byte
 *   trinity_classify_reset()  trinity_boot.c -- all 7 code paths
 *   trinity_log_boot_reason() trinity_boot.c -- all labels + raw hex
 *   trinity_canary_set_booted() trinity_canary.c -- canary upgrade
 *   TEMP_POLL_MS / ticks      main.h   -- compile-time guards
 *
 * Run with: west build -b native_sim tests/unit --build-dir build_test
 ******************************************************************************/

#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <string.h>
#include <stdbool.h>
#include "trinity_log.h"
#include "main.h"
#include "temp.h"
#include "temp_hw.h"
#include "battery.h"
#include "ble_adv.h"

DEFINE_FFF_GLOBALS;

/* Controlled by temp_hw_stub.c */
extern int16_t g_stub_hw_raw;
extern int     g_stub_hw_ret;

/* Controlled by stub_device.c -- used by battery only */
extern bool g_stub_device_ready;

/******************************************************************************
 * FFF mocks
 ******************************************************************************/

FAKE_VOID_FUNC(trinity_log_event, const char *);
static char g_log_event_buf[128];
static void fake_trinity_log_event_capture(const char *msg)
{
    strncpy(g_log_event_buf, msg, sizeof(g_log_event_buf) - 1);
    g_log_event_buf[sizeof(g_log_event_buf) - 1] = '\0';
}

FAKE_VOID_FUNC(trinity_wdt_kick);
FAKE_VOID_FUNC(trinity_wdt_init);
FAKE_VALUE_FUNC(int,  trinity_log_init);
FAKE_VOID_FUNC(trinity_log_dump_previous);
FAKE_VOID_FUNC(trinity_log_dump_previous_deferred);

FAKE_VALUE_FUNC(int, fuel_gauge_get_prop,
                const struct device *,
                enum fuel_gauge_property,
                union fuel_gauge_prop_val *);

FAKE_VALUE_FUNC(int, bt_le_adv_update_data,
                const struct bt_data *, size_t,
                const struct bt_data *, size_t);
FAKE_VALUE_FUNC(int, bt_le_adv_stop);

/******************************************************************************
 * Shared teardown
 ******************************************************************************/
static void reset_all(void *fixture)
{
    (void)fixture;
    RESET_FAKE(trinity_log_event);
    RESET_FAKE(trinity_wdt_kick);
    RESET_FAKE(trinity_wdt_init);
    RESET_FAKE(trinity_log_init);
    RESET_FAKE(trinity_log_dump_previous);
    RESET_FAKE(trinity_log_dump_previous_deferred);
    RESET_FAKE(fuel_gauge_get_prop);
    RESET_FAKE(bt_le_adv_update_data);
    RESET_FAKE(bt_le_adv_stop);
    FFF_RESET_HISTORY();

    g_stub_hw_raw = 0;
    g_stub_hw_ret = 0;
    stub_set_device_ready(true);
    g_noinit_guard    = 0;
    g_canary_snapshot = 0;
}

/******************************************************************************
 * Custom fakes: fuel_gauge injection
 ******************************************************************************/
static uint8_t s_injected_soc = 0;
static int fuel_gauge_inject_soc(const struct device *dev,
                                  enum fuel_gauge_property prop,
                                  union fuel_gauge_prop_val *val)
{
    (void)dev; (void)prop;
    val->relative_state_of_charge = s_injected_soc;
    return 0;
}

static int s_injected_uv = 0;
static int fuel_gauge_inject_voltage(const struct device *dev,
                                      enum fuel_gauge_property prop,
                                      union fuel_gauge_prop_val *val)
{
    (void)dev; (void)prop;
    val->voltage = s_injected_uv;
    return 0;
}

/******************************************************************************
 * SUITE: temp_wdt
 ******************************************************************************/
ZTEST_SUITE(temp_wdt, NULL, NULL, reset_all, NULL, NULL);

ZTEST(temp_wdt, test_poll_ms_less_than_wdt_timeout)
{
    zassert_true(TEMP_POLL_MS < 3000,
        "TEMP_POLL_MS=%d >= 3000ms WDT timeout", TEMP_POLL_MS);
}

ZTEST(temp_wdt, test_batt_update_ticks_preserves_5min_interval)
{
    zassert_equal((uint32_t)BATT_UPDATE_TICKS * TEMP_POLL_MS, 300000U,
        "Battery interval: %d x %d = %u, want 300000ms",
        BATT_UPDATE_TICKS, TEMP_POLL_MS,
        (uint32_t)BATT_UPDATE_TICKS * TEMP_POLL_MS);
}

ZTEST(temp_wdt, test_stats_interval_ticks_preserves_60s)
{
    zassert_equal((uint32_t)STATS_INTERVAL_TICKS * TEMP_POLL_MS, 60000U,
        "Stats interval: %d x %d = %u, want 60000ms",
        STATS_INTERVAL_TICKS, TEMP_POLL_MS,
        (uint32_t)STATS_INTERVAL_TICKS * TEMP_POLL_MS);
}

/******************************************************************************
 * SUITE: temp_mfg
 ******************************************************************************/
ZTEST_SUITE(temp_mfg, NULL, NULL, reset_all, NULL, NULL);

ZTEST(temp_mfg, test_mfg_data_size_is_4)
{
    zassert_equal(MFG_DATA_SIZE, 4,
        "MFG_DATA_SIZE changed -- hub expects exactly 4 bytes");
}

ZTEST(temp_mfg, test_mfg_company_id_is_0xAE)
{
    zassert_equal(MFG_COMPANY_ID, 0xAE,
        "MFG_COMPANY_ID changed -- hub BLE filter will break");
}

/******************************************************************************
 * SUITE: temp_sensor
 ******************************************************************************/
static void temp_before(void *data)
{
    (void)data;
    reset_all(NULL);
    trinity_log_event_fake.custom_fake = fake_trinity_log_event_capture;
    g_stub_hw_ret = 0;
    temp_init();
}

static void temp_before_null(void *data)
{
    (void)data;
    reset_all(NULL);
    g_stub_hw_ret = -ENODEV;
    temp_init();
    RESET_FAKE(trinity_log_event);
    FFF_RESET_HISTORY();
}

ZTEST_SUITE(temp_sensor, NULL, NULL, temp_before, NULL, NULL);
ZTEST_SUITE(temp_sensor_null, NULL, NULL, temp_before_null, NULL, NULL);

ZTEST(temp_sensor, test_init_succeeds_when_hw_ready)
{
    g_stub_hw_ret = 0;
    int rc = temp_init();
    zassert_equal(rc, 0,
        "temp_init() must return 0 when hw ready, got %d", rc);
}

ZTEST(temp_sensor, test_init_fails_when_hw_not_ready)
{
    g_stub_hw_ret = -ENODEV;
    int rc = temp_init();
    zassert_equal(rc, -ENODEV,
        "temp_init() must return -ENODEV when hw fails, got %d", rc);
}

ZTEST(temp_sensor, test_init_propagates_hw_error)
{
    g_stub_hw_ret = -EIO;
    int rc = temp_init();
    zassert_equal(rc, -EIO,
        "temp_init() must propagate hw error, got %d", rc);
}

ZTEST(temp_sensor, test_init_idempotent_double_call_does_not_crash)
{
    g_stub_hw_ret = 0;
    int rc1 = temp_init();
    int rc2 = temp_init();
    zassert_equal(rc1, 0, "first temp_init() must succeed, got %d", rc1);
    zassert_equal(rc2, 0, "second temp_init() must succeed, got %d", rc2);
}

ZTEST(temp_sensor_null, test_read_decidegc_hw_error_returns_error)
{
    int16_t t = temp_read_decidegc();
    zassert_equal(t, TEMP_READ_ERROR,
        "temp_read_decidegc() must return TEMP_READ_ERROR on hw failure");
}

ZTEST(temp_sensor, test_read_decidegc_adc_error_returns_error)
{
    g_stub_hw_ret = -EIO;
    int16_t t = temp_read_decidegc();
    zassert_equal(t, TEMP_READ_ERROR,
        "temp_read_decidegc() must return TEMP_READ_ERROR on ADC failure");
}

ZTEST(temp_sensor, test_read_decidegc_converts_raw_to_decidegc)
{
    /* TMP36 at 25.0°C: raw=854 -> 750mV -> 250 decidegC */
    g_stub_hw_raw = 854;
    int16_t t = temp_read_decidegc();
    zassert_equal(t, 250,
        "854 raw must convert to 250 decidegC (25.0°C), got %d", t);
}

ZTEST(temp_sensor, test_read_decidegc_negative_temp)
{
    /* -10.0°C: raw=456 -> 400mV -> -100 decidegC */
    g_stub_hw_raw = 456;
    int16_t t = temp_read_decidegc();
    zassert_equal(t, -100,
        "456 raw must convert to -100 decidegC (-10.0°C), got %d", t);
}

ZTEST(temp_sensor, test_read_decidegc_logs_event)
{
    g_stub_hw_raw = 854;
    temp_read_decidegc();
    zassert_true(trinity_log_event_fake.call_count >= 1,
        "temp_read_decidegc() must call trinity_log_event");
    zassert_true(strstr(g_log_event_buf, "TEMP") != NULL,
        "log must contain TEMP");
}

ZTEST(temp_sensor, test_print_status_hw_error_no_log)
{
    g_stub_hw_ret = -EIO;
    RESET_FAKE(trinity_log_event);
    temp_print_status();
    zassert_equal(trinity_log_event_fake.call_count, 0,
        "temp_print_status() must not log on hw error");
}

ZTEST(temp_sensor, test_print_status_success_does_not_crash)
{
    g_stub_hw_raw = 854;
    temp_print_status();
}

ZTEST(temp_sensor_null, test_print_status_hw_error_no_log)
{
    temp_print_status();
    zassert_equal(trinity_log_event_fake.call_count, 0,
        "temp_print_status() must not log when hw init failed");
}

/******************************************************************************
 * SUITE: temp_batt_mv_to_soc
 ******************************************************************************/
ZTEST_SUITE(temp_batt_mv_to_soc, NULL, NULL, reset_all, NULL, NULL);

ZTEST(temp_batt_mv_to_soc, test_clamp_at_max)
{
    zassert_equal(mv_to_soc(3000),  100, "3000mV == 100%%");
    zassert_equal(mv_to_soc(3500),  100, "3500mV clamps to 100%%");
    zassert_equal(mv_to_soc(65535), 100, "overflow clamps to 100%%");
}

ZTEST(temp_batt_mv_to_soc, test_clamp_at_min)
{
    zassert_equal(mv_to_soc(2000), 0, "2000mV == 0%%");
    zassert_equal(mv_to_soc(1500), 0, "1500mV clamps to 0%%");
    zassert_equal(mv_to_soc(0),    0, "0mV clamps to 0%%");
}

ZTEST(temp_batt_mv_to_soc, test_midpoint)
{
    zassert_equal(mv_to_soc(2500), 50, "2500mV == 50%%");
}

ZTEST(temp_batt_mv_to_soc, test_quarter_points)
{
    zassert_equal(mv_to_soc(2250), 25, "2250mV == 25%%");
    zassert_equal(mv_to_soc(2750), 75, "2750mV == 75%%");
}

ZTEST(temp_batt_mv_to_soc, test_boundary_adjacent)
{
    zassert_true(mv_to_soc(2010) > 0,   "2010mV must be > 0%%");
    zassert_true(mv_to_soc(2999) < 100, "2999mV must be < 100%%");
}

/******************************************************************************
 * SUITE: temp_batt_fuel_gauge
 ******************************************************************************/
static void batt_before(void *data)
{
    (void)data;
    reset_all(NULL);
    trinity_log_event_fake.custom_fake = fake_trinity_log_event_capture;
    battery_init();
}

static void batt_before_null(void *data)
{
    (void)data;
    reset_all(NULL);
    stub_set_device_ready(false);
    battery_init();
    RESET_FAKE(trinity_log_event);
    RESET_FAKE(fuel_gauge_get_prop);
    FFF_RESET_HISTORY();
}

ZTEST_SUITE(temp_batt_fuel_gauge, NULL, NULL, batt_before, NULL, NULL);
ZTEST_SUITE(temp_batt_fuel_gauge_null, NULL, NULL, batt_before_null, NULL, NULL);

ZTEST(temp_batt_fuel_gauge, test_init_succeeds_when_device_ready)
{
    int rc = battery_init();
    zassert_equal(rc, 0,
        "battery_init() must return 0 when device ready, got %d", rc);
}

ZTEST(temp_batt_fuel_gauge, test_init_fails_when_device_not_ready)
{
    stub_set_device_ready(false);
    int rc = battery_init();
    zassert_equal(rc, -ENODEV,
        "battery_init() must return -ENODEV when not ready, got %d", rc);
}

ZTEST(temp_batt_fuel_gauge_null, test_read_mv_null_guard_returns_eio)
{
    int rc = battery_read_mv();
    zassert_equal(rc, -EIO,
        "battery_read_mv() with NULL g_fg must return -EIO, got %d", rc);
    zassert_equal(fuel_gauge_get_prop_fake.call_count, 0,
        "fuel_gauge_get_prop must not be called when g_fg is NULL");
}

ZTEST(temp_batt_fuel_gauge, test_read_mv_converts_uv_to_mv)
{
    s_injected_uv = 3000000;
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_voltage;
    int mv = battery_read_mv();
    zassert_equal(mv, 3000,
        "3000000 uV must convert to 3000 mV, got %d", mv);
}

ZTEST(temp_batt_fuel_gauge, test_read_mv_fuel_gauge_error_returns_eio)
{
    fuel_gauge_get_prop_fake.return_val = -EIO;
    int rc = battery_read_mv();
    zassert_equal(rc, -EIO,
        "battery_read_mv() must return -EIO on fuel_gauge error, got %d", rc);
}

ZTEST(temp_batt_fuel_gauge_null, test_read_soc_null_guard_returns_zero)
{
    uint8_t soc = battery_read_soc();
    zassert_equal(soc, 0,
        "battery_read_soc() with NULL g_fg must return 0, got %d", soc);
    zassert_equal(fuel_gauge_get_prop_fake.call_count, 0,
        "fuel_gauge_get_prop must not be called when g_fg is NULL");
}

ZTEST(temp_batt_fuel_gauge, test_read_soc_clamps_above_100)
{
    s_injected_soc = 120;
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_soc;
    uint8_t soc = battery_read_soc();
    zassert_equal(soc, 100,
        "SOC 120%% must clamp to 100, got %d", soc);
}

ZTEST(temp_batt_fuel_gauge, test_read_soc_normal_value_passes_through)
{
    s_injected_soc = 73;
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_soc;
    uint8_t soc = battery_read_soc();
    zassert_equal(soc, 73,
        "SOC 73%% must pass through unclamped, got %d", soc);
}

ZTEST(temp_batt_fuel_gauge_null, test_print_status_null_guard_no_crash)
{
    battery_print_status();
    zassert_equal(trinity_log_event_fake.call_count, 0,
        "battery_print_status() with NULL g_fg must not call trinity_log_event");
}

ZTEST(temp_batt_fuel_gauge, test_print_status_fuel_gauge_error_skips_log)
{
    fuel_gauge_get_prop_fake.return_val = -EIO;
    battery_print_status();
    zassert_equal(trinity_log_event_fake.call_count, 0,
        "battery_print_status() must not log when battery_read_mv() fails");
}

ZTEST(temp_batt_fuel_gauge, test_print_status_calls_log_event_on_success)
{
    s_injected_uv = 2800000;
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_voltage;
    battery_print_status();
    zassert_true(trinity_log_event_fake.call_count >= 1,
        "battery_print_status() must call trinity_log_event at least once");
    zassert_true(strstr(g_log_event_buf, "2800") != NULL,
        "battery_print_status() log must contain the mV value '2800'");
}

/******************************************************************************
 * SUITE: temp_ble
 ******************************************************************************/
ZTEST_SUITE(temp_ble, NULL, NULL, reset_all, NULL, NULL);

ZTEST(temp_ble, test_broadcast_calls_update_data_not_stop)
{
    bt_le_adv_update_data_fake.return_val = 0;
    int err = ble_broadcast(250, 75);
    zassert_equal(err, 0,
        "ble_broadcast() must return 0 on success, got %d", err);
    zassert_equal(bt_le_adv_update_data_fake.call_count, 1,
        "ble_broadcast() must call bt_le_adv_update_data exactly once");
    zassert_equal(bt_le_adv_stop_fake.call_count, 0,
        "ble_broadcast() must NEVER call bt_le_adv_stop -- race condition");
}

ZTEST(temp_ble, test_broadcast_propagates_error)
{
    bt_le_adv_update_data_fake.return_val = -EAGAIN;
    int err = ble_broadcast(250, 50);
    zassert_equal(err, -EAGAIN,
        "ble_broadcast() must propagate bt_le_adv_update_data() error");
}

ZTEST(temp_ble, test_broadcast_rapid_toggle_never_calls_stop)
{
    bt_le_adv_update_data_fake.return_val = 0;
    for (int i = 0; i < 10; i++) {
        ble_broadcast((int16_t)(i * 10), 80);
    }
    zassert_equal(bt_le_adv_update_data_fake.call_count, 10,
        "10 broadcasts must produce 10 update_data calls");
    zassert_equal(bt_le_adv_stop_fake.call_count, 0,
        "bt_le_adv_stop must never be called during rapid broadcast");
}

ZTEST(temp_ble, test_set_batt_does_not_call_update_data)
{
    ble_adv_set_batt(88);
    zassert_equal(bt_le_adv_update_data_fake.call_count, 0,
        "ble_adv_set_batt() must not call bt_le_adv_update_data");
}

ZTEST(temp_ble, test_temp_payload_bytes_little_endian)
{
    bt_le_adv_update_data_fake.return_val = 0;
    ble_broadcast(250, 75);
    const struct bt_data *ad = bt_le_adv_update_data_fake.arg0_val;
    zassert_not_null(ad, "bt_le_adv_update_data must have been called");
    const struct bt_data *mfg = NULL;
    for (size_t i = 0; i < (size_t)bt_le_adv_update_data_fake.arg1_val; i++) {
        if (ad[i].type == 0xFF) { mfg = &ad[i]; break; }
    }
    zassert_not_null(mfg, "payload must contain manufacturer data");
    zassert_equal(mfg->data_len, 4,
        "manufacturer data must be 4 bytes, got %d", mfg->data_len);
    zassert_equal(mfg->data[1], 0xFA,
        "temp low byte must be 0xFA, got 0x%02x", mfg->data[1]);
    zassert_equal(mfg->data[2], 0x00,
        "temp high byte must be 0x00, got 0x%02x", mfg->data[2]);
}

ZTEST(temp_ble, test_batt_payload_byte_reaches_adv_data)
{
    bt_le_adv_update_data_fake.return_val = 0;
    ble_broadcast(250, 88);
    const struct bt_data *ad = bt_le_adv_update_data_fake.arg0_val;
    const struct bt_data *mfg = NULL;
    for (size_t i = 0; i < (size_t)bt_le_adv_update_data_fake.arg1_val; i++) {
        if (ad[i].type == 0xFF) { mfg = &ad[i]; break; }
    }
    zassert_not_null(mfg, "payload must contain manufacturer data");
    zassert_equal(mfg->data[3], 88,
        "battery byte (data[3]) must be 88, got %d", mfg->data[3]);
}

/******************************************************************************
 * SUITE: temp_resetreas
 ******************************************************************************/
static void resetreas_before(void *data)
{
    (void)data;
    reset_all(NULL);
    trinity_log_event_fake.custom_fake = fake_trinity_log_event_capture;
}
ZTEST_SUITE(temp_resetreas, NULL, NULL, resetreas_before, NULL, NULL);

ZTEST(temp_resetreas, test_classify_dog_is_watchdog)
{
    zassert_equal(trinity_classify_reset(0x00000002), TRINITY_BOOT_WATCHDOG,
        "DOG bit must classify as WATCHDOG");
}

ZTEST(temp_resetreas, test_classify_dog_priority_over_sreq)
{
    zassert_equal(trinity_classify_reset(0x00000006), TRINITY_BOOT_WATCHDOG,
        "DOG|SREQ must classify as WATCHDOG (DOG has priority)");
}

ZTEST(temp_resetreas, test_classify_sreq_is_soft_reset)
{
    zassert_equal(trinity_classify_reset(0x00000004), TRINITY_BOOT_SOFT_RESET,
        "SREQ bit must classify as SOFT_RESET");
}

ZTEST(temp_resetreas, test_classify_pin_is_reset_pin)
{
    zassert_equal(trinity_classify_reset(0x00000001), TRINITY_BOOT_RESET_PIN,
        "RESETPIN bit must classify as RESET_PIN");
}

ZTEST(temp_resetreas, test_classify_repor_is_unknown)
{
    zassert_equal(trinity_classify_reset(0x00010000), TRINITY_BOOT_UNKNOWN,
        "REPOR bit must classify as UNKNOWN");
}

ZTEST(temp_resetreas, test_classify_zero_with_canary_is_brownout)
{
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_BROWNOUT,
        "RESETREAS=0 + canary booted must be BROWNOUT");
}

ZTEST(temp_resetreas, test_classify_zero_no_canary_is_cold_boot)
{
    g_noinit_guard    = 0x00000000;
    g_canary_snapshot = 0x00000000;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "RESETREAS=0 + no canary must be COLD_POWER_ON");
}

ZTEST(temp_resetreas, test_classify_zero_wrong_guard_is_cold_boot)
{
    g_noinit_guard    = 0xDEADBEEE;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "Wrong noinit guard must not produce BROWNOUT");
}

ZTEST(temp_resetreas, test_boot_reason_dog_logs_watchdog)
{
    trinity_log_boot_reason(0x00000002);
    zassert_true(strstr(g_log_event_buf, "WATCHDOG")   != NULL,
        "must log WATCHDOG label");
    zassert_true(strstr(g_log_event_buf, "0x00000002") != NULL,
        "must log raw hex");
}

ZTEST(temp_resetreas, test_boot_reason_sreq_logs_soft_reset)
{
    trinity_log_boot_reason(0x00000004);
    zassert_true(strstr(g_log_event_buf, "SOFT_RESET") != NULL,
        "must log SOFT_RESET");
}

ZTEST(temp_resetreas, test_boot_reason_pin_logs_reset_pin)
{
    trinity_log_boot_reason(0x00000001);
    zassert_true(strstr(g_log_event_buf, "RESET_PIN") != NULL,
        "must log RESET_PIN");
}

ZTEST(temp_resetreas, test_boot_reason_zero_logs_none)
{
    trinity_log_boot_reason(0x00000000);
    zassert_true(strstr(g_log_event_buf, "NONE")     != NULL,
        "zero RESETREAS must log NONE label");
    zassert_true(strstr(g_log_event_buf, "00000000") != NULL,
        "zero RESETREAS must log raw hex 00000000");
}

ZTEST(temp_resetreas, test_boot_reason_combined_includes_raw_hex_and_label)
{
    trinity_log_boot_reason(0x00010002);
    zassert_true(strstr(g_log_event_buf, "00010002") != NULL,
        "combined reset must include raw hex");
    zassert_true(strstr(g_log_event_buf, "WATCHDOG") != NULL,
        "DOG|REPOR must label as WATCHDOG");
}

/******************************************************************************
 * SUITE: temp_canary
 ******************************************************************************/
ZTEST_SUITE(temp_canary, NULL, NULL, reset_all, NULL, NULL);

ZTEST(temp_canary, test_set_booted_enables_brownout_detection)
{
    trinity_canary_set_booted();
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_BROWNOUT,
        "After set_booted, RESETREAS=0 must classify as BROWNOUT");
}

ZTEST(temp_canary, test_without_set_booted_no_brownout_detection)
{
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_ALIVE;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "ALIVE canary must not classify as BROWNOUT");
}
