/******************************************************************************
 * \file test_reed_main.c
 * \brief Unit tests for reed sensor -- full regression coverage.
 *
 * Coverage map:
 *   mv_to_soc()                  battery.h      -- boundary + midpoint
 *   battery_init()               battery.c      -- ready, not-ready, null
 *   battery_read_mv()            battery.c      -- null guard, uV->mV, error
 *   battery_read_soc()           battery.c      -- null guard, clamp, error
 *   battery_print_status()       battery.c      -- null guard path
 *   ble_broadcast()              ble_adv.c      -- payload, no stop, errors
 *   ble_adv_set_batt()           ble_adv.c      -- no update_data called
 *   trinity_classify_reset()     trinity_boot.c -- all 7 code paths
 *   trinity_log_boot_reason()    trinity_boot.c -- all labels + raw hex
 *   trinity_canary_set_booted()  trinity_canary.c -- canary upgrade
 *   REED_POLL_MS / ticks         main.h         -- compile-time guards
 *
 * Run with: west build -t run (ztest framework)
 ******************************************************************************/

#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <string.h>
#include <stdbool.h>
#include "trinity_log.h"
#include "main.h"
#include "battery.h"
#include "ble_adv.h"
#include <zephyr/bluetooth/bluetooth.h>

DEFINE_FFF_GLOBALS;

/* Declared in stubs/stub_device.c -- controls device_is_ready() return */
extern bool g_stub_device_ready;

/******************************************************************************
 * FFF mocks -- every external symbol the production .c files call.
 * Signatures must exactly match the real headers.
 ******************************************************************************/

/* trinity_flash.c surface */
FAKE_VOID_FUNC(trinity_log_event, const char *);
/* Capture buffer for trinity_log_event -- the real function passes a stack
 * pointer so we copy before the callee returns. Read g_log_event_buf in tests
 * instead of trinity_log_event_fake.arg0_val. */
static char g_log_event_buf[128];
static void fake_trinity_log_event_capture(const char *msg)
{
    strncpy(g_log_event_buf, msg, sizeof(g_log_event_buf) - 1);
    g_log_event_buf[sizeof(g_log_event_buf) - 1] = '\0';
}


/* trinity_wdt.c surface */
FAKE_VOID_FUNC(trinity_wdt_kick);
FAKE_VOID_FUNC(trinity_wdt_init);

/* trinity_log init surface (called from main, not tested here) */
FAKE_VALUE_FUNC(int,  trinity_log_init);
FAKE_VOID_FUNC(trinity_log_dump_previous);
FAKE_VOID_FUNC(trinity_log_dump_previous_deferred);

/* Zephyr fuel_gauge API -- called by battery.c */
FAKE_VALUE_FUNC(int, fuel_gauge_get_prop,
                const struct device *,
                enum fuel_gauge_property,
                union fuel_gauge_prop_val *);

/* BLE advertising API -- called by ble_broadcast() */
FAKE_VALUE_FUNC(int, bt_le_adv_update_data,
                const struct bt_data *, size_t,
                const struct bt_data *, size_t);
FAKE_VALUE_FUNC(int, bt_le_adv_stop);

/******************************************************************************
 * Shared teardown -- resets all fakes and stub state between tests.
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

    /* Restore stub device to ready state */
    stub_set_device_ready(true);

    /* Clear noinit sentinels so canary tests start clean */
    g_noinit_guard    = 0;
    g_canary_snapshot = 0;
}

/******************************************************************************
 * Custom fake: injects a SOC value into the fuel_gauge output parameter.
 * Used by battery_read_soc() clamp tests.
 ******************************************************************************/
static uint8_t s_injected_soc = 0;
static int fuel_gauge_inject_soc(const struct device *dev,
                                  enum fuel_gauge_property prop,
                                  union fuel_gauge_prop_val *val)
{
    (void)dev;
    (void)prop;
    val->relative_state_of_charge = s_injected_soc;
    return 0;
}

/******************************************************************************
 * Custom fake: injects a voltage (uV) into the fuel_gauge output parameter.
 ******************************************************************************/
static int s_injected_uv = 0;
static int fuel_gauge_inject_voltage(const struct device *dev,
                                      enum fuel_gauge_property prop,
                                      union fuel_gauge_prop_val *val)
{
    (void)dev;
    (void)prop;
    val->voltage = s_injected_uv;
    return 0;
}

/******************************************************************************
 * SUITE: reed_wdt
 * Compile-time constant constraint guards.
 ******************************************************************************/
ZTEST_SUITE(reed_wdt, NULL, NULL, reset_all, NULL, NULL);

ZTEST(reed_wdt, test_poll_ms_less_than_wdt_timeout)
{
    /* REED_POLL_MS must be < 3000ms WDT timeout.
     * Bug (2026-03-21): was 10000ms -- caused reset every loop. */
    zassert_true(REED_POLL_MS < 3000,
        "REED_POLL_MS=%d >= 3000ms WDT timeout", REED_POLL_MS);
}

ZTEST(reed_wdt, test_batt_update_ticks_preserves_5min_interval)
{
    zassert_equal((uint32_t)BATT_UPDATE_TICKS * REED_POLL_MS, 300000U,
        "Battery interval: %d x %d = %u, want 300000ms",
        BATT_UPDATE_TICKS, REED_POLL_MS,
        (uint32_t)BATT_UPDATE_TICKS * REED_POLL_MS);
}

ZTEST(reed_wdt, test_stats_interval_ticks_preserves_60s)
{
    zassert_equal((uint32_t)STATS_INTERVAL_TICKS * REED_POLL_MS, 60000U,
        "Stats interval: %d x %d = %u, want 60000ms",
        STATS_INTERVAL_TICKS, REED_POLL_MS,
        (uint32_t)STATS_INTERVAL_TICKS * REED_POLL_MS);
}

/******************************************************************************
 * SUITE: reed_mfg
 * Layout constants -- hub parses g_mfg_data[] by fixed byte index.
 ******************************************************************************/
ZTEST_SUITE(reed_mfg, NULL, NULL, reset_all, NULL, NULL);

ZTEST(reed_mfg, test_mfg_data_size_is_3)
{
    zassert_equal(MFG_DATA_SIZE, 5,
        "MFG_DATA_SIZE changed -- hub expects exactly 5 bytes (state, batt, tx_id_lo, tx_id_hi)");
}

ZTEST(reed_mfg, test_mfg_company_id_is_0xAB)
{
    zassert_equal(MFG_COMPANY_ID, 0xAB,
        "MFG_COMPANY_ID changed -- hub BLE filter will break");
}

/******************************************************************************
 * SUITE: reed_batt_mv_to_soc
 * mv_to_soc() -- inline in battery.h, pure arithmetic, no mocks needed.
 ******************************************************************************/
ZTEST_SUITE(reed_batt_mv_to_soc, NULL, NULL, reset_all, NULL, NULL);

ZTEST(reed_batt_mv_to_soc, test_clamp_at_max)
{
    zassert_equal(mv_to_soc(3000),  100, "3000mV == 100%%");
    zassert_equal(mv_to_soc(3500),  100, "3500mV clamps to 100%%");
    zassert_equal(mv_to_soc(65535), 100, "overflow clamps to 100%%");
}

ZTEST(reed_batt_mv_to_soc, test_clamp_at_min)
{
    zassert_equal(mv_to_soc(2000), 0, "2000mV == 0%%");
    zassert_equal(mv_to_soc(1500), 0, "1500mV clamps to 0%%");
    zassert_equal(mv_to_soc(0),    0, "0mV clamps to 0%%");
}

ZTEST(reed_batt_mv_to_soc, test_midpoint)
{
    zassert_equal(mv_to_soc(2500), 50, "2500mV == 50%%");
}

ZTEST(reed_batt_mv_to_soc, test_quarter_points)
{
    zassert_equal(mv_to_soc(2250), 25, "2250mV == 25%%");
    zassert_equal(mv_to_soc(2750), 75, "2750mV == 75%%");
}

ZTEST(reed_batt_mv_to_soc, test_boundary_adjacent)
{
    zassert_true(mv_to_soc(2010) > 0,   "2010mV must be > 0%%");
    zassert_true(mv_to_soc(2999) < 100, "2999mV must be < 100%%");
}

/******************************************************************************
 * SUITE: reed_batt_fuel_gauge
 * battery_init(), battery_read_mv(), battery_read_soc(),
 * battery_print_status() -- all exercised via mocked fuel_gauge_get_prop().
 ******************************************************************************/
static void batt_before(void *data)
{
    (void)data;
    reset_all(NULL);
    trinity_log_event_fake.custom_fake = fake_trinity_log_event_capture;
    /* Device is ready by default after reset_all(); initialise the driver
     * so tests focus on the behaviour under test, not on setup. */
    battery_init();
}
ZTEST_SUITE(reed_batt_fuel_gauge, NULL, NULL, batt_before, NULL, NULL);

/* Before-hook for null-guard tests: reset fakes but do NOT call battery_init()
 * so that g_fg remains NULL inside the driver. */
static void batt_before_null(void *data)
{
    (void)data;
    reset_all(NULL);
    /* Force battery_init() to set g_fg = NULL by marking device not ready. */
    stub_set_device_ready(false);
    battery_init();
    /* Reset fakes so null-guard tests start with a clean call count. */
    RESET_FAKE(trinity_log_event);
    RESET_FAKE(fuel_gauge_get_prop);
    FFF_RESET_HISTORY();
}
ZTEST_SUITE(reed_batt_fuel_gauge_null, NULL, NULL, batt_before_null, NULL, NULL);

/* --- battery_init() paths --- */

ZTEST(reed_batt_fuel_gauge, test_init_succeeds_when_device_ready)
{
    stub_set_device_ready(true);
    int rc = battery_init();
    zassert_equal(rc, 0,
        "battery_init() must return 0 when device is ready, got %d", rc);
}

ZTEST(reed_batt_fuel_gauge, test_init_fails_when_device_not_ready)
{
    /* Simulates MAX17048 not found on I2C bus (NACK / not powered). */
    stub_set_device_ready(false);
    int rc = battery_init();
    zassert_equal(rc, -ENODEV,
        "battery_init() must return -ENODEV when device not ready, got %d", rc);
}

ZTEST(reed_batt_fuel_gauge, test_init_idempotent_double_call_does_not_crash)
{
    /* Calling battery_init() twice must not crash or corrupt state.
     * Second call re-assigns g_fg -- reads after must still work. */
    int rc1 = battery_init();
    int rc2 = battery_init();
    zassert_equal(rc1, 0, "first battery_init() must succeed, got %d", rc1);
    zassert_equal(rc2, 0, "second battery_init() must succeed, got %d", rc2);

    fuel_gauge_get_prop_fake.return_val = 0;
    int rc = battery_read_mv();
    zassert_not_equal(rc, -EIO,
        "battery_read_mv() must not return -EIO after double init");
}

/* --- battery_read_mv() paths --- */

ZTEST(reed_batt_fuel_gauge_null, test_read_mv_null_guard_returns_eio)
{
    /* g_fg is NULL -- battery_init() not called.
     * Must return -EIO without crashing. */
    int rc = battery_read_mv();
    zassert_equal(rc, -EIO,
        "battery_read_mv() with NULL g_fg must return -EIO, got %d", rc);
    zassert_equal(fuel_gauge_get_prop_fake.call_count, 0,
        "fuel_gauge_get_prop must not be called when g_fg is NULL");
}

ZTEST(reed_batt_fuel_gauge, test_read_mv_fuel_gauge_error_returns_eio)
{
    fuel_gauge_get_prop_fake.return_val = -EIO;

    int rc = battery_read_mv();
    zassert_equal(rc, -EIO,
        "battery_read_mv() must return -EIO on fuel_gauge error, got %d", rc);
}

ZTEST(reed_batt_fuel_gauge, test_read_mv_converts_uv_to_mv)
{
    /* fuel_gauge_get_prop returns voltage in uV.
     * 3000000 uV -> 3000 mV. */
    stub_set_device_ready(true);
    battery_init();

    s_injected_uv = 3000000;
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_voltage;

    int mv = battery_read_mv();
    zassert_equal(mv, 3000,
        "3000000 uV must convert to 3000 mV, got %d", mv);
}

ZTEST(reed_batt_fuel_gauge, test_read_mv_calls_fuel_gauge_exactly_once)
{
    RESET_FAKE(fuel_gauge_get_prop);
    fuel_gauge_get_prop_fake.return_val = 0;

    battery_read_mv();
    zassert_equal(fuel_gauge_get_prop_fake.call_count, 1,
        "battery_read_mv() must call fuel_gauge_get_prop exactly once");
}

/* --- battery_read_soc() paths --- */

ZTEST(reed_batt_fuel_gauge_null, test_read_soc_null_guard_returns_zero)
{
    uint8_t soc = battery_read_soc();
    zassert_equal(soc, 0,
        "battery_read_soc() with NULL g_fg must return 0, got %d", soc);
    zassert_equal(fuel_gauge_get_prop_fake.call_count, 0,
        "fuel_gauge_get_prop must not be called when g_fg is NULL");
}

ZTEST(reed_batt_fuel_gauge, test_read_soc_fuel_gauge_error_returns_zero)
{
    fuel_gauge_get_prop_fake.return_val = -EIO;

    uint8_t soc = battery_read_soc();
    zassert_equal(soc, 0,
        "battery_read_soc() on fuel_gauge error must return 0, got %d", soc);
}

ZTEST(reed_batt_fuel_gauge, test_read_soc_clamps_above_100)
{
    /* ModelGauge reports >100% on fresh cells until algorithm settles.
     * battery_read_soc() must clamp to 100. */
    stub_set_device_ready(true);
    battery_init();

    s_injected_soc = 120;  /* fresh cell over-report */
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_soc;

    uint8_t soc = battery_read_soc();
    zassert_equal(soc, 100,
        "SOC 120%% from ModelGauge must clamp to 100, got %d", soc);
}

ZTEST(reed_batt_fuel_gauge, test_read_soc_normal_value_passes_through)
{

    s_injected_soc = 73;
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_soc;

    uint8_t soc = battery_read_soc();
    zassert_equal(soc, 73,
        "SOC 73%% must pass through unclamped, got %d", soc);
}

ZTEST(reed_batt_fuel_gauge, test_read_soc_boundary_100_not_clamped)
{

    s_injected_soc = 100;
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_soc;

    uint8_t soc = battery_read_soc();
    zassert_equal(soc, 100,
        "SOC exactly 100%% must not be clamped, got %d", soc);
}

/* --- battery_print_status() paths --- */

ZTEST(reed_batt_fuel_gauge_null, test_print_status_null_guard_no_crash)
{
    /* g_fg is NULL -- must return early without fault or log call. */
    battery_print_status();
    zassert_equal(trinity_log_event_fake.call_count, 0,
        "battery_print_status() with NULL g_fg must not call trinity_log_event");
}

ZTEST(reed_batt_fuel_gauge, test_print_status_fuel_gauge_error_skips_log)
{
    /* If battery_read_mv() fails, battery_print_status() must return early
     * and must NOT call trinity_log_event. */
    fuel_gauge_get_prop_fake.return_val = -EIO;

    battery_print_status();

    zassert_equal(trinity_log_event_fake.call_count, 0,
        "battery_print_status() must not log when battery_read_mv() fails");
}

ZTEST(reed_batt_fuel_gauge, test_print_status_calls_log_event_on_success)
{

    s_injected_uv = 2800000;  /* 2800 mV */
    fuel_gauge_get_prop_fake.custom_fake = fuel_gauge_inject_voltage;

    battery_print_status();
    zassert_true(trinity_log_event_fake.call_count >= 1,
        "battery_print_status() must call trinity_log_event at least once");

    /* Logged string must contain the voltage */
    zassert_true(strstr(g_log_event_buf, "2800") != NULL,
        "battery_print_status() log must contain the mV value '2800'");
}

/******************************************************************************
 * SUITE: reed_ble
 * ble_broadcast() and ble_adv_set_batt() -- call production functions,
 * observe mocked bt_le_adv_update_data / bt_le_adv_stop.
 *
 * Key regression (2026-03-23): stop/start pattern races BLE radio ISR.
 ******************************************************************************/
ZTEST_SUITE(reed_ble, NULL, NULL, reset_all, NULL, NULL);

ZTEST(reed_ble, test_broadcast_calls_update_data_not_stop)
{
    bt_le_adv_update_data_fake.return_val = 0;

    int err = ble_broadcast(1, 75);

    zassert_equal(err, 0,
        "ble_broadcast() must return 0 on success, got %d", err);
    zassert_equal(bt_le_adv_update_data_fake.call_count, 1,
        "ble_broadcast() must call bt_le_adv_update_data exactly once");
    zassert_equal(bt_le_adv_stop_fake.call_count, 0,
        "ble_broadcast() must NEVER call bt_le_adv_stop -- race condition bug");
}

ZTEST(reed_ble, test_broadcast_propagates_error)
{
    bt_le_adv_update_data_fake.return_val = -EAGAIN;

    int err = ble_broadcast(0, 50);

    zassert_equal(err, -EAGAIN,
        "ble_broadcast() must propagate bt_le_adv_update_data() error code");
}

ZTEST(reed_ble, test_broadcast_rapid_toggle_never_calls_stop)
{
    /* Regression: rapid toggling was the exact trigger for the ISR race.
     * Ten back-to-back broadcasts must never call stop. */
    bt_le_adv_update_data_fake.return_val = 0;

    for (int i = 0; i < 10; i++) {
        ble_broadcast((uint8_t)(i % 2), 80);
    }

    zassert_equal(bt_le_adv_update_data_fake.call_count, 10,
        "10 broadcasts must produce 10 update_data calls");
    zassert_equal(bt_le_adv_stop_fake.call_count, 0,
        "bt_le_adv_stop must be called 0 times during rapid toggling");
}

ZTEST(reed_ble, test_set_batt_does_not_call_update_data)
{
    /* ble_adv_set_batt() only writes g_mfg_data[2].
     * It must NOT call bt_le_adv_update_data -- the BLE stack may not
     * be up yet when this is called from main() before bt_enable(). */
    ble_adv_set_batt(88);

    zassert_equal(bt_le_adv_update_data_fake.call_count, 0,
        "ble_adv_set_batt() must not call bt_le_adv_update_data");
    zassert_equal(bt_le_adv_stop_fake.call_count, 0,
        "ble_adv_set_batt() must not call bt_le_adv_stop");
}

ZTEST(reed_ble, test_set_batt_then_broadcast_uses_update_data_once)
{
    /* set_batt() pre-populates, broadcast() transmits -- one update_data. */
    bt_le_adv_update_data_fake.return_val = 0;

    ble_adv_set_batt(55);
    ble_broadcast(0, 55);

    zassert_equal(bt_le_adv_update_data_fake.call_count, 1,
        "set_batt + broadcast must produce exactly one update_data call");
}

ZTEST(reed_ble, test_set_batt_payload_byte_reaches_adv_data)
{
    /* Regression guard: ble_adv_set_batt(X) must result in the battery byte
     * inside the manufacturer data payload equalling X when broadcast fires.
     * g_mfg_data is static in ble_adv.c; inspect via bt_data pointer captured
     * by the fake. data[2] is the battery byte. */
    bt_le_adv_update_data_fake.return_val = 0;

    ble_adv_set_batt(88);
    ble_broadcast(1, 88);

    const struct bt_data *ad = bt_le_adv_update_data_fake.arg0_val;
    zassert_not_null(ad, "bt_le_adv_update_data must have been called");

    const struct bt_data *mfg = NULL;
    for (size_t i = 0; i < (size_t)bt_le_adv_update_data_fake.arg1_val; i++) {
        if (ad[i].type == 0xFF) { mfg = &ad[i]; break; }
    }
    zassert_not_null(mfg, "advertising payload must contain manufacturer data");
    zassert_equal(mfg->data_len, 5,
        "manufacturer data must be 5 bytes, got %d", mfg->data_len);
    zassert_equal(mfg->data[2], 88,
        "battery byte (data[2]) must be 88, got %d", mfg->data[2]);
}

/******************************************************************************
 * SUITE: reed_resetreas
 * trinity_classify_reset() -- all 7 code paths.
 * trinity_log_boot_reason() -- all label paths + raw hex in output.
 ******************************************************************************/
static void resetreas_before(void *data)
{
    (void)data;
    reset_all(NULL);
    trinity_log_event_fake.custom_fake = fake_trinity_log_event_capture;
}
ZTEST_SUITE(reed_resetreas, NULL, NULL, resetreas_before, NULL, NULL);

/* --- trinity_classify_reset() --- */

ZTEST(reed_resetreas, test_classify_dog_is_watchdog)
{
    zassert_equal(trinity_classify_reset(0x00000002), TRINITY_BOOT_WATCHDOG,
        "DOG bit (bit1) must classify as WATCHDOG");
}

ZTEST(reed_resetreas, test_classify_dog_priority_over_sreq)
{
    /* 0x06 = DOG|SREQ. DOG is first in the if-chain -- must win. */
    zassert_equal(trinity_classify_reset(0x00000006), TRINITY_BOOT_WATCHDOG,
        "DOG|SREQ must classify as WATCHDOG (DOG has priority)");
}

ZTEST(reed_resetreas, test_classify_sreq_is_soft_reset)
{
    zassert_equal(trinity_classify_reset(0x00000004), TRINITY_BOOT_SOFT_RESET,
        "SREQ bit (bit2) must classify as SOFT_RESET");
}

ZTEST(reed_resetreas, test_classify_pin_is_reset_pin_not_brownout)
{
    /* Regression: old code had wrong bit for BROWNOUT, mapped bit0 wrong. */
    zassert_equal(trinity_classify_reset(0x00000001), TRINITY_BOOT_RESET_PIN,
        "RESETPIN (bit0) must classify as RESET_PIN, not BROWNOUT");
}

ZTEST(reed_resetreas, test_classify_repor_is_unknown)
{
    zassert_equal(trinity_classify_reset(0x00010000), TRINITY_BOOT_UNKNOWN,
        "REPOR (bit16) must classify as UNKNOWN");
}

ZTEST(reed_resetreas, test_classify_zero_with_canary_is_brownout)
{
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;

    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_BROWNOUT,
        "RESETREAS=0 + canary booted must be BROWNOUT");
}

ZTEST(reed_resetreas, test_classify_zero_no_canary_is_cold_boot)
{
    g_noinit_guard    = 0x00000000;
    g_canary_snapshot = 0x00000000;

    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "RESETREAS=0 + no canary must be COLD_POWER_ON");
}

ZTEST(reed_resetreas, test_classify_zero_wrong_guard_is_cold_boot)
{
    /* Guard must be exactly 0xDEADBEEF -- partial match must not pass. */
    g_noinit_guard    = 0xDEADBEEE;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;

    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "Wrong noinit guard must not produce BROWNOUT");
}

/* --- trinity_log_boot_reason() --- */

ZTEST(reed_resetreas, test_boot_reason_dog_logs_watchdog)
{
    trinity_log_boot_reason(0x00000002);

    zassert_equal(trinity_log_event_fake.call_count, 1,
        "trinity_log_boot_reason must call trinity_log_event once");
    const char *msg = g_log_event_buf;
    zassert_true(strstr(msg, "WATCHDOG")   != NULL, "must log WATCHDOG label");
    zassert_true(strstr(msg, "0x00000002") != NULL, "must log raw hex");
}

ZTEST(reed_resetreas, test_boot_reason_lockup_logs_lockup)
{
    trinity_log_boot_reason(0x00000008); /* LOCKUP bit3 */
    const char *msg = g_log_event_buf;
    zassert_true(strstr(msg, "LOCKUP") != NULL, "must log LOCKUP label");
}

ZTEST(reed_resetreas, test_boot_reason_sreq_logs_soft_reset)
{
    trinity_log_boot_reason(0x00000004);
    const char *msg = g_log_event_buf;
    zassert_true(strstr(msg, "SOFT_RESET") != NULL, "must log SOFT_RESET label");
}

ZTEST(reed_resetreas, test_boot_reason_pin_logs_reset_pin)
{
    trinity_log_boot_reason(0x00000001);
    const char *msg = g_log_event_buf;
    zassert_true(strstr(msg, "RESET_PIN") != NULL, "must log RESET_PIN label");
}

ZTEST(reed_resetreas, test_boot_reason_repor_logs_cold_por)
{
    trinity_log_boot_reason(0x00010000);
    const char *msg = g_log_event_buf;
    zassert_true(strstr(msg, "COLD_POR") != NULL, "must log COLD_POR label");
}

ZTEST(reed_resetreas, test_boot_reason_zero_logs_none)
{
    trinity_log_boot_reason(0x00000000);
    const char *msg = g_log_event_buf;
    zassert_true(strstr(msg, "NONE")     != NULL, "zero RESETREAS must log NONE label");
    zassert_true(strstr(msg, "00000000") != NULL, "zero RESETREAS must log raw hex 00000000");
}

ZTEST(reed_resetreas, test_boot_reason_combined_includes_raw_hex_and_label)
{
    /* First post-flash boot: DOG|REPOR = 0x00010002.
     * Must log raw hex AND decode DOG as WATCHDOG (DOG wins priority). */
    trinity_log_boot_reason(0x00010002);
    const char *msg = g_log_event_buf;
    zassert_true(strstr(msg, "00010002") != NULL,
        "combined reset must include raw hex 00010002");
    zassert_true(strstr(msg, "WATCHDOG") != NULL,
        "DOG|REPOR must label as WATCHDOG");
}

/******************************************************************************
 * SUITE: reed_canary
 * trinity_canary_set_booted() -- upgrades canary so next boot's
 * classify_reset() can detect brownout vs cold boot.
 ******************************************************************************/
ZTEST_SUITE(reed_canary, NULL, NULL, reset_all, NULL, NULL);

ZTEST(reed_canary, test_set_booted_enables_brownout_detection)
{
    /* Call set_booted() then simulate what canary_pre_init() does on the
     * NEXT boot: copy g_pre_init_canary -> g_canary_snapshot.
     * g_pre_init_canary is static so we can't read it directly.
     * We verify the downstream effect: after set_booted(), if the system
     * snapshots TRINITY_CANARY_BOOTED and RESETREAS=0, classify returns
     * BROWNOUT -- proving the canary was upgraded. */
    trinity_canary_set_booted();

    /* Simulate next-boot snapshot */
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;

    TRINITY_BOOT_REASON_E reason = trinity_classify_reset(0x00000000);
    zassert_equal(reason, TRINITY_BOOT_BROWNOUT,
        "After set_booted, RESETREAS=0 must classify as BROWNOUT");
}

ZTEST(reed_canary, test_without_set_booted_no_brownout_detection)
{
    /* If set_booted() was never called (e.g. crash before trinity_log_init),
     * canary remains ALIVE, not BOOTED -- classify must return COLD_POWER_ON. */
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_ALIVE; /* pre-init crash scenario */

    TRINITY_BOOT_REASON_E reason = trinity_classify_reset(0x00000000);
    zassert_equal(reason, TRINITY_BOOT_COLD_POWER_ON,
        "ALIVE canary (pre-init crash) must not classify as BROWNOUT");
}

/******************************************************************************
 * SUITE: reed_log_stack
 * Reads CONFIG_LOG_PROCESS_THREAD_STACK_SIZE from Kconfig (via autoconf.h)
 * not a hardcoded literal -- a prj.conf regression will actually fail this.
 ******************************************************************************/
ZTEST_SUITE(reed_log_stack, NULL, NULL, reset_all, NULL, NULL);

ZTEST(reed_log_stack, test_log_thread_stack_size_sufficient)
{
    /* Bug (2026-04-21): stack=1024 overflowed during 9-message stats burst.
     * Fixed to 2048 in prj.conf. This test reads the actual Kconfig value. */
#ifdef CONFIG_LOG_PROCESS_THREAD_STACK_SIZE
    zassert_true(CONFIG_LOG_PROCESS_THREAD_STACK_SIZE >= 2048,
        "Log thread stack %d < 2048 -- will overflow on stats burst",
        CONFIG_LOG_PROCESS_THREAD_STACK_SIZE);
#else
    ztest_test_skip();
#endif
}
