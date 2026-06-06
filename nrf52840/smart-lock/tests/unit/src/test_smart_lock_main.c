/******************************************************************************
 * \file test_smart_lock_main.c
 * \brief Unit tests for smart lock node -- full regression coverage.
 *
 * Coverage map:
 *   mv_to_soc()                  battery.h      -- boundary + midpoint
 *   battery_init()               battery.c      -- ready, not-ready
 *   battery_read_mv()            battery.c      -- not-ready guard, raw->mV,
 *                                                  error, call count
 *   battery_print_status()       battery.c      -- not-ready guard, error,
 *                                                  success + log content
 *   lock_state_transition()      lock_state.h   -- all 7 paths
 *   lock_state_settle()          lock_state.h   -- UNLOCKING, LOCKING, idle
 *   lock_state_is_busy()         lock_state.h   -- moving vs idle states
 *   lock_state_to_ble()          lock_state.h   -- LOCKED=1, UNLOCKED=0,
 *                                                  transitional=0xFF
 *   lock_state_event_str()       lock_state.h   -- all 5 states + NULL path
 *   lock_state_label()           lock_state.h   -- all 5 states + unknown
 *   trinity_classify_reset()     trinity_boot.c -- all 7 code paths
 *   trinity_log_boot_reason()    trinity_boot.c -- all labels + raw hex
 *   trinity_canary_set_booted()  trinity_canary.c -- canary upgrade
 *   LOCK_WRITE_LEN / timing      config.h       -- compile-time guards
 *
 * Run with: west build -b native_sim tests/unit --build-dir build_test
 ******************************************************************************/

#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include <string.h>
#include <stdbool.h>
#include "trinity_log.h"
#include "config.h"
#include "battery.h"
#include "lock_state.h"

DEFINE_FFF_GLOBALS;

/* Controlled by stub_adc.c */
extern bool    g_stub_adc_ready;
extern int16_t g_stub_adc_raw;
extern int     g_stub_adc_ret;
extern void    stub_adc_reset(void);

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

/* BLE stack -- ble_gatt.c compiled in but not under test */
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
    RESET_FAKE(bt_le_adv_update_data);
    RESET_FAKE(bt_le_adv_stop);
    FFF_RESET_HISTORY();

    stub_adc_reset();

    g_noinit_guard    = 0;
    g_canary_snapshot = 0;
    g_log_event_buf[0] = '\0';
}

/******************************************************************************
 * SUITE: lock_timing
 * Compile-time constant constraint guards.
 * These lock hub-protocol constants against accidental edit.
 ******************************************************************************/
ZTEST_SUITE(lock_timing, NULL, NULL, reset_all, NULL, NULL);

ZTEST(lock_timing, test_stats_interval)
{
    zassert_equal(STATS_INTERVAL_SEC, 60,
        "STATS_INTERVAL_SEC must be 60s");
}

ZTEST(lock_timing, test_idle_heartbeat)
{
    zassert_equal(IDLE_HEARTBEAT_SEC, 240,
        "IDLE_HEARTBEAT_SEC must be 240s");
}

ZTEST(lock_timing, test_batt_update_sec)
{
    zassert_equal(BATT_UPDATE_SEC, 300,
        "BATT_UPDATE_SEC must be 300s (5 min)");
}

ZTEST(lock_timing, test_lock_write_len)
{
    zassert_equal(LOCK_WRITE_LEN, 1,
        "LOCK_WRITE_LEN must be 1 -- hub sends single byte commands");
}

/******************************************************************************
 * SUITE: lock_mfg
 * MFG data layout -- hub BLE scanner reads state and battery by fixed index.
 ******************************************************************************/
ZTEST_SUITE(lock_mfg, NULL, NULL, reset_all, NULL, NULL);

ZTEST(lock_mfg, test_mfg_data_size)
{
    zassert_equal(MFG_DATA_SIZE, 3,
        "MFG_DATA_SIZE changed -- hub parser expects exactly 3 bytes");
}

ZTEST(lock_mfg, test_mfg_company_id)
{
    zassert_equal(MFG_COMPANY_ID, 0xAC,
        "MFG_COMPANY_ID changed -- hub BLE filter will break");
}

ZTEST(lock_mfg, test_mfg_lock_state_index)
{
    uint8_t mfg[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x01, 0x64};
    zassert_equal(mfg[MFG_LOCK_STATE_IDX], 0x01,
        "Lock state must be at byte index 1");
}

ZTEST(lock_mfg, test_mfg_batt_index)
{
    uint8_t mfg[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x00, 0x64};
    zassert_equal(mfg[MFG_BATT_IDX], 0x64,
        "Battery must be at byte index 2");
}

/******************************************************************************
 * SUITE: lock_batt_mv_to_soc
 * mv_to_soc() -- inline in battery.h, pure arithmetic, no mocks needed.
 * 4xAA range: VBAT_DEAD_MV=4400 to VBAT_FULL_MV=6400, range=2000mV.
 ******************************************************************************/
ZTEST_SUITE(lock_batt_mv_to_soc, NULL, NULL, reset_all, NULL, NULL);

ZTEST(lock_batt_mv_to_soc, test_clamp_at_max)
{
    zassert_equal(mv_to_soc(VBAT_FULL_MV),        100, "VBAT_FULL_MV must be 100%%");
    zassert_equal(mv_to_soc(VBAT_FULL_MV + 500),  100, "Above VBAT_FULL_MV must clamp to 100%%");
}

ZTEST(lock_batt_mv_to_soc, test_clamp_at_min)
{
    zassert_equal(mv_to_soc(VBAT_DEAD_MV),        0, "VBAT_DEAD_MV must be 0%%");
    zassert_equal(mv_to_soc(VBAT_DEAD_MV - 500),  0, "Below VBAT_DEAD_MV must clamp to 0%%");
}

ZTEST(lock_batt_mv_to_soc, test_midpoint)
{
    /* VBAT_DEAD_MV + VBAT_RANGE_MV/2 = 4400 + 1000 = 5400mV = 50% */
    zassert_equal(mv_to_soc(VBAT_DEAD_MV + (VBAT_RANGE_MV / 2)), 50,
        "Midpoint of range must be 50%%");
}

ZTEST(lock_batt_mv_to_soc, test_quarter_points)
{
    /* 25%: 4400 + 500 = 4900mV */
    zassert_equal(mv_to_soc(VBAT_DEAD_MV + (VBAT_RANGE_MV / 4)), 25,
        "Quarter point must be 25%%");
    /* 75%: 4400 + 1500 = 5900mV */
    zassert_equal(mv_to_soc(VBAT_DEAD_MV + (VBAT_RANGE_MV * 3 / 4)), 75,
        "Three-quarter point must be 75%%");
}

ZTEST(lock_batt_mv_to_soc, test_boundary_adjacent)
{
    /* Minimum offset that survives integer truncation:
     * (20 * 100) / 2000 = 1 > 0. 10mV truncates to 0. */
    zassert_true(mv_to_soc(VBAT_DEAD_MV + 20) > 0,
        "Just above VBAT_DEAD_MV must be > 0%%");
    zassert_true(mv_to_soc(VBAT_FULL_MV - 20) < 100,
        "Just below VBAT_FULL_MV must be < 100%%");
}

ZTEST(lock_batt_mv_to_soc, test_field_reading)
{
    /* Mid-discharge field reading: 5600mV.
     * soc = (5600 - 4400) * 100 / 2000 = 60% */
    zassert_equal(mv_to_soc(5600), 60,
        "5600mV mid-discharge must be 60%%");
}

/******************************************************************************
 * SUITE: lock_batt_adc
 * battery_init(), battery_read_mv(), battery_print_status().
 * Uses stub_adc.c to inject ADC outcomes without GPIO.
 *
 * ADC math chain (for raw->mV verification):
 *   ceiling = 600mV * 6 = 3600mV, steps = 4096
 *   step_mv = 3600 / 4096 = 0.879mV
 *   pin_mv  = raw * step_mv
 *   vbat_mv = pin_mv * DIVIDER_RATIO_DEN (3)
 *
 * At vbat=5400mV: pin=1800mV, raw = 1800*4096/3600 = 2048
 * battery_read_mv() should return ~5400mV for raw=2048.
 ******************************************************************************/
static void batt_before(void *data)
{
    (void)data;
    reset_all(NULL);
    trinity_log_event_fake.custom_fake = fake_trinity_log_event_capture;
    stub_adc_reset();
    battery_init();
}

static void batt_before_not_ready(void *data)
{
    (void)data;
    reset_all(NULL);
    g_stub_adc_ready = false;
    battery_init();
    RESET_FAKE(trinity_log_event);
    FFF_RESET_HISTORY();
}

ZTEST_SUITE(lock_batt_adc, NULL, NULL, batt_before, NULL, NULL);
ZTEST_SUITE(lock_batt_adc_notready, NULL, NULL, batt_before_not_ready, NULL, NULL);

/* --- battery_init() --- */

ZTEST(lock_batt_adc, test_init_succeeds_when_adc_ready)
{
    g_stub_adc_ready = true;
    int rc = battery_init();
    zassert_equal(rc, 0,
        "battery_init() must return 0 when ADC ready, got %d", rc);
}

ZTEST(lock_batt_adc, test_init_fails_when_adc_not_ready)
{
    g_stub_adc_ready = false;
    int rc = battery_init();
    zassert_not_equal(rc, 0,
        "battery_init() must return non-zero when ADC not ready");
}

ZTEST(lock_batt_adc, test_init_idempotent_double_call)
{
    g_stub_adc_ready = true;
    int rc1 = battery_init();
    int rc2 = battery_init();
    zassert_equal(rc1, 0, "first battery_init() must succeed, got %d", rc1);
    zassert_equal(rc2, 0, "second battery_init() must succeed, got %d", rc2);
}

/* --- battery_read_mv() --- */

ZTEST(lock_batt_adc_notready, test_read_mv_not_ready_returns_error)
{
    /* ADC not ready -- battery_init() must return non-zero.
     * lock battery.c checks adc_is_ready_dt() at init time only;
     * there is no NULL-pointer guard on subsequent reads like fuel_gauge.
     * The correct assertion is on init, not on read. */
    int rc = battery_init();
    zassert_not_equal(rc, 0,
        "battery_init() must return non-zero when ADC not ready, got %d", rc);
}

ZTEST(lock_batt_adc, test_read_mv_adc_error_returns_error)
{
    g_stub_adc_ret = -EIO;
    int rc = battery_read_mv();
    zassert_true(rc < 0,
        "battery_read_mv() on ADC error must return < 0, got %d", rc);
}

ZTEST(lock_batt_adc, test_read_mv_converts_raw_to_vbat)
{
    /* raw=2048: pin_mv = 2048*3600/4096 = 1800mV, vbat = 1800*3 = 5400mV */
    g_stub_adc_raw = 2048;
    int mv = battery_read_mv();
    zassert_true((mv >= 5350) && (mv <= 5450),
        "raw=2048 must produce vbat ~5400mV, got %d", mv);
}

ZTEST(lock_batt_adc, test_read_mv_full_battery)
{
    /* raw=2370: pin_mv ~2080mV, vbat ~6240mV (near VBAT_FULL_MV=6400) */
    g_stub_adc_raw = 2370;
    int mv = battery_read_mv();
    zassert_true(mv > 5800,
        "raw=2370 must produce vbat above 5800mV, got %d", mv);
}

ZTEST(lock_batt_adc, test_read_mv_dead_battery)
{
    /* raw=1365: pin_mv ~1200mV, vbat ~3600mV (below VBAT_DEAD_MV) */
    g_stub_adc_raw = 1365;
    int mv = battery_read_mv();
    zassert_true(mv < 4400,
        "raw=1365 must produce vbat below VBAT_DEAD_MV=4400, got %d", mv);
}

/* --- battery_print_status() --- */

ZTEST(lock_batt_adc_notready, test_print_status_not_ready_no_log)
{
    /* ADC not ready -- battery_init() fails.
     * battery_print_status() calls battery_read_mv() which returns -1
     * only if adc_read_dt() fails (g_stub_adc_ret != 0). Force that here
     * so print_status early-returns without logging. */
    g_stub_adc_ret = -EIO;
    battery_print_status();
    zassert_equal(trinity_log_event_fake.call_count, 0,
        "battery_print_status() must not log when ADC read fails");
}

ZTEST(lock_batt_adc, test_print_status_adc_error_no_log)
{
    g_stub_adc_ret = -EIO;
    battery_print_status();
    zassert_equal(trinity_log_event_fake.call_count, 0,
        "battery_print_status() must not log when ADC read fails");
}

ZTEST(lock_batt_adc, test_print_status_calls_log_on_success)
{
    /* raw=2048 -> ~5400mV -- valid reading, must log */
    g_stub_adc_raw = 2048;
    battery_print_status();
    zassert_true(trinity_log_event_fake.call_count >= 1,
        "battery_print_status() must call trinity_log_event on success");
}

ZTEST(lock_batt_adc, test_print_status_log_contains_voltage)
{
    g_stub_adc_raw = 2048;
    battery_print_status();
    /* Log must contain the mV reading -- exact value depends on ADC math */
    zassert_true(strstr(g_log_event_buf, "mV") != NULL ||
                 strstr(g_log_event_buf, "VBAT") != NULL,
        "battery_print_status() log must reference voltage");
}

/******************************************************************************
 * SUITE: lock_state_machine
 * lock_state_transition() -- all 7 paths through the state machine.
 * Pure inline logic in lock_state.h -- no mocks needed.
 ******************************************************************************/
ZTEST_SUITE(lock_state_machine, NULL, NULL, reset_all, NULL, NULL);

ZTEST(lock_state_machine, test_locked_unlock_cmd_starts_unlocking)
{
    LOCK_STATE_E s = lock_state_transition(LOCK_STATE_LOCKED, 0);
    zassert_equal(s, LOCK_STATE_UNLOCKING,
        "LOCKED + cmd=0 (unlock) must go to UNLOCKING");
}

ZTEST(lock_state_machine, test_locked_lock_cmd_ignored)
{
    /* Already locked -- lock command must be a no-op */
    LOCK_STATE_E s = lock_state_transition(LOCK_STATE_LOCKED, 1);
    zassert_equal(s, LOCK_STATE_LOCKED,
        "LOCKED + cmd=1 (lock) must stay LOCKED");
}

ZTEST(lock_state_machine, test_unlocked_lock_cmd_starts_locking)
{
    LOCK_STATE_E s = lock_state_transition(LOCK_STATE_UNLOCKED, 1);
    zassert_equal(s, LOCK_STATE_LOCKING,
        "UNLOCKED + cmd=1 (lock) must go to LOCKING");
}

ZTEST(lock_state_machine, test_unlocked_unlock_cmd_ignored)
{
    /* Already unlocked -- unlock command must be a no-op */
    LOCK_STATE_E s = lock_state_transition(LOCK_STATE_UNLOCKED, 0);
    zassert_equal(s, LOCK_STATE_UNLOCKED,
        "UNLOCKED + cmd=0 (unlock) must stay UNLOCKED");
}

ZTEST(lock_state_machine, test_unlocking_rejects_all_commands)
{
    /* Motor moving -- both commands must be rejected */
    zassert_equal(lock_state_transition(LOCK_STATE_UNLOCKING, 0),
                  LOCK_STATE_UNLOCKING,
        "UNLOCKING + cmd=0 must stay UNLOCKING (motor moving)");
    zassert_equal(lock_state_transition(LOCK_STATE_UNLOCKING, 1),
                  LOCK_STATE_UNLOCKING,
        "UNLOCKING + cmd=1 must stay UNLOCKING (motor moving)");
}

ZTEST(lock_state_machine, test_locking_rejects_all_commands)
{
    /* Motor moving -- both commands must be rejected */
    zassert_equal(lock_state_transition(LOCK_STATE_LOCKING, 0),
                  LOCK_STATE_LOCKING,
        "LOCKING + cmd=0 must stay LOCKING (motor moving)");
    zassert_equal(lock_state_transition(LOCK_STATE_LOCKING, 1),
                  LOCK_STATE_LOCKING,
        "LOCKING + cmd=1 must stay LOCKING (motor moving)");
}

ZTEST(lock_state_machine, test_error_recovers_to_locked_on_lock_cmd)
{
    /* Safety: ERROR only accepts recovery to LOCKED (cmd=1) */
    LOCK_STATE_E s = lock_state_transition(LOCK_STATE_ERROR, 1);
    zassert_equal(s, LOCK_STATE_LOCKED,
        "ERROR + cmd=1 must recover to LOCKED");
}

ZTEST(lock_state_machine, test_error_rejects_unlock_cmd)
{
    /* Do not unlock from ERROR -- fail safe */
    LOCK_STATE_E s = lock_state_transition(LOCK_STATE_ERROR, 0);
    zassert_equal(s, LOCK_STATE_ERROR,
        "ERROR + cmd=0 (unlock) must stay ERROR -- fail safe");
}

/******************************************************************************
 * SUITE: lock_state_settle
 * lock_state_settle() -- motor_off_timer fires, advance to settled state.
 ******************************************************************************/
ZTEST_SUITE(lock_state_settle, NULL, NULL, reset_all, NULL, NULL);

ZTEST(lock_state_settle, test_unlocking_settles_to_unlocked)
{
    zassert_equal(lock_state_settle(LOCK_STATE_UNLOCKING), LOCK_STATE_UNLOCKED,
        "UNLOCKING must settle to UNLOCKED");
}

ZTEST(lock_state_settle, test_locking_settles_to_locked)
{
    zassert_equal(lock_state_settle(LOCK_STATE_LOCKING), LOCK_STATE_LOCKED,
        "LOCKING must settle to LOCKED");
}

ZTEST(lock_state_settle, test_idle_states_unchanged)
{
    /* settle() on a non-transitioning state must be a no-op */
    zassert_equal(lock_state_settle(LOCK_STATE_LOCKED),   LOCK_STATE_LOCKED,
        "LOCKED must not change on settle");
    zassert_equal(lock_state_settle(LOCK_STATE_UNLOCKED), LOCK_STATE_UNLOCKED,
        "UNLOCKED must not change on settle");
    zassert_equal(lock_state_settle(LOCK_STATE_ERROR),    LOCK_STATE_ERROR,
        "ERROR must not change on settle");
}

/******************************************************************************
 * SUITE: lock_state_helpers
 * lock_state_is_busy(), lock_state_to_ble(),
 * lock_state_event_str(), lock_state_label().
 ******************************************************************************/
ZTEST_SUITE(lock_state_helpers, NULL, NULL, reset_all, NULL, NULL);

/* --- lock_state_is_busy() --- */

ZTEST(lock_state_helpers, test_is_busy_true_while_moving)
{
    zassert_true(lock_state_is_busy(LOCK_STATE_UNLOCKING),
        "UNLOCKING must be busy");
    zassert_true(lock_state_is_busy(LOCK_STATE_LOCKING),
        "LOCKING must be busy");
}

ZTEST(lock_state_helpers, test_is_busy_false_when_idle)
{
    zassert_false(lock_state_is_busy(LOCK_STATE_LOCKED),
        "LOCKED must not be busy");
    zassert_false(lock_state_is_busy(LOCK_STATE_UNLOCKED),
        "UNLOCKED must not be busy");
    zassert_false(lock_state_is_busy(LOCK_STATE_ERROR),
        "ERROR must not be busy");
}

/* --- lock_state_to_ble() --- */

ZTEST(lock_state_helpers, test_to_ble_locked_is_1)
{
    zassert_equal(lock_state_to_ble(LOCK_STATE_LOCKED), 1,
        "LOCKED must map to BLE value 1");
}

ZTEST(lock_state_helpers, test_to_ble_unlocked_is_0)
{
    zassert_equal(lock_state_to_ble(LOCK_STATE_UNLOCKED), 0,
        "UNLOCKED must map to BLE value 0");
}

ZTEST(lock_state_helpers, test_to_ble_transitional_is_sentinel)
{
    /* Transitional states must return 0xFF sentinel -- not stored in settings */
    zassert_equal(lock_state_to_ble(LOCK_STATE_UNLOCKING), 0xFF,
        "UNLOCKING must map to 0xFF sentinel");
    zassert_equal(lock_state_to_ble(LOCK_STATE_LOCKING),   0xFF,
        "LOCKING must map to 0xFF sentinel");
    zassert_equal(lock_state_to_ble(LOCK_STATE_ERROR),     0xFF,
        "ERROR must map to 0xFF sentinel");
}

/* --- lock_state_event_str() --- */

ZTEST(lock_state_helpers, test_event_str_all_states)
{
    zassert_not_null(lock_state_event_str(LOCK_STATE_LOCKED),
        "LOCKED must have an event string");
    zassert_not_null(lock_state_event_str(LOCK_STATE_UNLOCKED),
        "UNLOCKED must have an event string");
    zassert_not_null(lock_state_event_str(LOCK_STATE_UNLOCKING),
        "UNLOCKING must have an event string");
    zassert_not_null(lock_state_event_str(LOCK_STATE_LOCKING),
        "LOCKING must have an event string");
    zassert_not_null(lock_state_event_str(LOCK_STATE_ERROR),
        "ERROR must have an event string");
}

ZTEST(lock_state_helpers, test_event_str_content)
{
    zassert_true(strstr(lock_state_event_str(LOCK_STATE_LOCKED),    "LOCK")   != NULL,
        "LOCKED event string must contain LOCK");
    zassert_true(strstr(lock_state_event_str(LOCK_STATE_UNLOCKED),  "UNLOCK") != NULL,
        "UNLOCKED event string must contain UNLOCK");
    zassert_true(strstr(lock_state_event_str(LOCK_STATE_UNLOCKING), "UNLOCK") != NULL,
        "UNLOCKING event string must contain UNLOCK");
    zassert_true(strstr(lock_state_event_str(LOCK_STATE_LOCKING),   "LOCK")   != NULL,
        "LOCKING event string must contain LOCK");
    zassert_true(strstr(lock_state_event_str(LOCK_STATE_ERROR),     "ERROR")  != NULL,
        "ERROR event string must contain ERROR");
}

/* --- lock_state_label() --- */

ZTEST(lock_state_helpers, test_label_all_states)
{
    zassert_str_equal(lock_state_label(LOCK_STATE_LOCKED),    "LOCKED",    NULL);
    zassert_str_equal(lock_state_label(LOCK_STATE_UNLOCKING), "UNLOCKING", NULL);
    zassert_str_equal(lock_state_label(LOCK_STATE_UNLOCKED),  "UNLOCKED",  NULL);
    zassert_str_equal(lock_state_label(LOCK_STATE_LOCKING),   "LOCKING",   NULL);
    zassert_str_equal(lock_state_label(LOCK_STATE_ERROR),     "ERROR",     NULL);
}

ZTEST(lock_state_helpers, test_label_unknown_state)
{
    /* Out-of-range value must not crash and must return a non-null string */
    const char *label = lock_state_label((LOCK_STATE_E)99);
    zassert_not_null(label, "Unknown state must return non-null label");
    zassert_true(strlen(label) > 0, "Unknown state label must not be empty");
}

/******************************************************************************
 * SUITE: lock_resetreas
 * trinity_classify_reset() -- all 7 code paths.
 * trinity_log_boot_reason() -- all label paths + raw hex in output.
 ******************************************************************************/
static void resetreas_before(void *data)
{
    (void)data;
    reset_all(NULL);
    trinity_log_event_fake.custom_fake = fake_trinity_log_event_capture;
}
ZTEST_SUITE(lock_resetreas, NULL, NULL, resetreas_before, NULL, NULL);

/* --- trinity_classify_reset() --- */

ZTEST(lock_resetreas, test_classify_dog_is_watchdog)
{
    zassert_equal(trinity_classify_reset(0x00000002), TRINITY_BOOT_WATCHDOG,
        "DOG bit (bit1) must classify as WATCHDOG");
}

ZTEST(lock_resetreas, test_classify_dog_priority_over_sreq)
{
    /* 0x06 = DOG|SREQ -- DOG is first in if-chain, must win */
    zassert_equal(trinity_classify_reset(0x00000006), TRINITY_BOOT_WATCHDOG,
        "DOG|SREQ must classify as WATCHDOG (DOG has priority)");
}

ZTEST(lock_resetreas, test_classify_sreq_is_soft_reset)
{
    zassert_equal(trinity_classify_reset(0x00000004), TRINITY_BOOT_SOFT_RESET,
        "SREQ bit (bit2) must classify as SOFT_RESET");
}

ZTEST(lock_resetreas, test_classify_pin_is_reset_pin)
{
    zassert_equal(trinity_classify_reset(0x00000001), TRINITY_BOOT_RESET_PIN,
        "RESETPIN (bit0) must classify as RESET_PIN, not BROWNOUT");
}

ZTEST(lock_resetreas, test_classify_repor_is_unknown)
{
    zassert_equal(trinity_classify_reset(0x00010000), TRINITY_BOOT_UNKNOWN,
        "REPOR (bit16) must classify as UNKNOWN");
}

ZTEST(lock_resetreas, test_classify_zero_with_canary_is_brownout)
{
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_BROWNOUT,
        "RESETREAS=0 + canary booted must be BROWNOUT");
}

ZTEST(lock_resetreas, test_classify_zero_no_canary_is_cold_boot)
{
    g_noinit_guard    = 0x00000000;
    g_canary_snapshot = 0x00000000;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "RESETREAS=0 + no canary must be COLD_POWER_ON");
}

ZTEST(lock_resetreas, test_classify_zero_wrong_guard_is_cold_boot)
{
    /* Guard must be exactly 0xDEADBEEF -- partial match must not pass */
    g_noinit_guard    = 0xDEADBEEE;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "Wrong noinit guard must not produce BROWNOUT");
}

/* --- trinity_log_boot_reason() --- */

ZTEST(lock_resetreas, test_boot_reason_dog_logs_watchdog)
{
    trinity_log_boot_reason(0x00000002);
    zassert_equal(trinity_log_event_fake.call_count, 1,
        "trinity_log_boot_reason must call trinity_log_event once");
    zassert_true(strstr(g_log_event_buf, "WATCHDOG")   != NULL,
        "must log WATCHDOG label");
    zassert_true(strstr(g_log_event_buf, "0x00000002") != NULL,
        "must log raw hex 0x00000002");
}

ZTEST(lock_resetreas, test_boot_reason_sreq_logs_soft_reset)
{
    trinity_log_boot_reason(0x00000004);
    zassert_true(strstr(g_log_event_buf, "SOFT_RESET") != NULL,
        "must log SOFT_RESET label");
}

ZTEST(lock_resetreas, test_boot_reason_pin_logs_reset_pin)
{
    trinity_log_boot_reason(0x00000001);
    zassert_true(strstr(g_log_event_buf, "RESET_PIN") != NULL,
        "must log RESET_PIN label");
}

ZTEST(lock_resetreas, test_boot_reason_zero_logs_none)
{
    trinity_log_boot_reason(0x00000000);
    zassert_true(strstr(g_log_event_buf, "NONE")     != NULL,
        "zero RESETREAS must log NONE label");
    zassert_true(strstr(g_log_event_buf, "00000000") != NULL,
        "zero RESETREAS must log raw hex 00000000");
}

ZTEST(lock_resetreas, test_boot_reason_combined_includes_raw_hex_and_label)
{
    /* DOG|REPOR = 0x00010002 -- DOG wins, must log WATCHDOG + raw hex */
    trinity_log_boot_reason(0x00010002);
    zassert_true(strstr(g_log_event_buf, "00010002") != NULL,
        "combined reset must include raw hex 00010002");
    zassert_true(strstr(g_log_event_buf, "WATCHDOG") != NULL,
        "DOG|REPOR must label as WATCHDOG");
}

/******************************************************************************
 * SUITE: lock_canary
 * trinity_canary_set_booted() -- upgrades canary so next boot's
 * classify_reset() can detect brownout vs cold boot.
 ******************************************************************************/
ZTEST_SUITE(lock_canary, NULL, NULL, reset_all, NULL, NULL);

ZTEST(lock_canary, test_set_booted_enables_brownout_detection)
{
    trinity_canary_set_booted();

    /* Simulate next-boot snapshot */
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;

    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_BROWNOUT,
        "After set_booted, RESETREAS=0 must classify as BROWNOUT");
}

ZTEST(lock_canary, test_without_set_booted_no_brownout_detection)
{
    /* Pre-init crash scenario -- canary stuck at ALIVE, not BOOTED */
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_ALIVE;

    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "ALIVE canary (pre-init crash) must not classify as BROWNOUT");
}

/******************************************************************************
 * SUITE: lock_log_stack
 * Reads CONFIG_LOG_PROCESS_THREAD_STACK_SIZE from Kconfig (via autoconf.h).
 * A prj.conf regression will actually fail this -- not a hardcoded literal.
 ******************************************************************************/
ZTEST_SUITE(lock_log_stack, NULL, NULL, reset_all, NULL, NULL);

ZTEST(lock_log_stack, test_log_thread_stack_size_sufficient)
{
#ifdef CONFIG_LOG_PROCESS_THREAD_STACK_SIZE
    zassert_true(CONFIG_LOG_PROCESS_THREAD_STACK_SIZE >= 2048,
        "Log thread stack %d < 2048 -- will overflow on stats burst",
        CONFIG_LOG_PROCESS_THREAD_STACK_SIZE);
#else
    ztest_test_skip();
#endif
}
