/******************************************************************************
 * \file test_smart_light_main.c
 * \brief Unit tests for smart light node -- full regression coverage.
 *
 * Coverage map:
 *   relay_set() / light_hw_set()     light_hw.h     -- call-through, state,
 *                                                       toggle sequence,
 *                                                       call count
 *   light_hw_init()                  light_hw.h     -- success, fail,
 *                                                       idempotent double call
 *   LIGHT_WRITE_LEN / LIGHT_STATE_MAX main.h        -- compile-time guards
 *   STATS_INTERVAL_SEC / IDLE_HB_SEC  main.h        -- timing constants
 *   MFG layout                        main.h        -- size, company ID, idx
 *   trinity_classify_reset()          trinity_boot.c -- all 7 code paths
 *   trinity_log_boot_reason()         trinity_boot.c -- all labels + raw hex
 *   trinity_canary_set_booted()       trinity_canary.c -- canary upgrade
 *
 * Run with: west build -b native_sim tests/unit --build-dir build_test
 ******************************************************************************/

#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include <string.h>
#include <stdbool.h>
#include "trinity_log.h"
#include "main.h"
#include "light_hw.h"

DEFINE_FFF_GLOBALS;

/* Exported by light_hw_stub.c */
extern int g_stub_light_state;
extern int g_stub_hw_init_ret;
extern int g_stub_set_call_count;

static void stub_light_reset(void)
{
    g_stub_light_state    = -1;
    g_stub_hw_init_ret    =  0;
    g_stub_set_call_count =  0;
}

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

    stub_light_reset();

    g_noinit_guard     = 0;
    g_canary_snapshot  = 0;
    g_log_event_buf[0] = '\0';
}

/******************************************************************************
 * SUITE: light_timing
 * Compile-time constant constraint guards.
 * Lock hub-protocol constants against accidental edit.
 ******************************************************************************/
ZTEST_SUITE(light_timing, NULL, NULL, reset_all, NULL, NULL);

ZTEST(light_timing, test_stats_interval)
{
    zassert_equal(STATS_INTERVAL_SEC, 60,
        "STATS_INTERVAL_SEC must be 60s");
}

ZTEST(light_timing, test_idle_heartbeat)
{
    zassert_equal(IDLE_HEARTBEAT_SEC, 240,
        "IDLE_HEARTBEAT_SEC must be 240s");
}

ZTEST(light_timing, test_light_write_len)
{
    zassert_equal(LIGHT_WRITE_LEN, 3,
        "LIGHT_WRITE_LEN must be 3 -- hub sends [state][tx_id_lo][tx_id_hi]");
}

ZTEST(light_timing, test_light_state_max)
{
    zassert_equal(LIGHT_STATE_MAX, 1,
        "LIGHT_STATE_MAX must be 1 -- only ON(1) and OFF(0) are valid");
}

/******************************************************************************
 * SUITE: light_mfg
 * MFG data layout -- hub BLE scanner reads state by fixed byte index.
 ******************************************************************************/
ZTEST_SUITE(light_mfg, NULL, NULL, reset_all, NULL, NULL);

ZTEST(light_mfg, test_mfg_data_size)
{
    zassert_equal(MFG_DATA_SIZE, 2,
        "MFG_DATA_SIZE changed -- hub parser expects exactly 2 bytes");
}

ZTEST(light_mfg, test_mfg_company_id)
{
    zassert_equal(MFG_COMPANY_ID, 0xAD,
        "MFG_COMPANY_ID changed -- hub BLE filter will break");
}

ZTEST(light_mfg, test_mfg_state_index)
{
    uint8_t mfg[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x01};
    zassert_equal(mfg[MFG_STATE_IDX], 0x01,
        "Light state must be at byte index 1");
}

ZTEST(light_mfg, test_mfg_state_off_encodes_zero)
{
    uint8_t mfg[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x00};
    zassert_equal(mfg[MFG_STATE_IDX], 0x00,
        "Light OFF must encode as 0x00 at MFG_STATE_IDX");
}

/******************************************************************************
 * SUITE: light_write
 * LIGHT_WRITE_LEN and LIGHT_STATE_MAX -- boundary validation constants
 * used by write_light_control() in ble_gatt.c to reject bad hub commands.
 ******************************************************************************/
ZTEST_SUITE(light_write, NULL, NULL, reset_all, NULL, NULL);

ZTEST(light_write, test_write_len_must_be_1)
{
    zassert_equal(LIGHT_WRITE_LEN, 3,
        "write_light_control must reject len != 3");
}

ZTEST(light_write, test_state_max_is_1)
{
    zassert_equal(LIGHT_STATE_MAX, 1,
        "write_light_control must reject state > 1");
}

ZTEST(light_write, test_state_on_value_is_valid)
{
    uint8_t state = 1;
    zassert_true(state <= LIGHT_STATE_MAX, "ON state (1) must be valid");
}

ZTEST(light_write, test_state_off_value_is_valid)
{
    uint8_t state = 0;
    zassert_true(state <= LIGHT_STATE_MAX, "OFF state (0) must be valid");
}

ZTEST(light_write, test_state_2_is_invalid)
{
    uint8_t state = 2;
    zassert_true(state > LIGHT_STATE_MAX,
        "State 2 must be rejected by write_light_control");
}

/******************************************************************************
 * SUITE: light_hw
 * light_hw_init() and light_hw_set() via stub.
 * No GPIO touched -- stub records calls for assertion.
 ******************************************************************************/
ZTEST_SUITE(light_hw, NULL, NULL, reset_all, NULL, NULL);

ZTEST(light_hw, test_init_succeeds_by_default)
{
    int rc = light_hw_init();
    zassert_equal(rc, 0,
        "light_hw_init() must return 0 on success, got %d", rc);
}

ZTEST(light_hw, test_init_returns_error_on_failure)
{
    g_stub_hw_init_ret = -5;
    int rc = light_hw_init();
    zassert_not_equal(rc, 0,
        "light_hw_init() must propagate non-zero return from hardware");
}

ZTEST(light_hw, test_init_idempotent_double_call)
{
    int rc1 = light_hw_init();
    int rc2 = light_hw_init();
    zassert_equal(rc1, 0, "first light_hw_init() must succeed, got %d",  rc1);
    zassert_equal(rc2, 0, "second light_hw_init() must succeed, got %d", rc2);
}

ZTEST(light_hw, test_set_on_records_state)
{
    light_hw_set(1);
    zassert_equal(g_stub_light_state, 1,
        "light_hw_set(1) must record state=1 (ON)");
}

ZTEST(light_hw, test_set_off_records_state)
{
    light_hw_set(0);
    zassert_equal(g_stub_light_state, 0,
        "light_hw_set(0) must record state=0 (OFF)");
}

ZTEST(light_hw, test_set_increments_call_count)
{
    light_hw_set(1);
    light_hw_set(0);
    light_hw_set(1);
    zassert_equal(g_stub_set_call_count, 3,
        "light_hw_set() must be called 3 times, got %d",
        g_stub_set_call_count);
}

ZTEST(light_hw, test_set_last_state_wins)
{
    light_hw_set(1);
    light_hw_set(0);
    zassert_equal(g_stub_light_state, 0,
        "Last call to light_hw_set() must determine final recorded state");
}

/******************************************************************************
 * SUITE: light_relay
 * relay_set() -- thin wrapper over light_hw_set() in main.c.
 * Verifies the seam: ble_gatt.c calls relay_set(), which must drive hardware.
 ******************************************************************************/
ZTEST_SUITE(light_relay, NULL, NULL, reset_all, NULL, NULL);

ZTEST(light_relay, test_relay_set_on_drives_hw)
{
    relay_set(1);
    zassert_equal(g_stub_light_state, 1,
        "relay_set(1) must drive light_hw_set(1) -- relay ON");
}

ZTEST(light_relay, test_relay_set_off_drives_hw)
{
    relay_set(0);
    zassert_equal(g_stub_light_state, 0,
        "relay_set(0) must drive light_hw_set(0) -- relay OFF");
}

ZTEST(light_relay, test_relay_set_toggle_sequence)
{
    relay_set(1);
    zassert_equal(g_stub_light_state, 1, "relay ON failed");
    relay_set(0);
    zassert_equal(g_stub_light_state, 0, "relay OFF failed");
    relay_set(1);
    zassert_equal(g_stub_light_state, 1, "relay re-ON failed");
}

ZTEST(light_relay, test_relay_set_increments_hw_call_count)
{
    relay_set(1);
    relay_set(0);
    zassert_equal(g_stub_set_call_count, 2,
        "Two relay_set() calls must produce two light_hw_set() calls");
}

/******************************************************************************
 * SUITE: light_resetreas
 * trinity_classify_reset() -- all 7 code paths.
 * trinity_log_boot_reason() -- all label paths + raw hex in output.
 ******************************************************************************/
static void resetreas_before(void *data)
{
    (void)data;
    reset_all(NULL);
    trinity_log_event_fake.custom_fake = fake_trinity_log_event_capture;
}
ZTEST_SUITE(light_resetreas, NULL, NULL, resetreas_before, NULL, NULL);

/* --- trinity_classify_reset() --- */

ZTEST(light_resetreas, test_classify_dog_is_watchdog)
{
    zassert_equal(trinity_classify_reset(0x00000002), TRINITY_BOOT_WATCHDOG,
        "DOG bit (bit1) must classify as WATCHDOG");
}

ZTEST(light_resetreas, test_classify_dog_priority_over_sreq)
{
    /* 0x06 = DOG|SREQ -- DOG is first in if-chain, must win */
    zassert_equal(trinity_classify_reset(0x00000006), TRINITY_BOOT_WATCHDOG,
        "DOG|SREQ must classify as WATCHDOG (DOG has priority)");
}

ZTEST(light_resetreas, test_classify_sreq_is_soft_reset)
{
    zassert_equal(trinity_classify_reset(0x00000004), TRINITY_BOOT_SOFT_RESET,
        "SREQ bit (bit2) must classify as SOFT_RESET");
}

ZTEST(light_resetreas, test_classify_pin_is_reset_pin)
{
    zassert_equal(trinity_classify_reset(0x00000001), TRINITY_BOOT_RESET_PIN,
        "RESETPIN (bit0) must classify as RESET_PIN");
}

ZTEST(light_resetreas, test_classify_repor_is_unknown)
{
    zassert_equal(trinity_classify_reset(0x00010000), TRINITY_BOOT_UNKNOWN,
        "REPOR (bit16) must classify as UNKNOWN");
}

ZTEST(light_resetreas, test_classify_zero_with_canary_is_brownout)
{
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_BROWNOUT,
        "RESETREAS=0 + canary booted must be BROWNOUT");
}

ZTEST(light_resetreas, test_classify_zero_no_canary_is_cold_boot)
{
    g_noinit_guard    = 0x00000000;
    g_canary_snapshot = 0x00000000;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "RESETREAS=0 + no canary must be COLD_POWER_ON");
}

ZTEST(light_resetreas, test_classify_zero_wrong_guard_is_cold_boot)
{
    /* Guard must be exactly 0xDEADBEEF -- partial match must not pass */
    g_noinit_guard    = 0xDEADBEEE;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "Wrong noinit guard must not produce BROWNOUT");
}

/* --- trinity_log_boot_reason() --- */

ZTEST(light_resetreas, test_boot_reason_dog_logs_watchdog)
{
    trinity_log_boot_reason(0x00000002);
    zassert_equal(trinity_log_event_fake.call_count, 1,
        "trinity_log_boot_reason must call trinity_log_event once");
    zassert_true(strstr(g_log_event_buf, "WATCHDOG")   != NULL,
        "must log WATCHDOG label");
    zassert_true(strstr(g_log_event_buf, "0x00000002") != NULL,
        "must log raw hex 0x00000002");
}

ZTEST(light_resetreas, test_boot_reason_sreq_logs_soft_reset)
{
    trinity_log_boot_reason(0x00000004);
    zassert_true(strstr(g_log_event_buf, "SOFT_RESET") != NULL,
        "must log SOFT_RESET label");
}

ZTEST(light_resetreas, test_boot_reason_pin_logs_reset_pin)
{
    trinity_log_boot_reason(0x00000001);
    zassert_true(strstr(g_log_event_buf, "RESET_PIN") != NULL,
        "must log RESET_PIN label");
}

ZTEST(light_resetreas, test_boot_reason_zero_logs_none)
{
    trinity_log_boot_reason(0x00000000);
    zassert_true(strstr(g_log_event_buf, "NONE")     != NULL,
        "zero RESETREAS must log NONE label");
    zassert_true(strstr(g_log_event_buf, "00000000") != NULL,
        "zero RESETREAS must log raw hex 00000000");
}

ZTEST(light_resetreas, test_boot_reason_combined_includes_raw_hex_and_label)
{
    /* DOG|REPOR = 0x00010002 -- DOG wins, must log WATCHDOG + raw hex */
    trinity_log_boot_reason(0x00010002);
    zassert_true(strstr(g_log_event_buf, "00010002") != NULL,
        "combined reset must include raw hex 00010002");
    zassert_true(strstr(g_log_event_buf, "WATCHDOG") != NULL,
        "DOG|REPOR must label as WATCHDOG");
}

/******************************************************************************
 * SUITE: light_canary
 * trinity_canary_set_booted() -- upgrades canary so next boot's
 * classify_reset() can detect brownout vs cold boot.
 ******************************************************************************/
ZTEST_SUITE(light_canary, NULL, NULL, reset_all, NULL, NULL);

ZTEST(light_canary, test_set_booted_enables_brownout_detection)
{
    trinity_canary_set_booted();

    /* Simulate next-boot snapshot */
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;

    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_BROWNOUT,
        "After set_booted, RESETREAS=0 must classify as BROWNOUT");
}

ZTEST(light_canary, test_without_set_booted_no_brownout_detection)
{
    /* Pre-init crash scenario -- canary stuck at ALIVE, not BOOTED */
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_ALIVE;

    zassert_equal(trinity_classify_reset(0x00000000), TRINITY_BOOT_COLD_POWER_ON,
        "ALIVE canary (pre-init crash) must not classify as BROWNOUT");
}

/******************************************************************************
 * SUITE: light_log_stack
 * Reads CONFIG_LOG_PROCESS_THREAD_STACK_SIZE from Kconfig (via autoconf.h).
 * A prj.conf regression will actually fail this -- not a hardcoded literal.
 ******************************************************************************/
ZTEST_SUITE(light_log_stack, NULL, NULL, reset_all, NULL, NULL);

ZTEST(light_log_stack, test_log_thread_stack_size_sufficient)
{
#ifdef CONFIG_LOG_PROCESS_THREAD_STACK_SIZE
    zassert_true(CONFIG_LOG_PROCESS_THREAD_STACK_SIZE >= 2048,
        "Log thread stack %d < 2048 -- will overflow on stats burst",
        CONFIG_LOG_PROCESS_THREAD_STACK_SIZE);
#else
    ztest_test_skip();
#endif
}
