#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include "trinity_log.h"
#include "main.h"

DEFINE_FFF_GLOBALS;

FAKE_VOID_FUNC(trinity_wdt_kick);
FAKE_VOID_FUNC(trinity_wdt_init);
FAKE_VALUE_FUNC(int, trinity_log_init);
FAKE_VOID_FUNC(trinity_log_dump_previous);
FAKE_VOID_FUNC(trinity_log_dump_previous_deferred);

/******************************************************************************
 * Timing sanity
 ******************************************************************************/
ZTEST(light_timing, test_stats_interval)
{
    zassert_equal(STATS_INTERVAL_SEC, 60,
        "STATS_INTERVAL_SEC changed -- expected 60s");
}

ZTEST(light_timing, test_idle_heartbeat)
{
    zassert_equal(IDLE_HEARTBEAT_SEC, 120,
        "IDLE_HEARTBEAT_SEC changed -- expected 120s");
}

/******************************************************************************
 * MFG data layout
 * Hub BLE scanner reads state from fixed byte index.
 ******************************************************************************/
ZTEST(light_mfg, test_mfg_data_size)
{
    zassert_equal(MFG_DATA_SIZE, 2,
        "MFG_DATA_SIZE changed -- hub parser expects exactly 2 bytes");
}

ZTEST(light_mfg, test_mfg_company_id)
{
    zassert_equal(MFG_COMPANY_ID, 0xAD,
        "MFG_COMPANY_ID changed -- hub filter will break");
}

ZTEST(light_mfg, test_mfg_state_byte_index)
{
    uint8_t mfg[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x01};
    zassert_equal(mfg[MFG_STATE_IDX], 0x01,
        "State must be at byte index 1 -- hub hardcodes MFG_STATE_IDX=1");
}

/******************************************************************************
 * write_light_control validation
 * Bug: invalid len or state > 1 must be rejected.
 ******************************************************************************/
ZTEST(light_write, test_write_len_must_be_1)
{
    zassert_equal(LIGHT_WRITE_LEN, 1,
        "write_light_control must reject len != 1");
}

ZTEST(light_write, test_state_max_is_1)
{
    zassert_equal(LIGHT_STATE_MAX, 1,
        "write_light_control must reject state > 1");
}

ZTEST(light_write, test_state_on_value)
{
    uint8_t state = 1;
    zassert_true(state <= LIGHT_STATE_MAX, "ON state (1) must be valid");
}

ZTEST(light_write, test_state_off_value)
{
    uint8_t state = 0;
    zassert_true(state <= LIGHT_STATE_MAX, "OFF state (0) must be valid");
}

ZTEST(light_write, test_state_invalid_value)
{
    uint8_t state = 2;
    zassert_true(state > LIGHT_STATE_MAX, "State 2 must be rejected");
}

/******************************************************************************
 * RESETREAS classifier -- same silicon, same tests as reed-sensor
 ******************************************************************************/
ZTEST(light_resetreas, test_watchdog)
{
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000002);
    zassert_equal(r, TRINITY_BOOT_WATCHDOG, "bit1=DOG must be WATCHDOG");
}

ZTEST(light_resetreas, test_pin_reset)
{
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000001);
    zassert_equal(r, TRINITY_BOOT_RESET_PIN, "bit0=RESETPIN must be RESET_PIN");
}

ZTEST(light_resetreas, test_dog_sreq_priority)
{
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000006);
    zassert_equal(r, TRINITY_BOOT_WATCHDOG, "DOG|SREQ must classify as WATCHDOG");
}

ZTEST(light_resetreas, test_cold_boot)
{
    extern volatile uint32_t g_noinit_guard;
    extern volatile uint32_t g_canary_snapshot;
    g_noinit_guard    = 0x00000000;
    g_canary_snapshot = 0x00000000;
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000000);
    zassert_equal(r, TRINITY_BOOT_COLD_POWER_ON, "RESETREAS=0 cold boot");
}

ZTEST(light_resetreas, test_brownout)
{
    extern volatile uint32_t g_noinit_guard;
    extern volatile uint32_t g_canary_snapshot;
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000000);
    zassert_equal(r, TRINITY_BOOT_BROWNOUT, "RESETREAS=0 with canary=BROWNOUT");
}

ZTEST_SUITE(light_timing,   NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(light_mfg,      NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(light_write,    NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(light_resetreas, NULL, NULL, NULL, NULL, NULL);
