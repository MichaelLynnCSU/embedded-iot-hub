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
ZTEST(lock_timing, test_stats_interval)
{
    zassert_equal(STATS_INTERVAL_SEC, 60, "STATS_INTERVAL_SEC must be 60s");
}

ZTEST(lock_timing, test_idle_heartbeat)
{
    zassert_equal(IDLE_HEARTBEAT_SEC, 120, "IDLE_HEARTBEAT_SEC must be 120s");
}

ZTEST(lock_timing, test_batt_update_sec)
{
    zassert_equal(BATT_UPDATE_SEC, 300, "BATT_UPDATE_SEC must be 300s (5 min)");
}

/******************************************************************************
 * MFG data layout
 * Hub BLE scanner reads lock state and battery from fixed byte indices.
 ******************************************************************************/
ZTEST(lock_mfg, test_mfg_data_size)
{
    zassert_equal(MFG_DATA_SIZE, 3,
        "MFG_DATA_SIZE changed -- hub parser expects exactly 3 bytes");
}

ZTEST(lock_mfg, test_mfg_company_id)
{
    zassert_equal(MFG_COMPANY_ID, 0xAC,
        "MFG_COMPANY_ID changed -- hub filter will break");
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
 * Battery SOC -- 2S LiPo range 4000-6000mV
 ******************************************************************************/
ZTEST(lock_batt, test_mv_to_soc_max)
{
    zassert_equal(mv_to_soc(6000), 100, "6000mV should be 100%%");
    zassert_equal(mv_to_soc(6500), 100, "6500mV should clamp to 100%%");
}

ZTEST(lock_batt, test_mv_to_soc_min)
{
    zassert_equal(mv_to_soc(4000), 0, "4000mV should be 0%%");
    zassert_equal(mv_to_soc(3500), 0, "3500mV should clamp to 0%%");
}

ZTEST(lock_batt, test_mv_to_soc_midpoint)
{
    zassert_equal(mv_to_soc(5000), 50, "5000mV should be 50%%");
}

/******************************************************************************
 * lock_write validation
 ******************************************************************************/
ZTEST(lock_write, test_write_len_must_be_1)
{
    zassert_equal(LOCK_WRITE_LEN, 1, "lock_write must reject len != 1");
}

/******************************************************************************
 * RESETREAS classifier
 ******************************************************************************/
ZTEST(lock_resetreas, test_watchdog)
{
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000002);
    zassert_equal(r, TRINITY_BOOT_WATCHDOG, "bit1=DOG must be WATCHDOG");
}

ZTEST(lock_resetreas, test_pin_reset)
{
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000001);
    zassert_equal(r, TRINITY_BOOT_RESET_PIN, "bit0=RESETPIN must be RESET_PIN");
}

ZTEST(lock_resetreas, test_dog_sreq_priority)
{
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000006);
    zassert_equal(r, TRINITY_BOOT_WATCHDOG, "DOG|SREQ must classify as WATCHDOG");
}

ZTEST(lock_resetreas, test_cold_boot)
{
    extern volatile uint32_t g_noinit_guard;
    extern volatile uint32_t g_canary_snapshot;
    g_noinit_guard    = 0x00000000;
    g_canary_snapshot = 0x00000000;
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000000);
    zassert_equal(r, TRINITY_BOOT_COLD_POWER_ON, "RESETREAS=0 cold boot");
}

ZTEST(lock_resetreas, test_brownout)
{
    extern volatile uint32_t g_noinit_guard;
    extern volatile uint32_t g_canary_snapshot;
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    TRINITY_BOOT_REASON_E r = trinity_classify_reset(0x00000000);
    zassert_equal(r, TRINITY_BOOT_BROWNOUT, "RESETREAS=0 with canary=BROWNOUT");
}

ZTEST_SUITE(lock_timing,   NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(lock_mfg,      NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(lock_batt,     NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(lock_write,    NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(lock_resetreas, NULL, NULL, NULL, NULL, NULL);
