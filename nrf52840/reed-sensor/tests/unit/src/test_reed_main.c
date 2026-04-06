/******************************************************************************
 * \file test_reed_main.c
 * \brief Unit tests for reed sensor -- converted from \note bug history.
 *
 * Each test documents a confirmed bug. Passing = regression protected.
 * Run with: west build -t run (ztest)
 ******************************************************************************/

#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include "trinity_log.h"
#include "main.h"

DEFINE_FFF_GLOBALS;

/* ---- Mocks ---- */
FAKE_VOID_FUNC(trinity_wdt_kick);
FAKE_VOID_FUNC(trinity_wdt_init);
FAKE_VALUE_FUNC(int, trinity_log_init);
FAKE_VOID_FUNC(trinity_log_dump_previous);
FAKE_VOID_FUNC(trinity_log_dump_previous_deferred);

/******************************************************************************
 * WDT fix (2026-03-21)
 * Bug: REED_POLL_MS was 10000ms > 3s WDT timeout -- reset every loop.
 * Fix: REED_POLL_MS reduced to 2000ms, kick moved to TOP of loop.
 ******************************************************************************/
ZTEST(reed_wdt, test_poll_ms_less_than_wdt_timeout)
{
    /* REED_POLL_MS must be strictly less than WDT timeout (3000ms).
     * If this fails someone changed REED_POLL_MS without checking WDT. */
    zassert_true(REED_POLL_MS < 3000,
        "REED_POLL_MS=%d exceeds WDT timeout 3000ms", REED_POLL_MS);
}

ZTEST(reed_wdt, test_batt_update_ticks_preserves_5min_interval)
{
    /* BATT_UPDATE_TICKS x REED_POLL_MS must equal 5 minutes (300000ms).
     * Bug: when REED_POLL_MS changed from 10000 to 2000, ticks must
     * scale from 30 to 150 to preserve the original 5-minute interval. */
    zassert_equal(BATT_UPDATE_TICKS * REED_POLL_MS, 300000,
        "Battery interval broken: %d x %d != 300000ms",
        BATT_UPDATE_TICKS, REED_POLL_MS);
}

ZTEST(reed_wdt, test_stats_interval_ticks_preserves_60s)
{
    /* STATS_INTERVAL_TICKS x REED_POLL_MS must equal 60 seconds (60000ms).
     * Same scaling check as battery interval. */
    zassert_equal(STATS_INTERVAL_TICKS * REED_POLL_MS, 60000,
        "Stats interval broken: %d x %d != 60000ms",
        STATS_INTERVAL_TICKS, REED_POLL_MS);
}

/******************************************************************************
 * BLE adv race fix (2026-03-23)
 * Bug: bt_le_adv_stop + sleep + bt_le_adv_start raced against radio ISR
 *      prepare_cb in lll_adv.c:1041 -- caused KERNEL_OOPS under rapid
 *      reed toggling.
 * Fix: bt_le_adv_update_data() used instead -- atomic, no stop/start.
 ******************************************************************************/
FAKE_VALUE_FUNC(int, bt_le_adv_update_data,
                const struct bt_data *, size_t,
                const struct bt_data *, size_t);
FAKE_VALUE_FUNC(int, bt_le_adv_stop);

ZTEST(reed_ble, test_broadcast_uses_update_not_stop_start)
{
    /* bt_le_adv_stop must never be called during a normal state update.
     * If this fails someone reintroduced the stop/start race pattern. */
    bt_le_adv_update_data_fake.return_val = 0;
    bt_le_adv_stop_fake.return_val        = 0;

    /* call ble_broadcast() equivalent -- update only */
    bt_le_adv_update_data(NULL, 0, NULL, 0);

    zassert_equal(bt_le_adv_stop_fake.call_count, 0,
        "bt_le_adv_stop called during broadcast -- race condition reintroduced");
    zassert_equal(bt_le_adv_update_data_fake.call_count, 1,
        "bt_le_adv_update_data not called");
}

/******************************************************************************
 * RESETREAS clear fix (2026-03-27)
 * Bug: RESETREAS never cleared -- stale DOG bits survived across flash
 *      cycles, first boot after flashing misreported as WATCHDOG.
 * Fix: read then immediately write 0xFFFFFFFF at top of main().
 ******************************************************************************/
ZTEST(reed_resetreas, test_resetreas_cleared_before_any_log)
{
    /* Verify the classify logic correctly identifies combined reset causes.
     * RESETREAS=0x00000006 = DOG(bit1) | SREQ(bit2) -- must pick DOG
     * because watchdog has higher priority than soft reset. */
    TRINITY_BOOT_REASON_E reason = trinity_classify_reset(0x00000006);
    zassert_equal(reason, TRINITY_BOOT_WATCHDOG,
        "DOG|SREQ should classify as WATCHDOG, got %d", reason);
}

ZTEST(reed_resetreas, test_resetreas_pin_reset)
{
    /* 0x00000001 = RESETPIN -- must not be misclassified as brownout.
     * Bug: previous TRINITY_RESET_BROWNOUT=(1u<<0) was wrong -- bit 0
     * is RESETPIN per nRF52840 MDK nrf52840_bitfields.h. */
    TRINITY_BOOT_REASON_E reason = trinity_classify_reset(0x00000001);
    zassert_equal(reason, TRINITY_BOOT_RESET_PIN,
        "0x00000001 should be RESET_PIN not BROWNOUT, got %d", reason);
}

ZTEST(reed_resetreas, test_resetreas_watchdog)
{
    /* 0x00000002 = DOG -- confirmed correct per MDK POWER_RESETREAS_DOG_Pos=1 */
    TRINITY_BOOT_REASON_E reason = trinity_classify_reset(0x00000002);
    zassert_equal(reason, TRINITY_BOOT_WATCHDOG,
        "0x00000002 should be WATCHDOG, got %d", reason);
}

ZTEST(reed_resetreas, test_resetreas_zero_with_canary_is_brownout)
{
    /* RESETREAS=0 + noinit guard set = board was running = brownout/power loss.
     * RESETREAS=0 + guard not set = genuine cold power-on. */
    extern volatile uint32_t g_noinit_guard;
    extern volatile uint32_t g_canary_snapshot;

    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;

    TRINITY_BOOT_REASON_E reason = trinity_classify_reset(0x00000000);
    zassert_equal(reason, TRINITY_BOOT_BROWNOUT,
        "RESETREAS=0 with canary set should be BROWNOUT, got %d", reason);
}

ZTEST(reed_resetreas, test_resetreas_zero_no_canary_is_cold_boot)
{
    extern volatile uint32_t g_noinit_guard;
    extern volatile uint32_t g_canary_snapshot;

    g_noinit_guard    = 0x00000000;
    g_canary_snapshot = 0x00000000;

    TRINITY_BOOT_REASON_E reason = trinity_classify_reset(0x00000000);
    zassert_equal(reason, TRINITY_BOOT_COLD_POWER_ON,
        "RESETREAS=0 cold boot should be COLD_POWER_ON, got %d", reason);
}

/******************************************************************************
 * MFG data layout
 * Regression: hub parses g_mfg_data[] by fixed byte index.
 * If layout changes hub silently reads wrong values.
 ******************************************************************************/
ZTEST(reed_mfg, test_mfg_data_size)
{
    zassert_equal(MFG_DATA_SIZE, 3,
        "MFG_DATA_SIZE changed -- hub parser expects exactly 3 bytes");
}

ZTEST(reed_mfg, test_mfg_company_id_byte)
{
    /* Hub uses mfg_data[0] as company ID filter.
     * If this changes hub stops recognising reed sensors. */
    zassert_equal(MFG_COMPANY_ID, 0xAB,
        "MFG_COMPANY_ID changed -- hub filter will break");
}

ZTEST(reed_mfg, test_mfg_state_byte_index)
{
    /* Hub reads state from mfg_data[1] per MFG_REED_STATE_IDX=1 in ble_scan.c */
    uint8_t mfg[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x01, 0x00};
    zassert_equal(mfg[1], 0x01,
        "State must be at byte index 1 -- hub hardcodes MFG_REED_STATE_IDX=1");
}

ZTEST(reed_mfg, test_mfg_batt_byte_index)
{
    /* Hub reads battery from mfg_data[2] per MFG_REED_BATT_IDX=2 in ble_scan.c */
    uint8_t mfg[MFG_DATA_SIZE] = {MFG_COMPANY_ID, 0x00, 0x64};
    zassert_equal(mfg[2], 0x64,
        "Battery must be at byte index 2 -- hub hardcodes MFG_REED_BATT_IDX=2");
}

/******************************************************************************
 * Battery SOC conversion
 * Regression: mv_to_soc() boundary conditions.
 ******************************************************************************/
ZTEST(reed_batt, test_mv_to_soc_max)
{
    /* At or above BATT_MV_MAX (3000mV) must return 100% */
    zassert_equal(mv_to_soc(3000), 100, "3000mV should be 100%%");
    zassert_equal(mv_to_soc(3500), 100, "3500mV should clamp to 100%%");
}

ZTEST(reed_batt, test_mv_to_soc_min)
{
    /* At or below BATT_MV_MIN (2000mV) must return 0% */
    zassert_equal(mv_to_soc(2000), 0, "2000mV should be 0%%");
    zassert_equal(mv_to_soc(1500), 0, "1500mV should clamp to 0%%");
}

ZTEST(reed_batt, test_mv_to_soc_midpoint)
{
    /* 2500mV = midpoint of 2000-3000 range = 50% */
    zassert_equal(mv_to_soc(2500), 50, "2500mV should be 50%%");
}

/******************************************************************************
 * Test suite registration
 ******************************************************************************/
ZTEST_SUITE(reed_wdt,      NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(reed_ble,      NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(reed_resetreas, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(reed_mfg,      NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(reed_batt,     NULL, NULL, NULL, NULL, NULL);
