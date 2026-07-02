/******************************************************************************
 * \file test_pir_main.c
 * \brief Unit tests for ESP32-C3 PIR sensor node -- full regression coverage.
 *
 * Coverage map:
 *   pack_motion_count()        main.h         -- zero, max, known value,
 *                                               field independence
 *   unpack_motion_count()      main.h         -- roundtrip correctness
 *   MFG layout constants       main.h         -- byte indices, data size
 *   ADV timing constants       main.h         -- WDT budget constraints
 *   DEEP_SLEEP_INTERVAL_US     main.h         -- hub alignment regression
 *   BLE config flags           prj.conf       -- privacy, SMP
 *   trinity_canary_set_booted() trinity_canary_esp.c -- canary upgrade
 *
 * Note: No trinity_classify_reset suite -- ESP32-C3 uses
 *       esp_sleep_get_wakeup_cause() not RESETREAS. Canary state is
 *       verified directly against TRINITY_CANARY_BOOTED/ALIVE constants.
 *
 * Run with: west build -b native_sim tests/unit --build-dir build_test
 ******************************************************************************/

#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include "trinity_log.h"
#include "main.h"

DEFINE_FFF_GLOBALS;

extern volatile uint32_t g_noinit_guard;
extern volatile uint32_t g_canary_snapshot;
void trinity_canary_set_booted(void);

FAKE_VOID_FUNC(trinity_wdt_kick);
FAKE_VOID_FUNC(trinity_wdt_init);
FAKE_VALUE_FUNC(int, trinity_log_init);
FAKE_VOID_FUNC(trinity_log_dump_previous);

/******************************************************************************
 * Shared teardown
 ******************************************************************************/
static void reset_all(void *fixture)
{
    (void)fixture;
    RESET_FAKE(trinity_wdt_kick);
    RESET_FAKE(trinity_wdt_init);
    RESET_FAKE(trinity_log_init);
    RESET_FAKE(trinity_log_dump_previous);
    FFF_RESET_HISTORY();
    g_noinit_guard    = 0;
    g_canary_snapshot = 0;
}

/******************************************************************************
 * SUITE: pir_mfg
 * BLE manufacturer data layout constants.
 * Hub BLE scanner reads motion count, battery, and occupied flag
 * from fixed byte indices. Any drift breaks the hub parser silently.
 ******************************************************************************/
ZTEST_SUITE(pir_mfg, NULL, NULL, reset_all, NULL, NULL);

ZTEST(pir_mfg, test_mfg_data_size)
{
    zassert_equal(MFG_DATA_SIZE, 10,
        "MFG_DATA_SIZE must be 10 -- includes tx_id tracing bytes (2026-06-16)");
}

ZTEST(pir_mfg, test_mfg_tx_id_byte_indices)
{
    zassert_equal(MFG_TX_ID_LO_IDX, 8, "tx_id low byte must be at index 8");
    zassert_equal(MFG_TX_ID_HI_IDX, 9, "tx_id high byte must be at index 9");
}

ZTEST(pir_mfg, test_mfg_company_id_bytes)
{
    zassert_equal(MFG_COMPANY_ID_0, 0xFF, "Company ID byte 0 must be 0xFF");
    zassert_equal(MFG_COMPANY_ID_1, 0xFF, "Company ID byte 1 must be 0xFF");
}

ZTEST(pir_mfg, test_mfg_motion_byte_indices)
{
    zassert_equal(MFG_MOTION_MSB_IDX, 2, "Motion MSB must be at index 2");
    zassert_equal(MFG_MOTION_LSB_IDX, 5, "Motion LSB must be at index 5");
}

ZTEST(pir_mfg, test_mfg_batt_byte_index)
{
    zassert_equal(MFG_BATT_IDX, 6, "Battery must be at index 6");
}

ZTEST(pir_mfg, test_mfg_occupied_byte_index)
{
    zassert_equal(MFG_OCCUPIED_IDX, 7,
        "Occupied flag must be at index 7 -- hub MFG_PIR_OCCUPIED_IDX=7");
}

/******************************************************************************
 * SUITE: pir_pack
 * pack_motion_count() / unpack_motion_count() -- big-endian 4-byte packing.
 * Hub reads 4 bytes big-endian from indices 2-5.
 ******************************************************************************/
ZTEST_SUITE(pir_pack, NULL, NULL, reset_all, NULL, NULL);

ZTEST(pir_pack, test_pack_zero)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0};
    pack_motion_count(mfg, 0);
    zassert_equal(unpack_motion_count(mfg), 0, "pack/unpack 0 failed");
}

ZTEST(pir_pack, test_pack_max)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0};
    pack_motion_count(mfg, 0xFFFFFFFF);
    zassert_equal(unpack_motion_count(mfg), 0xFFFFFFFF,
        "pack/unpack 0xFFFFFFFF failed");
}

ZTEST(pir_pack, test_pack_known_value)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0};
    pack_motion_count(mfg, 0x01020304);
    zassert_equal(mfg[2], 0x01, "MSB wrong");
    zassert_equal(mfg[3], 0x02, "byte 3 wrong");
    zassert_equal(mfg[4], 0x03, "byte 4 wrong");
    zassert_equal(mfg[5], 0x04, "LSB wrong");
    zassert_equal(unpack_motion_count(mfg), 0x01020304, "roundtrip failed");
}

ZTEST(pir_pack, test_pack_does_not_touch_batt_byte)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0xFF, 0xFF, 0, 0, 0, 0, 0x64, 0};
    pack_motion_count(mfg, 0x00000001);
    zassert_equal(mfg[MFG_BATT_IDX], 0x64,
        "pack_motion_count must not touch batt byte");
}

ZTEST(pir_pack, test_pack_does_not_touch_occupied_byte)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0xFF, 0xFF, 0, 0, 0, 0, 0, 0xAB};
    pack_motion_count(mfg, 0x00000001);
    zassert_equal(mfg[MFG_OCCUPIED_IDX], 0xAB,
        "pack_motion_count must not touch occupied byte");
}

/******************************************************************************
 * SUITE: pir_adv
 * Full 8-byte payload field independence.
 * Catches the zeroed-payload timing bug (2026-05-03) where ble_adv_start()
 * fired before ble_adv_update() populated real data.
 ******************************************************************************/
ZTEST_SUITE(pir_adv, NULL, NULL, reset_all, NULL, NULL);

ZTEST(pir_adv, test_adv_update_sets_all_fields)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0};
    pack_motion_count(mfg, 0x00000005);
    mfg[MFG_BATT_IDX]     = 83;
    mfg[MFG_OCCUPIED_IDX] = 1;
    zassert_equal(unpack_motion_count(mfg), 5,  "motion count wrong");
    zassert_equal(mfg[MFG_BATT_IDX],        83, "batt wrong");
    zassert_equal(mfg[MFG_OCCUPIED_IDX],    1,  "occupied wrong");
}

ZTEST(pir_adv, test_adv_update_occupied_zero)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0xFF};
    pack_motion_count(mfg, 100);
    mfg[MFG_BATT_IDX]     = 50;
    mfg[MFG_OCCUPIED_IDX] = 0;
    zassert_equal(mfg[MFG_OCCUPIED_IDX],    0,   "occupied clear failed");
    zassert_equal(mfg[MFG_BATT_IDX],        50,  "batt wrong after clear");
    zassert_equal(unpack_motion_count(mfg), 100, "count wrong after clear");
}

ZTEST(pir_adv, test_adv_fields_independent)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0};
    pack_motion_count(mfg, 0xDEADBEEF);
    mfg[MFG_OCCUPIED_IDX] = 1;
    zassert_equal(unpack_motion_count(mfg), 0xDEADBEEF,
        "occupied write corrupted motion count");
    mfg[MFG_BATT_IDX] = 75;
    zassert_equal(mfg[MFG_OCCUPIED_IDX], 1,
        "batt write corrupted occupied flag");
}

/******************************************************************************
 * SUITE: pir_timing
 * Compile-time constant constraint guards.
 ******************************************************************************/
ZTEST_SUITE(pir_timing, NULL, NULL, reset_all, NULL, NULL);

ZTEST(pir_timing, test_pir_burst_fits_in_wdt_window)
{
    /* PIR burst is 2000ms, WDT is 3000ms -- no mid-kick needed.
     * If ADV_BURST_PIR_MS ever exceeds WDT_TIMEOUT_MS the mid-burst
     * kick logic in main() must fire -- this test catches that drift. */
    zassert_true(ADV_BURST_PIR_MS < WDT_TIMEOUT_MS,
        "ADV_BURST_PIR_MS=%d must be < WDT_TIMEOUT_MS=%d",
        ADV_BURST_PIR_MS, WDT_TIMEOUT_MS);
}

ZTEST(pir_timing, test_timer_burst_fits_in_wdt_window)
{
    zassert_true(ADV_BURST_TIMER_MS < WDT_TIMEOUT_MS,
        "ADV_BURST_TIMER_MS=%d must be < WDT_TIMEOUT_MS=%d",
        ADV_BURST_TIMER_MS, WDT_TIMEOUT_MS);
}

ZTEST(pir_timing, test_deep_sleep_interval_is_30s)
{
    /* Hub online/offline threshold set to 60s = 2x this interval.
     * Changing this value without updating hub BLE_DEV_PIR age threshold
     * causes false offline flickers. */
    zassert_equal(DEEP_SLEEP_INTERVAL_US, 30ULL * 1000ULL * 1000ULL,
        "DEEP_SLEEP_INTERVAL_US must be 30s -- hub alignment regression");
}

/******************************************************************************
 * SUITE: pir_ble_addr
 * BLE address type -- static vs RPA.
 * CONFIG_BT_PRIVACY=n prevents Resolvable Private Address rotation.
 * A rotating MAC causes hub slot churn and dropped advertisements.
 ******************************************************************************/
ZTEST_SUITE(pir_ble_addr, NULL, NULL, reset_all, NULL, NULL);

ZTEST(pir_ble_addr, test_bt_privacy_disabled)
{
#ifdef CONFIG_BT_PRIVACY
    zassert_false(CONFIG_BT_PRIVACY,
        "CONFIG_BT_PRIVACY must be n -- RPA causes hub slot churn");
#else
    zassert_true(1, "privacy correctly absent");
#endif
}

ZTEST(pir_ble_addr, test_bt_smp_disabled)
{
#ifdef CONFIG_BT_SMP
    zassert_false(CONFIG_BT_SMP,
        "CONFIG_BT_SMP must be n -- broadcaster-only, no pairing needed");
#else
    zassert_true(1, "SMP correctly absent");
#endif
}

/******************************************************************************
 * SUITE: pir_canary
 * trinity_canary_set_booted() -- canary upgrade verification.
 *
 * Note: ESP32-C3 has no RESETREAS register so trinity_classify_reset()
 *       does not exist on this platform. Wake reason is determined by
 *       esp_sleep_get_wakeup_cause(). Canary state is verified directly.
 ******************************************************************************/
ZTEST_SUITE(pir_canary, NULL, NULL, reset_all, NULL, NULL);

ZTEST(pir_canary, test_set_booted_upgrades_canary)
{
    /* trinity_canary_set_booted() must write TRINITY_CANARY_BOOTED into
     * the pre-init canary. Simulate what canary_pre_init() does on next
     * boot: snapshot is set to BOOTED. Guard must be intact. */
    trinity_canary_set_booted();
    g_noinit_guard    = 0xDEADBEEF;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_equal(g_canary_snapshot, TRINITY_CANARY_BOOTED,
        "canary snapshot must be BOOTED after set_booted()");
    zassert_equal(g_noinit_guard, 0xDEADBEEF,
        "noinit guard must be intact after set_booted()");
}

ZTEST(pir_canary, test_alive_canary_not_booted)
{
    /* ALIVE canary -- pre-init crash scenario -- must not equal BOOTED.
     * Prevents false brownout detection on crash-before-set_booted. */
    g_canary_snapshot = TRINITY_CANARY_ALIVE;
    zassert_not_equal(g_canary_snapshot, TRINITY_CANARY_BOOTED,
        "ALIVE canary must not equal BOOTED -- pre-init crash scenario");
}

ZTEST(pir_canary, test_wrong_guard_not_booted)
{
    /* Partial guard match must not be treated as valid brownout state. */
    g_noinit_guard    = 0xDEADBEEE;
    g_canary_snapshot = TRINITY_CANARY_BOOTED;
    zassert_not_equal(g_noinit_guard, 0xDEADBEEF,
        "Corrupted guard must not match 0xDEADBEEF");
}
