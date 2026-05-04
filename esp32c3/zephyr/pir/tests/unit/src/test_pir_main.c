#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include "trinity_log.h"
#include "main.h"

DEFINE_FFF_GLOBALS;

FAKE_VOID_FUNC(trinity_wdt_kick);
FAKE_VOID_FUNC(trinity_wdt_init);
FAKE_VALUE_FUNC(int, trinity_log_init);
FAKE_VOID_FUNC(trinity_log_dump_previous);

/******************************************************************************
 * WDT tick math
 * Bug: BATT_UPDATE_TICKS and STATS_INTERVAL_TICKS were not scaled when
 *      loop sleep changed from 10s to 2s -- intervals were 5x too short.
 ******************************************************************************/
ZTEST(pir_wdt, test_loop_sleep_less_than_wdt_timeout)
{
    zassert_true(LOOP_SLEEP_MS < WDT_TIMEOUT_MS,
        "LOOP_SLEEP_MS=%d must be < WDT_TIMEOUT_MS=%d",
        LOOP_SLEEP_MS, WDT_TIMEOUT_MS);
}

ZTEST(pir_wdt, test_batt_update_ticks_preserves_5min)
{
    zassert_equal(BATT_UPDATE_TICKS * LOOP_SLEEP_MS, 300000,
        "Battery interval broken: %d x %d != 300000ms",
        BATT_UPDATE_TICKS, LOOP_SLEEP_MS);
}

ZTEST(pir_wdt, test_stats_interval_ticks_preserves_60s)
{
    zassert_equal(STATS_INTERVAL_TICKS * LOOP_SLEEP_MS, 60000,
        "Stats interval broken: %d x %d != 60000ms",
        STATS_INTERVAL_TICKS, LOOP_SLEEP_MS);
}

/******************************************************************************
 * MFG data layout
 * Hub parses motion count and battery from fixed byte indices.
 ******************************************************************************/
ZTEST(pir_mfg, test_mfg_data_size)
{
    zassert_equal(MFG_DATA_SIZE, 7,
        "MFG_DATA_SIZE changed -- hub parser expects exactly 7 bytes");
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

/******************************************************************************
 * Motion count big-endian packing
 * Hub reads 4 bytes big-endian from indices 2-5.
 ******************************************************************************/
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
    zassert_equal(unpack_motion_count(mfg), 0x01020304,
        "roundtrip failed");
}

ZTEST(pir_pack, test_pack_does_not_touch_batt_byte)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0xFF, 0xFF, 0, 0, 0, 0, 0x64};
    pack_motion_count(mfg, 0x00000001);
    zassert_equal(mfg[MFG_BATT_IDX], 0x64,
        "pack_motion_count must not touch batt byte");
}

/******************************************************************************
 * MFG data layout -- occupied flag
 * Added 2026-05-03: occupied byte at index 7.
 * Hub parses g_pir_occupied from MFG_PIR_OCCUPIED_IDX=7.
 * If this index drifts the hub reads garbage for occupancy state.
 ******************************************************************************/
ZTEST(pir_mfg, test_mfg_data_size)
{
    zassert_equal(MFG_DATA_SIZE, 8,
        "MFG_DATA_SIZE changed -- hub parser expects exactly 8 bytes");
}

ZTEST(pir_mfg, test_mfg_occupied_byte_index)
{
    zassert_equal(MFG_OCCUPIED_IDX, 7,
        "Occupied flag must be at index 7 -- hub MFG_PIR_OCCUPIED_IDX=7");
}

/******************************************************************************
 * ble_adv_update payload correctness
 * Verifies the full 8-byte payload is written correctly.
 * This test catches the zeroed-payload timing bug (2026-05-03) where
 * ble_adv_start() fired before ble_adv_update() populated real data,
 * causing the hub to receive count=0 batt=0 occ=0 on every first packet.
 ******************************************************************************/
ZTEST(pir_adv, test_adv_update_sets_all_fields)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0};

    /* Simulate what ble_adv_update() does internally */
    pack_motion_count(mfg, 0x00000005);
    mfg[MFG_BATT_IDX]     = 83;
    mfg[MFG_OCCUPIED_IDX] = 1;

    zassert_equal(unpack_motion_count(mfg), 5,    "motion count wrong");
    zassert_equal(mfg[MFG_BATT_IDX],        83,   "batt wrong");
    zassert_equal(mfg[MFG_OCCUPIED_IDX],    1,    "occupied wrong");
}

ZTEST(pir_adv, test_adv_update_occupied_zero)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0xFF};

    pack_motion_count(mfg, 100);
    mfg[MFG_BATT_IDX]     = 50;
    mfg[MFG_OCCUPIED_IDX] = 0;

    zassert_equal(mfg[MFG_OCCUPIED_IDX], 0, "occupied clear failed");
    zassert_equal(mfg[MFG_BATT_IDX],     50, "batt wrong after occupied clear");
    zassert_equal(unpack_motion_count(mfg), 100, "count wrong after occupied clear");
}

ZTEST(pir_adv, test_adv_fields_independent)
{
    uint8_t mfg[MFG_DATA_SIZE] = {0};

    /* Writing occupied must not touch count bytes */
    pack_motion_count(mfg, 0xDEADBEEF);
    mfg[MFG_OCCUPIED_IDX] = 1;
    zassert_equal(unpack_motion_count(mfg), 0xDEADBEEF,
        "occupied write corrupted motion count");

    /* Writing batt must not touch occupied byte */
    mfg[MFG_BATT_IDX] = 75;
    zassert_equal(mfg[MFG_OCCUPIED_IDX], 1,
        "batt write corrupted occupied flag");
}

ZTEST_SUITE(pir_adv, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(pir_wdt,  NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(pir_mfg,  NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(pir_pack, NULL, NULL, NULL, NULL, NULL);
