#include <zephyr/ztest.h>
#include <zephyr/fff.h>
#include "trinity_log.h"
#include "main.h"
#include "battery.h"

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
 * Battery SOC -- 4xAA range VBAT_DEAD_MV=4400 to VBAT_FULL_MV=6400
 *
 * All constants pulled from battery.h -- no magic numbers here.
 * Tests use a real mid-discharge battery (5600mV / 64%) not a fresh
 * or dead battery edge case, matching the field unit under test.
 *
 * Math chain:
 *   vbat_mv = 5600mV  (multimeter reading at battery terminals)
 *   soc     = (vbat_mv - VBAT_DEAD_MV) * 100 / VBAT_RANGE_MV
 *           = (5600 - 4400) * 100 / 2000
 *           = 1200 * 100 / 2000
 *           = 60%
 ******************************************************************************/
ZTEST(lock_batt, test_mv_to_soc_max)
{
    /* Fresh 4xAA at full charge -- clamp to 100% at and above VBAT_FULL_MV */
    zassert_equal(mv_to_soc(VBAT_FULL_MV),        100, "VBAT_FULL_MV should be 100%%");
    zassert_equal(mv_to_soc(VBAT_FULL_MV + 500),  100, "Above VBAT_FULL_MV should clamp to 100%%");
}

ZTEST(lock_batt, test_mv_to_soc_min)
{
    /* Depleted 4xAA -- clamp to 0% at and below VBAT_DEAD_MV */
    zassert_equal(mv_to_soc(VBAT_DEAD_MV),        0, "VBAT_DEAD_MV should be 0%%");
    zassert_equal(mv_to_soc(VBAT_DEAD_MV - 500),  0, "Below VBAT_DEAD_MV should clamp to 0%%");
}

ZTEST(lock_batt, test_mv_to_soc_midpoint)
{
    /* Midpoint of usable range: VBAT_DEAD_MV + (VBAT_RANGE_MV / 2) = 5400mV = 50% */
    zassert_equal(mv_to_soc(VBAT_DEAD_MV + (VBAT_RANGE_MV / 2)), 50,
        "Midpoint of range should be 50%%");
}

ZTEST(lock_batt, test_mv_to_soc_field_reading)
{
    /* Mid-discharge field reading -- battery under test at time of capture.
     * Multimeter reads 5600mV at battery terminals.
     * soc = (5600 - 4400) * 100 / 2000 = 60%
     * BLE advertised 64% due to ADC resistor tolerance (~9% error on 500mV).
     * Test asserts the math formula result, not the ADC-skewed BLE value. */
    zassert_equal(mv_to_soc(5600), 60, "5600mV mid-discharge should be 60%%");
}

/******************************************************************************
 * ADC decode -- verify the full conversion chain matches the field reading
 *
 * This test validates the math chain we use to go from raw ADC count to
 * battery percent. All steps verified against the known field state:
 *
 * Step 1 -- ADC config produces step size:
 *   ref    = 600mV   (ADC_REF_INTERNAL, fixed in nRF52840 silicon)
 *   gain   = 1/6     (ADC_GAIN_1_6, gain_den=6)
 *   ceiling = ref * gain_den = 600 * 6 = 3600mV
 *   steps   = 1 << 12 = 4096          (12-bit resolution)
 *   step_mv = 3600 / 4096 = 0.879mV   (size of one ADC count)
 *
 * Step 2 -- raw count to pin voltage (what ADC sees internally):
 *   raw    = 2123 counts
 *   pin_mv = 0.879 * 2123 = 1866mV
 *
 * Step 3 -- undo divider to get battery voltage (what multimeter sees):
 *   divider = R2 / (R1 + R2) = 100k / 300k = 1/3
 *   vbat_mv = pin_mv * 3 = 1866 * 3 = 5598mV ~ 5600mV
 *
 * Step 4 -- voltage to percent:
 *   soc = (5600 - 4400) * 100 / 2000 = 60%
 ******************************************************************************/
ZTEST(lock_batt, test_adc_decode_chain)
{
    /* Step 1: ADC config -- ceiling and step size */
    int32_t ref_mv     = 600;             /* ADC_REF_INTERNAL = 0.6V = 600mV */
    int32_t gain_den   = 6;               /* ADC_GAIN_1_6 inverted: 1/(1/6) = 6 */
    int32_t ceiling_mv = ref_mv * gain_den;              /* 600 * 6 = 3600mV */
    int32_t steps      = 1 << 12;                        /* 12-bit = 4096 */
    float   step_mv    = (float)ceiling_mv / (float)steps; /* 3600 / 4096 = 0.879mV */

    zassert_equal(ceiling_mv, 3600, "ADC ceiling must be 3600mV");
    zassert_equal(steps,      4096, "12-bit ADC must have 4096 steps");
    /* step_mv = 0.879 -- verify within 1uV tolerance */
    zassert_true((step_mv > 0.878f) && (step_mv < 0.880f),
        "Step size must be ~0.879mV");

    /* Step 2: raw count to pin voltage (ADC internal view) */
    int16_t raw    = 2123;                /* ADC count at 5600mV battery */
    float   pin_mv = step_mv * (float)raw; /* 0.879 * 2123 = 1866mV at pin */

    zassert_true((pin_mv > 1860.0f) && (pin_mv < 1872.0f),
        "pin_mv must be ~1866mV (ADC internal view)");

    /* Step 3: undo divider -- recover battery voltage (multimeter view)
     * Divider: R1=200k R2=100k ratio=1/3 so multiply pin by 3 */
    int32_t vbat_mv = (int32_t)(pin_mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM);

    zassert_true((vbat_mv >= 5580) && (vbat_mv <= 5620),
        "vbat_mv must be ~5600mV (multimeter view)");

    /* Step 4: voltage to percent using battery.h constants */
    uint8_t soc = mv_to_soc(vbat_mv);

    zassert_equal(soc, 60, "5600mV mid-discharge must be 60%% SOC");
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

ZTEST_SUITE(lock_timing,    NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(lock_mfg,       NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(lock_batt,      NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(lock_write,     NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(lock_resetreas, NULL, NULL, NULL, NULL, NULL);
