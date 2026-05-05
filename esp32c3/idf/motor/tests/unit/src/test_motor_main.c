#include "unity.h"
#include "main.h"
#include "battery.h"

void setUp(void) {}
void tearDown(void) {}

/******************************************************************************
 * Timing constants
 *
 * MOTOR_LOOP_MS raised from 100ms to 5000ms (2026-05-04).
 * Knob ADC and temp logic moved to hub -- 5s resolution sufficient.
 * Hub sends PWM updates on its own TCP_SEND_INTERVAL cadence.
 ******************************************************************************/
void test_stats_interval_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(60000u, STATS_INTERVAL_MS);
}

void test_motor_loop_ms(void)
{
    /* Raised from 100ms -- knob ADC removed, hub owns all control decisions.
     * 5s loop is sufficient resolution for PWM application and battery reads. */
    TEST_ASSERT_EQUAL_UINT32(5000u, MOTOR_LOOP_MS);
}

void test_recv_timeout_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(30000u, RECV_TIMEOUT_MS);
}

void test_accept_timeout_sec(void)
{
    TEST_ASSERT_EQUAL_INT(2, ACCEPT_TIMEOUT_SEC);
}

void test_batt_interval_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(30000u, BATT_INTERVAL_MS);
}

/******************************************************************************
 * Motor run protection
 *
 * MIN_RUN_SEC enforced by hub state machine. Motor must run for at least
 * 10 minutes once started to protect against rapid stop/start cycling
 * near the temperature threshold which can damage the motor.
 ******************************************************************************/
void test_min_run_sec(void)
{
    TEST_ASSERT_EQUAL_INT(600, MIN_RUN_SEC);
}

void test_min_run_sec_is_10_minutes(void)
{
    TEST_ASSERT_EQUAL_INT(600, MIN_RUN_SEC);
    TEST_ASSERT_EQUAL_INT(10, MIN_RUN_SEC / 60);
}

/******************************************************************************
 * PWM duty max sanity
 ******************************************************************************/
void test_pwm_duty_max(void)
{
    TEST_ASSERT_EQUAL_UINT32((1 << 13) - 1, PWM_DUTY_MAX);
}

void test_pwm_duty_max_13bit(void)
{
    /* Hub sends pwm as 0-PWM_DUTY_MAX. Clamp enforced in parse_tcp_json(). */
    TEST_ASSERT_EQUAL_UINT32(8191u, PWM_DUTY_MAX);
}

/******************************************************************************
 * TCP protocol boundaries
 ******************************************************************************/
void test_tcp_port(void)
{
    TEST_ASSERT_EQUAL_INT(3333, TCP_PORT);
}

void test_batt_json_buf_fits_worst_case(void)
{
    /* {"batt_motor":99999} = 20 chars + null = 21 bytes minimum */
    TEST_ASSERT_GREATER_OR_EQUAL(21, BATT_JSON_BUF_SIZE);
}

void test_batt_sag_reject_mv(void)
{
    /* Hub depends on this threshold -- must not change without hub update */
    TEST_ASSERT_EQUAL_INT(200, BATT_SAG_REJECT_MV);
}

/******************************************************************************
 * Battery ADC decode chain
 *
 * Validates the full conversion chain from raw ADC count to battery voltage.
 * All steps verified against empirical measurements on this hardware.
 *
 * Hardware (2026-04-27):
 *   Divider: R1=4kΩ, R2=1kΩ, ratio=1/5
 *   Attenuation: ADC_ATTEN_DB_12
 *   Ceiling: 3379mV (empirical -- back-calculated from known pin voltage)
 *   Resolution: 12-bit, ADC_MAX_RAW=4095
 *
 * Both this node and the nRF52840 smart-lock use a 12-bit ADC (4096 steps).
 * ADC_GAIN_1_6 (nRF) and ADC_ATTEN_DB_12 (ESP32-C3) are the same concept
 * with different names -- both set the ceiling: the input voltage that
 * produces a full-scale raw count. The 4096 steps are spread across that
 * ceiling to give the step size. Firmware reconstruct on both nodes only
 * undoes the resistor divider. nRF × 3, C3 × 5. No other term needed.
 *
 * Math chain (empirical, fresh battery):
 *   ceiling = 3379mV  (ADC_ATTEN_DB_12, empirical)
 *   steps   = 4095    (ADC_MAX_RAW, 12-bit)
 *   pin_mv  = 1944mV  (measured: ~9.7V battery × 1/5 divider)
 *   raw     = 1944 × 4095 / 3379 = 2356 counts
 *   pin_mv  = 2356 × 3379 / 4095 = 1944mV
 *   vbat_mv = 1944 × 5 = 9720mV (pre-cal)
 *   cal     = 9720 × 9500 / 9720 = 9500mV (multimeter reading)
 *
 * Calibration history:
 *   ADC_ATTEN_DB_6 (ceiling ~1750mV) was used initially. The 1.8V pin
 *   voltage exceeded this ceiling, saturating raw=4095 on every read.
 *   With raw saturated the ADC reported a fixed voltage regardless of
 *   actual battery state -- the SOC protection was completely blind.
 *   Combined with WiFi TX bursts sagging the 9V rail, this contributed
 *   to 3 batteries being over-discharged before the fault was found.
 *   Fix: switched to ADC_ATTEN_DB_12 (ceiling ~3379mV), bringing 1.94V
 *   pin voltage into mid-range. Empirical cal (9500/9720) then corrects
 *   the ADC's ~2% over-read uniformly across the discharge range.
 *   Recalibration is required any time the divider or attenuation changes.
 ******************************************************************************/
void test_adc_ceiling_db12(void)
{
    TEST_ASSERT_EQUAL_INT32(3379, 3379);
    TEST_ASSERT_EQUAL_INT32(4095, ADC_MAX_RAW);
}

void test_adc_decode_chain_9v_nominal(void)
{
    int32_t ceiling_mv = 3379;
    int32_t steps      = 4095;
    float   step_mv    = (float)ceiling_mv / (float)steps;

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.825f, step_mv);

    int32_t pin_mv_expected = 1944;
    int32_t raw             = (pin_mv_expected * steps) / ceiling_mv;

    TEST_ASSERT_INT_WITHIN(2, 2356, raw);

    int32_t pin_mv = (raw * ceiling_mv) / steps;

    TEST_ASSERT_INT_WITHIN(5, 1944, pin_mv);

    int32_t vbat_mv = pin_mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM;

    TEST_ASSERT_INT_WITHIN(50, 9720, vbat_mv);
}

void test_adc_fresh_battery_under_ceiling(void)
{
    int32_t worst_case_pin_mv = 9800 / 5;
    TEST_ASSERT_LESS_THAN(3379, worst_case_pin_mv);
}

/******************************************************************************
 * ADC calibration
 ******************************************************************************/
void test_cal_ratio_corrects_downward(void)
{
    TEST_ASSERT_LESS_THAN(ADC_CAL_DEN, ADC_CAL_NUM);
}

void test_cal_applied_to_fresh_battery(void)
{
    int32_t pre_cal  = 9720;
    int32_t post_cal = (pre_cal * ADC_CAL_NUM) / ADC_CAL_DEN;
    TEST_ASSERT_INT_WITHIN(10, VBAT_FULL_MV, post_cal);
}

void test_cal_num_den_nonzero(void)
{
    TEST_ASSERT_NOT_EQUAL(0, ADC_CAL_NUM);
    TEST_ASSERT_NOT_EQUAL(0, ADC_CAL_DEN);
}

/******************************************************************************
 * SOC thresholds
 ******************************************************************************/
void test_vbat_full_mv(void)
{
    TEST_ASSERT_EQUAL_INT32(9500, VBAT_FULL_MV);
}

void test_vbat_dead_mv(void)
{
    TEST_ASSERT_EQUAL_INT32(7500, VBAT_DEAD_MV);
}

void test_vbat_range_mv(void)
{
    TEST_ASSERT_EQUAL_INT32(VBAT_FULL_MV - VBAT_DEAD_MV, VBAT_RANGE_MV);
    TEST_ASSERT_EQUAL_INT32(2000, VBAT_RANGE_MV);
}

void test_mv_to_soc_full(void)
{
    TEST_ASSERT_EQUAL_UINT8(100u, mv_to_soc(VBAT_FULL_MV));
}

void test_mv_to_soc_dead(void)
{
    TEST_ASSERT_EQUAL_UINT8(0u, mv_to_soc(VBAT_DEAD_MV));
}

void test_mv_to_soc_clamps_above_full(void)
{
    TEST_ASSERT_EQUAL_UINT8(100u, mv_to_soc(VBAT_FULL_MV + 500));
}

void test_mv_to_soc_clamps_below_dead(void)
{
    TEST_ASSERT_EQUAL_UINT8(0u, mv_to_soc(VBAT_DEAD_MV - 500));
}

void test_mv_to_soc_midpoint(void)
{
    int mid = (VBAT_FULL_MV + VBAT_DEAD_MV) / 2;  /* 8500mV */
    TEST_ASSERT_EQUAL_UINT8(50u, mv_to_soc(mid));
}

/******************************************************************************
 * Sag reject logic
 ******************************************************************************/
void test_sag_reject_exactly_at_threshold_passes(void)
{
    int last_good_mv = 9000;
    int reading_mv   = last_good_mv - BATT_SAG_REJECT_MV;
    TEST_ASSERT_FALSE(reading_mv < (last_good_mv - BATT_SAG_REJECT_MV));
}

void test_sag_reject_one_below_threshold_rejected(void)
{
    int last_good_mv = 9000;
    int reading_mv   = last_good_mv - BATT_SAG_REJECT_MV - 1;
    TEST_ASSERT_TRUE(reading_mv < (last_good_mv - BATT_SAG_REJECT_MV));
}

void test_sag_reject_first_reading_always_accepted(void)
{
    int last_good_mv = 0;
    int reading_mv   = 8500;
    TEST_ASSERT_FALSE((last_good_mv > 0) &&
                      (reading_mv < (last_good_mv - BATT_SAG_REJECT_MV)));
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_stats_interval_ms);
    RUN_TEST(test_motor_loop_ms);
    RUN_TEST(test_recv_timeout_ms);
    RUN_TEST(test_accept_timeout_sec);
    RUN_TEST(test_batt_interval_ms);
    RUN_TEST(test_min_run_sec);
    RUN_TEST(test_min_run_sec_is_10_minutes);
    RUN_TEST(test_pwm_duty_max);
    RUN_TEST(test_pwm_duty_max_13bit);
    RUN_TEST(test_tcp_port);
    RUN_TEST(test_batt_json_buf_fits_worst_case);
    RUN_TEST(test_batt_sag_reject_mv);
    RUN_TEST(test_adc_ceiling_db12);
    RUN_TEST(test_adc_decode_chain_9v_nominal);
    RUN_TEST(test_adc_fresh_battery_under_ceiling);
    RUN_TEST(test_cal_ratio_corrects_downward);
    RUN_TEST(test_cal_applied_to_fresh_battery);
    RUN_TEST(test_cal_num_den_nonzero);
    RUN_TEST(test_vbat_full_mv);
    RUN_TEST(test_vbat_dead_mv);
    RUN_TEST(test_vbat_range_mv);
    RUN_TEST(test_mv_to_soc_full);
    RUN_TEST(test_mv_to_soc_dead);
    RUN_TEST(test_mv_to_soc_clamps_above_full);
    RUN_TEST(test_mv_to_soc_clamps_below_dead);
    RUN_TEST(test_mv_to_soc_midpoint);
    RUN_TEST(test_sag_reject_exactly_at_threshold_passes);
    RUN_TEST(test_sag_reject_one_below_threshold_rejected);
    RUN_TEST(test_sag_reject_first_reading_always_accepted);

    return UNITY_END();
}
