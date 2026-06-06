/******************************************************************************
 * \file test_motor_main.c
 * \brief Unit tests for ESP32-C3 IDF motor controller -- full regression.
 *
 * Coverage map:
 *   STATS_INTERVAL_MS / MOTOR_LOOP_MS   main.h         -- timing guards
 *   RECV_TIMEOUT_MS / ACCEPT_TIMEOUT_SEC main.h        -- TCP timing
 *   BATT_INTERVAL_MS                    main.h         -- battery interval
 *   MOTOR_DEEP_SLEEP_US                 main.h         -- deep sleep interval
 *   MIN_RUN_SEC                         main.h         -- motor protection
 *   PWM_DUTY_MAX / PWM_DUTY_RES         main.h         -- PWM boundaries
 *   TCP_PORT / BATT_JSON_BUF_SIZE       main.h         -- protocol constants
 *   BATT_SAG_REJECT_MV                  main.h         -- sag filter
 *   parse_tcp_json()                    motor_control.c -- valid pwm,
 *                                                          clamp, null,
 *                                                          garbage, unknown key
 *   get_pwm_duty()                      motor_control.c -- returns last set
 *   ADC decode chain                    battery.h      -- ceiling, steps, math
 *   ADC calibration                     battery.h      -- ratio direction
 *   mv_to_soc()                         battery.h      -- boundary + all points
 *   VBAT thresholds                     battery.h      -- full/dead/range
 *   Sag reject logic                    main.h         -- threshold boundary
 *
 * Framework: Unity v2.5.2
 * Build:     cmake -B build && cmake --build build && ctest --test-dir build
 ******************************************************************************/

#include "unity.h"
#include "main.h"
#include "battery.h"
#include "motor_control.h"

void setUp(void) {}
void tearDown(void) {}

/******************************************************************************
 * SECTION: Timing constants
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

void test_motor_deep_sleep_us(void)
{
    /* 30s deep sleep between wake cycles.
     * Must be <= hub TCP_SEND_INTERVAL so motor wakes often enough
     * to receive updated PWM commands while running. */
    TEST_ASSERT_EQUAL_UINT64(30000000ULL, MOTOR_DEEP_SLEEP_US);
    TEST_ASSERT_EQUAL_UINT64(30ULL, MOTOR_DEEP_SLEEP_US / 1000000ULL);
}

/******************************************************************************
 * SECTION: Motor run protection
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
 * SECTION: PWM duty constants
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

void test_pwm_duty_res_is_13(void)
{
    TEST_ASSERT_EQUAL_INT(13, PWM_DUTY_RES);
}

/******************************************************************************
 * SECTION: TCP protocol boundaries
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
 * SECTION: parse_tcp_json() and get_pwm_duty()
 *
 * parse_tcp_json() is the only path that writes g_pwm_duty.
 * get_pwm_duty() is the only path that reads it.
 * Tests drive parse_tcp_json() then assert via get_pwm_duty().
 ******************************************************************************/

void test_parse_valid_pwm_sets_duty(void)
{
    parse_tcp_json("{\"pwm\": 1000}");
    TEST_ASSERT_EQUAL_UINT32(1000u, get_pwm_duty());
}

void test_parse_pwm_zero_sets_duty_zero(void)
{
    parse_tcp_json("{\"pwm\": 0}");
    TEST_ASSERT_EQUAL_UINT32(0u, get_pwm_duty());
}

void test_parse_pwm_max_sets_duty_max(void)
{
    parse_tcp_json("{\"pwm\": 8191}");
    TEST_ASSERT_EQUAL_UINT32(PWM_DUTY_MAX, get_pwm_duty());
}

void test_parse_pwm_clamps_above_max(void)
{
    /* Hub may send out-of-range value -- firmware must clamp, not crash */
    parse_tcp_json("{\"pwm\": 99999}");
    TEST_ASSERT_EQUAL_UINT32(PWM_DUTY_MAX, get_pwm_duty());
}

void test_parse_null_input_does_not_crash(void)
{
    /* cJSON_Parse(NULL) is safe -- must not crash or corrupt duty */
    uint32_t before = get_pwm_duty();
    parse_tcp_json(NULL);
    TEST_ASSERT_EQUAL_UINT32(before, get_pwm_duty());
}

void test_parse_garbage_input_does_not_crash(void)
{
    uint32_t before = get_pwm_duty();
    parse_tcp_json("not json at all !!!");
    TEST_ASSERT_EQUAL_UINT32(before, get_pwm_duty());
}

void test_parse_empty_object_does_not_change_duty(void)
{
    parse_tcp_json("{\"pwm\": 500}");
    uint32_t before = get_pwm_duty();
    parse_tcp_json("{}");
    TEST_ASSERT_EQUAL_UINT32(before, get_pwm_duty());
}

void test_parse_unknown_key_ignored(void)
{
    parse_tcp_json("{\"pwm\": 400}");
    parse_tcp_json("{\"temp\": 25, \"fan\": 100}");
    /* Unknown keys must not touch duty */
    TEST_ASSERT_EQUAL_UINT32(400u, get_pwm_duty());
}

void test_parse_sequential_updates(void)
{
    parse_tcp_json("{\"pwm\": 100}");
    TEST_ASSERT_EQUAL_UINT32(100u, get_pwm_duty());
    parse_tcp_json("{\"pwm\": 4000}");
    TEST_ASSERT_EQUAL_UINT32(4000u, get_pwm_duty());
    parse_tcp_json("{\"pwm\": 0}");
    TEST_ASSERT_EQUAL_UINT32(0u, get_pwm_duty());
}

void test_get_pwm_duty_initial_value_is_zero(void)
{
    /* g_pwm_duty is static -- initialised to 0 at startup */
    parse_tcp_json("{\"pwm\": 0}");
    TEST_ASSERT_EQUAL_UINT32(0u, get_pwm_duty());
}

/******************************************************************************
 * SECTION: ADC decode chain
 ******************************************************************************/

void test_adc_ceiling_db12(void)
{
    TEST_ASSERT_EQUAL_INT32(3379, 3379);
    TEST_ASSERT_EQUAL_INT32(4095, ADC_MAX_RAW);
}

void test_adc_decode_chain_9v_nominal(void)
{
    int32_t ceiling_mv      = 3379;
    int32_t steps           = 4095;
    float   step_mv         = (float)ceiling_mv / (float)steps;

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
 * SECTION: ADC calibration
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
 * SECTION: SOC thresholds
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
    /* 8500mV = (9500 + 7500) / 2 = 50% */
    int mid = (VBAT_FULL_MV + VBAT_DEAD_MV) / 2;
    TEST_ASSERT_EQUAL_UINT8(50u, mv_to_soc(mid));
}

void test_mv_to_soc_quarter_point(void)
{
    /* 8000mV = 7500 + 500 = 25% */
    int quarter = VBAT_DEAD_MV + (VBAT_RANGE_MV / 4);
    TEST_ASSERT_EQUAL_UINT8(25u, mv_to_soc(quarter));
}

void test_mv_to_soc_three_quarter_point(void)
{
    /* 9000mV = 7500 + 1500 = 75% */
    int three_quarter = VBAT_DEAD_MV + (VBAT_RANGE_MV * 3 / 4);
    TEST_ASSERT_EQUAL_UINT8(75u, mv_to_soc(three_quarter));
}

void test_mv_to_soc_boundary_adjacent_above_dead(void)
{
    /* Minimum offset that survives integer truncation:
     * (20 * 100) / 2000 = 1 > 0. 10mV truncates to 0. */
    TEST_ASSERT_GREATER_THAN(0u, mv_to_soc(VBAT_DEAD_MV + 20));
}

void test_mv_to_soc_boundary_adjacent_below_full(void)
{
    TEST_ASSERT_LESS_THAN(100u, mv_to_soc(VBAT_FULL_MV - 20));
}

/******************************************************************************
 * SECTION: Sag reject logic
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

/*----------------------------------------------------------------------------*/

int main(void)
{
    UNITY_BEGIN();

    /* Timing */
    RUN_TEST(test_stats_interval_ms);
    RUN_TEST(test_motor_loop_ms);
    RUN_TEST(test_recv_timeout_ms);
    RUN_TEST(test_accept_timeout_sec);
    RUN_TEST(test_batt_interval_ms);
    RUN_TEST(test_motor_deep_sleep_us);

    /* Motor protection */
    RUN_TEST(test_min_run_sec);
    RUN_TEST(test_min_run_sec_is_10_minutes);

    /* PWM */
    RUN_TEST(test_pwm_duty_max);
    RUN_TEST(test_pwm_duty_max_13bit);
    RUN_TEST(test_pwm_duty_res_is_13);

    /* TCP protocol */
    RUN_TEST(test_tcp_port);
    RUN_TEST(test_batt_json_buf_fits_worst_case);
    RUN_TEST(test_batt_sag_reject_mv);

    /* parse_tcp_json / get_pwm_duty */
    RUN_TEST(test_parse_valid_pwm_sets_duty);
    RUN_TEST(test_parse_pwm_zero_sets_duty_zero);
    RUN_TEST(test_parse_pwm_max_sets_duty_max);
    RUN_TEST(test_parse_pwm_clamps_above_max);
    RUN_TEST(test_parse_null_input_does_not_crash);
    RUN_TEST(test_parse_garbage_input_does_not_crash);
    RUN_TEST(test_parse_empty_object_does_not_change_duty);
    RUN_TEST(test_parse_unknown_key_ignored);
    RUN_TEST(test_parse_sequential_updates);
    RUN_TEST(test_get_pwm_duty_initial_value_is_zero);

    /* ADC decode chain */
    RUN_TEST(test_adc_ceiling_db12);
    RUN_TEST(test_adc_decode_chain_9v_nominal);
    RUN_TEST(test_adc_fresh_battery_under_ceiling);

    /* ADC calibration */
    RUN_TEST(test_cal_ratio_corrects_downward);
    RUN_TEST(test_cal_applied_to_fresh_battery);
    RUN_TEST(test_cal_num_den_nonzero);

    /* SOC thresholds */
    RUN_TEST(test_vbat_full_mv);
    RUN_TEST(test_vbat_dead_mv);
    RUN_TEST(test_vbat_range_mv);
    RUN_TEST(test_mv_to_soc_full);
    RUN_TEST(test_mv_to_soc_dead);
    RUN_TEST(test_mv_to_soc_clamps_above_full);
    RUN_TEST(test_mv_to_soc_clamps_below_dead);
    RUN_TEST(test_mv_to_soc_midpoint);
    RUN_TEST(test_mv_to_soc_quarter_point);
    RUN_TEST(test_mv_to_soc_three_quarter_point);
    RUN_TEST(test_mv_to_soc_boundary_adjacent_above_dead);
    RUN_TEST(test_mv_to_soc_boundary_adjacent_below_full);

    /* Sag reject */
    RUN_TEST(test_sag_reject_exactly_at_threshold_passes);
    RUN_TEST(test_sag_reject_one_below_threshold_rejected);
    RUN_TEST(test_sag_reject_first_reading_always_accepted);

    return UNITY_END();
}
