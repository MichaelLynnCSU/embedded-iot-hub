#include "unity.h"
#include "main.h"

void setUp(void) {}
void tearDown(void) {}

/******************************************************************************
 * Timing constants
 ******************************************************************************/
void test_stats_interval_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(60000u, STATS_INTERVAL_MS);
}

void test_motor_loop_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(100, MOTOR_LOOP_MS);
}

void test_recv_timeout_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(30000, RECV_TIMEOUT_MS);
}

void test_accept_timeout_sec(void)
{
    TEST_ASSERT_EQUAL_INT(2, ACCEPT_TIMEOUT_SEC);
}

/******************************************************************************
 * Default values
 ******************************************************************************/
void test_default_aws_low(void)
{
    TEST_ASSERT_EQUAL_INT(20, DEFAULT_AWS_LOW);
}

void test_default_aws_high(void)
{
    TEST_ASSERT_EQUAL_INT(35, DEFAULT_AWS_HIGH);
}

void test_default_avg_temp(void)
{
    TEST_ASSERT_EQUAL_INT(25, DEFAULT_AVG_TEMP);
}

void test_default_temp_in_range(void)
{
    TEST_ASSERT_TRUE(DEFAULT_AVG_TEMP >= DEFAULT_AWS_LOW);
    TEST_ASSERT_TRUE(DEFAULT_AVG_TEMP <= DEFAULT_AWS_HIGH);
}

/******************************************************************************
 * PWM duty interpolation
 ******************************************************************************/
void test_duty_at_low_temp(void)
{
    uint32_t duty = motor_temp_to_duty(DEFAULT_AWS_LOW,
                                       DEFAULT_AWS_LOW, DEFAULT_AWS_HIGH);
    TEST_ASSERT_EQUAL_UINT32(0, duty);
}

void test_duty_at_high_temp(void)
{
    uint32_t duty = motor_temp_to_duty(DEFAULT_AWS_HIGH,
                                       DEFAULT_AWS_LOW, DEFAULT_AWS_HIGH);
    TEST_ASSERT_EQUAL_UINT32(PWM_DUTY_MAX, duty);
}

void test_duty_at_midpoint(void)
{
    int mid = (DEFAULT_AWS_LOW + DEFAULT_AWS_HIGH) / 2;
    uint32_t duty = motor_temp_to_duty(mid, DEFAULT_AWS_LOW, DEFAULT_AWS_HIGH);
    TEST_ASSERT_TRUE(duty > 0);
    TEST_ASSERT_TRUE(duty < PWM_DUTY_MAX);
}

void test_duty_clamps_below_low(void)
{
    uint32_t duty_at_low   = motor_temp_to_duty(DEFAULT_AWS_LOW - 10,
                                                 DEFAULT_AWS_LOW, DEFAULT_AWS_HIGH);
    TEST_ASSERT_EQUAL_UINT32(0, duty_at_low);
}

void test_duty_clamps_above_high(void)
{
    uint32_t duty_at_high  = motor_temp_to_duty(DEFAULT_AWS_HIGH + 10,
                                                  DEFAULT_AWS_LOW, DEFAULT_AWS_HIGH);
    TEST_ASSERT_EQUAL_UINT32(PWM_DUTY_MAX, duty_at_high);
}

void test_duty_zero_range_returns_zero(void)
{
    /* high == low -- guard against divide by zero */
    uint32_t duty = motor_temp_to_duty(25, 25, 25);
    TEST_ASSERT_EQUAL_UINT32(0, duty);
}

/******************************************************************************
 * Log throttle
 ******************************************************************************/
void test_log_throttle_n(void)
{
    TEST_ASSERT_EQUAL_UINT32(20u, LOG_THROTTLE_N);
}

void test_log_throttle_fires_on_20th(void)
{
    uint32_t count = 0;
    int fired = 0;
    for (int i = 0; i < 20; i++) {
        if (++count % LOG_THROTTLE_N == 0) { fired++; }
    }
    TEST_ASSERT_EQUAL_INT(1, fired);
}

void test_log_throttle_fires_on_40th(void)
{
    uint32_t count = 0;
    int fired = 0;
    for (int i = 0; i < 40; i++) {
        if (++count % LOG_THROTTLE_N == 0) { fired++; }
    }
    TEST_ASSERT_EQUAL_INT(2, fired);
}

/******************************************************************************
 * PWM duty max sanity
 ******************************************************************************/
void test_pwm_duty_max(void)
{
    TEST_ASSERT_EQUAL_UINT32((1 << 13) - 1, PWM_DUTY_MAX);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_stats_interval_ms);
    RUN_TEST(test_motor_loop_ms);
    RUN_TEST(test_recv_timeout_ms);
    RUN_TEST(test_accept_timeout_sec);
    RUN_TEST(test_default_aws_low);
    RUN_TEST(test_default_aws_high);
    RUN_TEST(test_default_avg_temp);
    RUN_TEST(test_default_temp_in_range);
    RUN_TEST(test_duty_at_low_temp);
    RUN_TEST(test_duty_at_high_temp);
    RUN_TEST(test_duty_at_midpoint);
    RUN_TEST(test_duty_clamps_below_low);
    RUN_TEST(test_duty_clamps_above_high);
    RUN_TEST(test_duty_zero_range_returns_zero);
    RUN_TEST(test_log_throttle_n);
    RUN_TEST(test_log_throttle_fires_on_20th);
    RUN_TEST(test_log_throttle_fires_on_40th);
    RUN_TEST(test_pwm_duty_max);

    return UNITY_END();
}
