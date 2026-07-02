#include "unity.h"
#include "device_gateway.h"
#include "uart_manager.h"
#include "cJSON.h"
#include "config.h"

void setUp(void)
{
    /* Reset cJSON stub to known good state before each test */
    extern int   g_cjson_parse_ok;
    extern cJSON g_cjson_kp_item;
    extern cJSON g_cjson_ki_item;
    extern cJSON g_cjson_kd_item;
    extern cJSON g_cjson_setpoint_item;
    extern int   g_cjson_kp_found;
    extern int   g_cjson_ki_found;
    extern int   g_cjson_kd_found;
    extern int   g_cjson_setpoint_found;
    extern int   g_cjson_light_found;
    extern int   g_cjson_avg_temp_found;

    g_cjson_parse_ok          = 1;
    g_cjson_kp_item.valuedouble      = DEFAULT_GATEWAY_KP;
    g_cjson_ki_item.valuedouble      = DEFAULT_GATEWAY_KI;
    g_cjson_kd_item.valuedouble      = DEFAULT_GATEWAY_KD;
    g_cjson_setpoint_item.valueint   = DEFAULT_GATEWAY_SETPOINT;
    g_cjson_kp_found          = 1;
    g_cjson_ki_found          = 1;
    g_cjson_kd_found          = 1;
    g_cjson_setpoint_found    = 1;
    g_cjson_light_found       = 0;
    g_cjson_avg_temp_found    = 0;

    /* Reset aws_manager to defaults */
    device_gateway_init();
}

void tearDown(void) {}

/******************************************************************************
 * aws_manager initial value tests
 ******************************************************************************/
void test_aws_kp_default(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, DEFAULT_GATEWAY_KP, gateway_get_kp());
}

void test_aws_ki_default(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, DEFAULT_GATEWAY_KI, gateway_get_ki());
}

void test_aws_kd_default(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, DEFAULT_GATEWAY_KD, gateway_get_kd());
}

void test_aws_setpoint_default(void)
{
    TEST_ASSERT_EQUAL_INT(DEFAULT_GATEWAY_SETPOINT, gateway_get_setpoint());
}

/******************************************************************************
 * uart_manager initial value tests
 ******************************************************************************/
void test_uart_avg_temp_default(void)
{
    TEST_ASSERT_EQUAL_INT(DEFAULT_AVG_TEMP, uart_get_avg_temp());
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_aws_kp_default);
    RUN_TEST(test_aws_ki_default);
    RUN_TEST(test_aws_kd_default);
    RUN_TEST(test_aws_setpoint_default);
    RUN_TEST(test_uart_avg_temp_default);

    return UNITY_END();
}
