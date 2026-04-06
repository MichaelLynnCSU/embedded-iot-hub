#include "unity.h"
#include "bluepill_logic.h"

void setUp(void) {}
void tearDown(void) {}

/******************************************************************************
 * Timing constants
 ******************************************************************************/
void test_default_task_delay(void)
{
    TEST_ASSERT_EQUAL_UINT32(5000u, DEFAULT_TASK_DELAY);
}

void test_sensor_task_delay(void)
{
    TEST_ASSERT_EQUAL_UINT32(2000u, SENSOR_TASK_DELAY);
}

void test_sensor_warmup_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(2000u, SENSOR_WARMUP_MS);
}

void test_iwdg_reload(void)
{
    /* LSI ~40kHz / prescaler 64 * 1875 ticks ~ 3s timeout */
    TEST_ASSERT_EQUAL_UINT32(1875u, IWDG_RELOAD);
}

void test_sensor_count(void)
{
    TEST_ASSERT_EQUAL_UINT32(3u, SENSOR_COUNT);
}

/******************************************************************************
 * classify_reset_cause -- F103 RCC_CSR bit decode
 ******************************************************************************/
void test_reset_por_is_brownout(void)
{
    /* RCC_CSR_PORRSTF = bit 27 */
    uint32_t csr = RCC_CSR_PORRSTF;
    TEST_ASSERT_EQUAL_INT(eTRINITY_ERR_BROWNOUT,
                          bluepill_classify_reset(csr));
}

void test_reset_iwdg_is_watchdog(void)
{
    uint32_t csr = RCC_CSR_IWDGRSTF;
    TEST_ASSERT_EQUAL_INT(eTRINITY_ERR_WATCHDOG,
                          bluepill_classify_reset(csr));
}

void test_reset_wwdg_is_watchdog(void)
{
    uint32_t csr = RCC_CSR_WWDGRSTF;
    TEST_ASSERT_EQUAL_INT(eTRINITY_ERR_WATCHDOG,
                          bluepill_classify_reset(csr));
}

void test_reset_pin_is_none(void)
{
    uint32_t csr = RCC_CSR_PINRSTF;
    TEST_ASSERT_EQUAL_INT(eTRINITY_ERR_NONE,
                          bluepill_classify_reset(csr));
}

void test_reset_soft_is_none(void)
{
    uint32_t csr = RCC_CSR_SFTRSTF;
    TEST_ASSERT_EQUAL_INT(eTRINITY_ERR_NONE,
                          bluepill_classify_reset(csr));
}

void test_reset_zero_is_unknown(void)
{
    TEST_ASSERT_EQUAL_INT(eTRINITY_ERR_UNKNOWN,
                          bluepill_classify_reset(0u));
}

void test_reset_por_beats_iwdg(void)
{
    /* POR has highest priority in the if-else chain */
    uint32_t csr = RCC_CSR_PORRSTF | RCC_CSR_IWDGRSTF;
    TEST_ASSERT_EQUAL_INT(eTRINITY_ERR_BROWNOUT,
                          bluepill_classify_reset(csr));
}

/******************************************************************************
 * Average temperature logic
 ******************************************************************************/
void test_avg_all_valid(void)
{
    uint8_t temps[3] = {20, 22, 24};
    uint8_t valid[3] = {1, 1, 1};
    TEST_ASSERT_EQUAL_INT(22, bluepill_avg_temp(temps, valid, 3));
}

void test_avg_one_valid(void)
{
    uint8_t temps[3] = {25, 0, 0};
    uint8_t valid[3] = {1, 0, 0};
    TEST_ASSERT_EQUAL_INT(25, bluepill_avg_temp(temps, valid, 3));
}

void test_avg_two_valid(void)
{
    uint8_t temps[3] = {20, 30, 0};
    uint8_t valid[3] = {1, 1, 0};
    TEST_ASSERT_EQUAL_INT(25, bluepill_avg_temp(temps, valid, 3));
}

void test_avg_none_valid_returns_zero(void)
{
    uint8_t temps[3] = {0, 0, 0};
    uint8_t valid[3] = {0, 0, 0};
    TEST_ASSERT_EQUAL_INT(0, bluepill_avg_temp(temps, valid, 3));
}

void test_avg_single_sensor(void)
{
    uint8_t temps[1] = {37};
    uint8_t valid[1] = {1};
    TEST_ASSERT_EQUAL_INT(37, bluepill_avg_temp(temps, valid, 1));
}

void test_avg_ignores_invalid_high_reading(void)
{
    /* sensor 2 has bad data but invalid flag -- must be ignored */
    uint8_t temps[3] = {20, 99, 22};
    uint8_t valid[3] = {1, 0, 1};
    TEST_ASSERT_EQUAL_INT(21, bluepill_avg_temp(temps, valid, 3));
}

/******************************************************************************
 * Fault type name routing
 ******************************************************************************/
void test_fault_name_hardfault(void)
{
    TEST_ASSERT_EQUAL_STRING("HARDFAULT", bluepill_fault_name(1u));
}

void test_fault_name_memmanage(void)
{
    TEST_ASSERT_EQUAL_STRING("MEMMANAGE", bluepill_fault_name(2u));
}

void test_fault_name_busfault(void)
{
    TEST_ASSERT_EQUAL_STRING("BUSFAULT", bluepill_fault_name(3u));
}

void test_fault_name_usagefault(void)
{
    TEST_ASSERT_EQUAL_STRING("USAGEFAULT", bluepill_fault_name(4u));
}

void test_fault_name_unknown(void)
{
    TEST_ASSERT_EQUAL_STRING("FAULT", bluepill_fault_name(0u));
    TEST_ASSERT_EQUAL_STRING("FAULT", bluepill_fault_name(99u));
}

/******************************************************************************
 * TRINITY_ERROR_E values
 ******************************************************************************/
void test_trinity_err_none(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x00u, eTRINITY_ERR_NONE);
}

void test_trinity_err_hardfault(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x01u, eTRINITY_ERR_HARDFAULT);
}

void test_trinity_err_stack(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x02u, eTRINITY_ERR_STACK);
}

void test_trinity_err_brownout(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x05u, eTRINITY_ERR_BROWNOUT);
}

void test_trinity_err_watchdog(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x06u, eTRINITY_ERR_WATCHDOG);
}

void test_trinity_err_unknown(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xFFu, eTRINITY_ERR_UNKNOWN);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_default_task_delay);
    RUN_TEST(test_sensor_task_delay);
    RUN_TEST(test_sensor_warmup_ms);
    RUN_TEST(test_iwdg_reload);
    RUN_TEST(test_sensor_count);
    RUN_TEST(test_reset_por_is_brownout);
    RUN_TEST(test_reset_iwdg_is_watchdog);
    RUN_TEST(test_reset_wwdg_is_watchdog);
    RUN_TEST(test_reset_pin_is_none);
    RUN_TEST(test_reset_soft_is_none);
    RUN_TEST(test_reset_zero_is_unknown);
    RUN_TEST(test_reset_por_beats_iwdg);
    RUN_TEST(test_avg_all_valid);
    RUN_TEST(test_avg_one_valid);
    RUN_TEST(test_avg_two_valid);
    RUN_TEST(test_avg_none_valid_returns_zero);
    RUN_TEST(test_avg_single_sensor);
    RUN_TEST(test_avg_ignores_invalid_high_reading);
    RUN_TEST(test_fault_name_hardfault);
    RUN_TEST(test_fault_name_memmanage);
    RUN_TEST(test_fault_name_busfault);
    RUN_TEST(test_fault_name_usagefault);
    RUN_TEST(test_fault_name_unknown);
    RUN_TEST(test_trinity_err_none);
    RUN_TEST(test_trinity_err_hardfault);
    RUN_TEST(test_trinity_err_stack);
    RUN_TEST(test_trinity_err_brownout);
    RUN_TEST(test_trinity_err_watchdog);
    RUN_TEST(test_trinity_err_unknown);

    return UNITY_END();
}
