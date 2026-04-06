#include "unity.h"
#include "blackpill_logic.h"

void setUp(void) {}
void tearDown(void) {}

/******************************************************************************
 * parse_int -- core parser primitive
 ******************************************************************************/
void test_parse_int_valid(void)
{
    int result = 0;
    TEST_ASSERT_EQUAL_INT(0, logic_parse_int("42", &result));
    TEST_ASSERT_EQUAL_INT(42, result);
}

void test_parse_int_zero(void)
{
    int result = 99;
    TEST_ASSERT_EQUAL_INT(0, logic_parse_int("0", &result));
    TEST_ASSERT_EQUAL_INT(0, result);
}

void test_parse_int_negative(void)
{
    int result = 0;
    TEST_ASSERT_EQUAL_INT(0, logic_parse_int("-1", &result));
    TEST_ASSERT_EQUAL_INT(-1, result);
}

void test_parse_int_null_str(void)
{
    int result = 0;
    TEST_ASSERT_EQUAL_INT(-1, logic_parse_int(NULL, &result));
}

void test_parse_int_null_result(void)
{
    TEST_ASSERT_EQUAL_INT(-1, logic_parse_int("42", NULL));
}

void test_parse_int_empty_string(void)
{
    int result = 0;
    TEST_ASSERT_EQUAL_INT(-1, logic_parse_int("", &result));
}

void test_parse_int_non_numeric(void)
{
    int result = 0;
    TEST_ASSERT_EQUAL_INT(-1, logic_parse_int("abc", &result));
}

void test_parse_int_max_valid(void)
{
    int result = 0;
    TEST_ASSERT_EQUAL_INT(0, logic_parse_int("65535", &result));
    TEST_ASSERT_EQUAL_INT(65535, result);
}

/******************************************************************************
 * DR slot routing
 ******************************************************************************/
void test_dr_id_valid_dr1(void)
{
    TEST_ASSERT_TRUE(logic_is_dr_id("DR1"));
}

void test_dr_id_valid_dr6(void)
{
    TEST_ASSERT_TRUE(logic_is_dr_id("DR6"));
}

void test_dr_id_invalid_dr0(void)
{
    TEST_ASSERT_FALSE(logic_is_dr_id("DR0"));
}

void test_dr_id_invalid_dr7(void)
{
    TEST_ASSERT_FALSE(logic_is_dr_id("DR7"));
}

void test_dr_id_invalid_pir(void)
{
    TEST_ASSERT_FALSE(logic_is_dr_id("PIR"));
}

void test_dr_id_invalid_empty(void)
{
    TEST_ASSERT_FALSE(logic_is_dr_id(""));
}

void test_dr_id_null(void)
{
    TEST_ASSERT_FALSE(logic_is_dr_id(NULL));
}

void test_dr_slot_dr1_is_0(void)
{
    TEST_ASSERT_EQUAL_INT(0, logic_dr_slot("DR1"));
}

void test_dr_slot_dr6_is_5(void)
{
    TEST_ASSERT_EQUAL_INT(5, logic_dr_slot("DR6"));
}

void test_dr_slot_dr3_is_2(void)
{
    TEST_ASSERT_EQUAL_INT(2, logic_dr_slot("DR3"));
}

/******************************************************************************
 * BLE age online check
 ******************************************************************************/
void test_ble_online_below_threshold(void)
{
    TEST_ASSERT_TRUE(logic_ble_is_online(149));
}

void test_ble_online_at_threshold(void)
{
    /* strictly less than — 150 is NOT online */
    TEST_ASSERT_FALSE(logic_ble_is_online(150));
}

void test_ble_online_above_threshold(void)
{
    TEST_ASSERT_FALSE(logic_ble_is_online(151));
}

void test_ble_online_zero(void)
{
    TEST_ASSERT_TRUE(logic_ble_is_online(0));
}

void test_ble_age_threshold_value(void)
{
    TEST_ASSERT_EQUAL_UINT32(150u, BLE_AGE_THRESHOLD_S);
}

/******************************************************************************
 * Reed count clamping
 ******************************************************************************/
void test_clamp_reed_count_valid(void)
{
    TEST_ASSERT_EQUAL_INT(3, logic_clamp_reed_count(3));
}

void test_clamp_reed_count_max(void)
{
    TEST_ASSERT_EQUAL_INT(6, logic_clamp_reed_count(6));
}

void test_clamp_reed_count_over_max(void)
{
    TEST_ASSERT_EQUAL_INT(6, logic_clamp_reed_count(10));
}

void test_clamp_reed_count_zero(void)
{
    TEST_ASSERT_EQUAL_INT(0, logic_clamp_reed_count(0));
}

void test_clamp_reed_count_negative(void)
{
    TEST_ASSERT_EQUAL_INT(0, logic_clamp_reed_count(-1));
}

/******************************************************************************
 * Stack canary
 ******************************************************************************/
void test_canary_intact_all_good(void)
{
    uint32_t stack[4] = {STACK_CANARY, STACK_CANARY, STACK_CANARY, STACK_CANARY};
    TEST_ASSERT_TRUE(logic_canary_intact(stack, 4));
}

void test_canary_corrupted_first_word(void)
{
    uint32_t stack[4] = {0xDEADDEAD, STACK_CANARY, STACK_CANARY, STACK_CANARY};
    TEST_ASSERT_FALSE(logic_canary_intact(stack, 4));
}

void test_canary_corrupted_last_word(void)
{
    uint32_t stack[4] = {STACK_CANARY, STACK_CANARY, STACK_CANARY, 0x00000000};
    TEST_ASSERT_FALSE(logic_canary_intact(stack, 4));
}

void test_canary_null_base_returns_true(void)
{
    /* NULL base means canary not installed -- not a failure */
    TEST_ASSERT_TRUE(logic_canary_intact(NULL, 4));
}

void test_canary_magic_value(void)
{
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFul, STACK_CANARY);
}

void test_stack_guard_words(void)
{
    TEST_ASSERT_EQUAL_UINT32(4u, STACK_GUARD_WORDS);
}

/******************************************************************************
 * Timing constants
 ******************************************************************************/
void test_hb_check_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(1000ul, HB_CHECK_MS);
}

void test_uart_queue_depth(void)
{
    TEST_ASSERT_EQUAL_UINT32(8u, UART_QUEUE_DEPTH);
}

void test_main_loop_delay_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(5u, MAIN_LOOP_DELAY_MS);
}

void test_iwdg_reload(void)
{
    /* LSI ~32kHz / prescaler 32 * 3000 ticks ~ 3s timeout */
    TEST_ASSERT_EQUAL_UINT32(3000u, IWDG_RELOAD);
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

void test_trinity_err_watchdog(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x06u, eTRINITY_ERR_WATCHDOG);
}

void test_trinity_err_unknown(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xFFu, eTRINITY_ERR_UNKNOWN);
}

/******************************************************************************
 * STATE field count
 ******************************************************************************/
void test_state_field_count(void)
{
    TEST_ASSERT_EQUAL_UINT8(8u, logic_state_field_count());
}

void test_uart_line_len(void)
{
    TEST_ASSERT_EQUAL_UINT32(128u, UART_LINE_LEN);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_parse_int_valid);
    RUN_TEST(test_parse_int_zero);
    RUN_TEST(test_parse_int_negative);
    RUN_TEST(test_parse_int_null_str);
    RUN_TEST(test_parse_int_null_result);
    RUN_TEST(test_parse_int_empty_string);
    RUN_TEST(test_parse_int_non_numeric);
    RUN_TEST(test_parse_int_max_valid);
    RUN_TEST(test_dr_id_valid_dr1);
    RUN_TEST(test_dr_id_valid_dr6);
    RUN_TEST(test_dr_id_invalid_dr0);
    RUN_TEST(test_dr_id_invalid_dr7);
    RUN_TEST(test_dr_id_invalid_pir);
    RUN_TEST(test_dr_id_invalid_empty);
    RUN_TEST(test_dr_id_null);
    RUN_TEST(test_dr_slot_dr1_is_0);
    RUN_TEST(test_dr_slot_dr6_is_5);
    RUN_TEST(test_dr_slot_dr3_is_2);
    RUN_TEST(test_ble_online_below_threshold);
    RUN_TEST(test_ble_online_at_threshold);
    RUN_TEST(test_ble_online_above_threshold);
    RUN_TEST(test_ble_online_zero);
    RUN_TEST(test_ble_age_threshold_value);
    RUN_TEST(test_clamp_reed_count_valid);
    RUN_TEST(test_clamp_reed_count_max);
    RUN_TEST(test_clamp_reed_count_over_max);
    RUN_TEST(test_clamp_reed_count_zero);
    RUN_TEST(test_clamp_reed_count_negative);
    RUN_TEST(test_canary_intact_all_good);
    RUN_TEST(test_canary_corrupted_first_word);
    RUN_TEST(test_canary_corrupted_last_word);
    RUN_TEST(test_canary_null_base_returns_true);
    RUN_TEST(test_canary_magic_value);
    RUN_TEST(test_stack_guard_words);
    RUN_TEST(test_hb_check_ms);
    RUN_TEST(test_uart_queue_depth);
    RUN_TEST(test_main_loop_delay_ms);
    RUN_TEST(test_iwdg_reload);
    RUN_TEST(test_trinity_err_none);
    RUN_TEST(test_trinity_err_hardfault);
    RUN_TEST(test_trinity_err_stack);
    RUN_TEST(test_trinity_err_watchdog);
    RUN_TEST(test_trinity_err_unknown);
    RUN_TEST(test_state_field_count);
    RUN_TEST(test_uart_line_len);

    return UNITY_END();
}
