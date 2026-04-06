#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include <string.h>
#include <stdint.h>
#include "server_logic.h"

/******************************************************************************
 * Age clamping
 ******************************************************************************/
void test_age_clamp_normal(void)
{
    CU_ASSERT_EQUAL(logic_clamp_age(100), 100);
}

void test_age_clamp_zero(void)
{
    CU_ASSERT_EQUAL(logic_clamp_age(0), 0);
}

void test_age_clamp_negative(void)
{
    CU_ASSERT_EQUAL(logic_clamp_age(-1), AGE_UNKNOWN);
}

void test_age_clamp_at_max(void)
{
    CU_ASSERT_EQUAL(logic_clamp_age((int)AGE_MAX), (uint16_t)AGE_MAX);
}

void test_age_clamp_over_max(void)
{
    CU_ASSERT_EQUAL(logic_clamp_age(0xFFFF), (uint16_t)AGE_MAX);
}

void test_age_unknown_sentinel(void)
{
    CU_ASSERT_EQUAL(AGE_UNKNOWN, 0xFFFFu);
}

void test_age_max_sentinel(void)
{
    CU_ASSERT_EQUAL(AGE_MAX, 0xFFFEu);
}

/******************************************************************************
 * Reed slot ID to index
 ******************************************************************************/
void test_reed_id_to_slot_1(void)
{
    CU_ASSERT_EQUAL(logic_reed_id_to_slot(1), 0);
}

void test_reed_id_to_slot_6(void)
{
    CU_ASSERT_EQUAL(logic_reed_id_to_slot(6), 5);
}

void test_reed_id_to_slot_3(void)
{
    CU_ASSERT_EQUAL(logic_reed_id_to_slot(3), 2);
}

void test_reed_slot_valid_0(void)
{
    CU_ASSERT_TRUE(logic_reed_slot_valid(0));
}

void test_reed_slot_valid_5(void)
{
    CU_ASSERT_TRUE(logic_reed_slot_valid(5));
}

void test_reed_slot_invalid_neg(void)
{
    CU_ASSERT_FALSE(logic_reed_slot_valid(-1));
}

void test_reed_slot_invalid_6(void)
{
    CU_ASSERT_FALSE(logic_reed_slot_valid(6));
}

/******************************************************************************
 * extract_json brace counting
 ******************************************************************************/
void test_extract_json_simple(void)
{
    char buf[] = "{\"a\":1}";
    char *result = logic_extract_json(buf);
    CU_ASSERT_PTR_NOT_NULL(result);
    CU_ASSERT_STRING_EQUAL(result, "{\"a\":1}");
}

void test_extract_json_nested(void)
{
    char buf[] = "{\"a\":{\"b\":2}}";
    char *result = logic_extract_json(buf);
    CU_ASSERT_PTR_NOT_NULL(result);
    CU_ASSERT_STRING_EQUAL(result, "{\"a\":{\"b\":2}}");
}

void test_extract_json_no_brace(void)
{
    char buf[] = "no json here";
    CU_ASSERT_PTR_NULL(logic_extract_json(buf));
}

void test_extract_json_incomplete(void)
{
    char buf[] = "{\"a\":1";
    CU_ASSERT_PTR_NULL(logic_extract_json(buf));
}

void test_extract_json_leading_garbage(void)
{
    char buf[] = "garbage{\"a\":1}";
    char *result = logic_extract_json(buf);
    CU_ASSERT_PTR_NOT_NULL(result);
    CU_ASSERT_STRING_EQUAL(result, "{\"a\":1}");
}

void test_extract_json_empty(void)
{
    char buf[] = "";
    CU_ASSERT_PTR_NULL(logic_extract_json(buf));
}

/******************************************************************************
 * Buffer overflow check
 ******************************************************************************/
void test_buf_has_room_ok(void)
{
    CU_ASSERT_TRUE(logic_buf_has_room(0, 100, BUFFER_SIZE));
}

void test_buf_has_room_full(void)
{
    CU_ASSERT_FALSE(logic_buf_has_room(BUFFER_SIZE - 1, 1, BUFFER_SIZE));
}

void test_buf_has_room_exact(void)
{
    CU_ASSERT_FALSE(logic_buf_has_room(BUFFER_SIZE - 1, 2, BUFFER_SIZE));
}

/******************************************************************************
 * Struct size constants
 ******************************************************************************/
void test_max_reeds(void)
{
    CU_ASSERT_EQUAL(MAX_REEDS, 6);
}

void test_max_rooms(void)
{
    CU_ASSERT_EQUAL(MAX_ROOMS, 10);
}

void test_reed_name_len(void)
{
    CU_ASSERT_EQUAL(REED_NAME_LEN, 16);
}

void test_room_name_len(void)
{
    CU_ASSERT_EQUAL(ROOM_NAME_LEN, 32);
}

void test_pipe_retry_sec(void)
{
    CU_ASSERT_EQUAL(PIPE_RETRY_SEC, 3);
}

/******************************************************************************
 * Test registry
 ******************************************************************************/
int main(void)
{
    CU_pSuite age_suite  = NULL;
    CU_pSuite reed_suite = NULL;
    CU_pSuite json_suite = NULL;
    CU_pSuite buf_suite  = NULL;
    CU_pSuite const_suite = NULL;

    if (CUE_SUCCESS != CU_initialize_registry()) { return CU_get_error(); }

    age_suite = CU_add_suite("age_clamping", NULL, NULL);
    CU_add_test(age_suite, "normal",       test_age_clamp_normal);
    CU_add_test(age_suite, "zero",         test_age_clamp_zero);
    CU_add_test(age_suite, "negative",     test_age_clamp_negative);
    CU_add_test(age_suite, "at_max",       test_age_clamp_at_max);
    CU_add_test(age_suite, "over_max",     test_age_clamp_over_max);
    CU_add_test(age_suite, "unknown_sentinel", test_age_unknown_sentinel);
    CU_add_test(age_suite, "max_sentinel", test_age_max_sentinel);

    reed_suite = CU_add_suite("reed_slots", NULL, NULL);
    CU_add_test(reed_suite, "id_to_slot_1",     test_reed_id_to_slot_1);
    CU_add_test(reed_suite, "id_to_slot_6",     test_reed_id_to_slot_6);
    CU_add_test(reed_suite, "id_to_slot_3",     test_reed_id_to_slot_3);
    CU_add_test(reed_suite, "slot_valid_0",     test_reed_slot_valid_0);
    CU_add_test(reed_suite, "slot_valid_5",     test_reed_slot_valid_5);
    CU_add_test(reed_suite, "slot_invalid_neg", test_reed_slot_invalid_neg);
    CU_add_test(reed_suite, "slot_invalid_6",   test_reed_slot_invalid_6);

    json_suite = CU_add_suite("extract_json", NULL, NULL);
    CU_add_test(json_suite, "simple",           test_extract_json_simple);
    CU_add_test(json_suite, "nested",           test_extract_json_nested);
    CU_add_test(json_suite, "no_brace",         test_extract_json_no_brace);
    CU_add_test(json_suite, "incomplete",       test_extract_json_incomplete);
    CU_add_test(json_suite, "leading_garbage",  test_extract_json_leading_garbage);
    CU_add_test(json_suite, "empty",            test_extract_json_empty);

    buf_suite = CU_add_suite("buffer_overflow", NULL, NULL);
    CU_add_test(buf_suite, "has_room",  test_buf_has_room_ok);
    CU_add_test(buf_suite, "full",      test_buf_has_room_full);
    CU_add_test(buf_suite, "exact",     test_buf_has_room_exact);

    const_suite = CU_add_suite("constants", NULL, NULL);
    CU_add_test(const_suite, "max_reeds",      test_max_reeds);
    CU_add_test(const_suite, "max_rooms",      test_max_rooms);
    CU_add_test(const_suite, "reed_name_len",  test_reed_name_len);
    CU_add_test(const_suite, "room_name_len",  test_room_name_len);
    CU_add_test(const_suite, "pipe_retry_sec", test_pipe_retry_sec);

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    CU_cleanup_registry();

    return CU_get_error();
}
