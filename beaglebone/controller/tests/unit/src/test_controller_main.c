#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include <stdint.h>
#include <time.h>
#include "controller_logic.h"

/******************************************************************************
 * Heartbeat online check
 ******************************************************************************/
void test_hb_never_seen_is_offline(void)
{
    CU_ASSERT_EQUAL(logic_hb_is_online(0, time(NULL), 30), 0);
}

void test_hb_recent_is_online(void)
{
    time_t now = time(NULL);
    CU_ASSERT_EQUAL(logic_hb_is_online(now - 5, now, 30), 1);
}

void test_hb_at_timeout_is_offline(void)
{
    time_t now = time(NULL);
    CU_ASSERT_EQUAL(logic_hb_is_online(now - 30, now, 30), 0);
}

void test_hb_just_before_timeout_is_online(void)
{
    time_t now = time(NULL);
    CU_ASSERT_EQUAL(logic_hb_is_online(now - 29, now, 30), 1);
}

void test_hb_long_ago_is_offline(void)
{
    time_t now = time(NULL);
    CU_ASSERT_EQUAL(logic_hb_is_online(now - 3600, now, 30), 0);
}

/******************************************************************************
 * Heartbeat state change detection
 ******************************************************************************/
void test_hb_state_changed_offline_to_online(void)
{
    CU_ASSERT_TRUE(logic_hb_state_changed(0, 1));
}

void test_hb_state_changed_online_to_offline(void)
{
    CU_ASSERT_TRUE(logic_hb_state_changed(1, 0));
}

void test_hb_state_unchanged_online(void)
{
    CU_ASSERT_FALSE(logic_hb_state_changed(1, 1));
}

void test_hb_state_unchanged_offline(void)
{
    CU_ASSERT_FALSE(logic_hb_state_changed(0, 0));
}

/******************************************************************************
 * Device index validation
 ******************************************************************************/
void test_dev_idx_valid_pir(void)
{
    CU_ASSERT_TRUE(logic_dev_idx_valid(DEV_PIR));
}

void test_dev_idx_valid_motor(void)
{
    CU_ASSERT_TRUE(logic_dev_idx_valid(DEV_MOTOR));
}

void test_dev_idx_invalid_neg(void)
{
    CU_ASSERT_FALSE(logic_dev_idx_valid(-1));
}

void test_dev_idx_invalid_dev_count(void)
{
    CU_ASSERT_FALSE(logic_dev_idx_valid(DEV_COUNT));
}

void test_dev_count_value(void)
{
    CU_ASSERT_EQUAL(DEV_COUNT, 5);
}

/******************************************************************************
 * Reed slot validation
 ******************************************************************************/
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

void test_max_reeds_value(void)
{
    CU_ASSERT_EQUAL(MAX_REEDS, 6);
}

/******************************************************************************
 * Alert severity labels
 ******************************************************************************/
void test_severity_low(void)
{
    CU_ASSERT_STRING_EQUAL("LOW", logic_severity_label(ALERT_SEVERITY_LOW));
}

void test_severity_med(void)
{
    CU_ASSERT_STRING_EQUAL("MEDIUM", logic_severity_label(ALERT_SEVERITY_MED));
}

void test_severity_high(void)
{
    CU_ASSERT_STRING_EQUAL("HIGH", logic_severity_label(ALERT_SEVERITY_HIGH));
}

void test_severity_unknown(void)
{
    CU_ASSERT_STRING_EQUAL("UNKNOWN", logic_severity_label(99));
}

void test_severity_values(void)
{
    CU_ASSERT_EQUAL(ALERT_SEVERITY_LOW,  1);
    CU_ASSERT_EQUAL(ALERT_SEVERITY_MED,  2);
    CU_ASSERT_EQUAL(ALERT_SEVERITY_HIGH, 3);
}

/******************************************************************************
 * History ring buffer
 ******************************************************************************/
void test_history_idx_zero(void)
{
    CU_ASSERT_EQUAL(logic_history_idx(0), 0);
}

void test_history_idx_wraps(void)
{
    CU_ASSERT_EQUAL(logic_history_idx(HISTORY_BUF_SIZE), 0);
}

void test_history_idx_mid(void)
{
    CU_ASSERT_EQUAL(logic_history_idx(50), 50);
}

void test_history_idx_just_before_wrap(void)
{
    CU_ASSERT_EQUAL(logic_history_idx(HISTORY_BUF_SIZE - 1),
                    HISTORY_BUF_SIZE - 1);
}

void test_history_buf_size(void)
{
    CU_ASSERT_EQUAL(HISTORY_BUF_SIZE, 100);
}

/******************************************************************************
 * Battery sentinel
 ******************************************************************************/
void test_batt_valid_100(void)
{
    CU_ASSERT_TRUE(logic_batt_valid(100));
}

void test_batt_valid_0(void)
{
    CU_ASSERT_TRUE(logic_batt_valid(0));
}

void test_batt_invalid_neg1(void)
{
    CU_ASSERT_FALSE(logic_batt_valid(-1));
}

/******************************************************************************
 * Shared memory constants
 ******************************************************************************/
void test_device_count(void)
{
    CU_ASSERT_EQUAL(DEVICE_COUNT, 9);
}

void test_alert_buf_size(void)
{
    CU_ASSERT_EQUAL(ALERT_BUF_SIZE, 10);
}

void test_room_buf_size(void)
{
    CU_ASSERT_EQUAL(ROOM_BUF_SIZE, 10);
}

void test_alert_msg_size(void)
{
    CU_ASSERT_EQUAL(ALERT_MSG_SIZE, 64);
}

/******************************************************************************
 * Phantom widget fix (2026-04-21)
 * Bug: process_reed_slots() unconditionally overwrote latest_data with
 *      incoming frame data, even when active=0. A slot missing from one
 *      JSON frame cleared active=0, dropped reed_count, hid the widget
 *      for 1-2 seconds.
 * Fix: only overwrite when incoming slot has active=1.
 ******************************************************************************/
void test_reed_slot_update_when_active(void)
{
    CU_ASSERT_TRUE(logic_reed_slot_should_update(1));
}

void test_reed_slot_no_update_when_inactive(void)
{
    CU_ASSERT_FALSE(logic_reed_slot_should_update(0));
}

void test_reed_slot_no_update_preserves_previous(void)
{
    /* Slot was active, incoming frame has active=0 -- must preserve. */
    uint8_t current_active  = 1;
    uint8_t incoming_active = 0;

    if (logic_reed_slot_should_update(incoming_active))
    {
        current_active = incoming_active;
    }

    CU_ASSERT_EQUAL(current_active, 1);
}

void test_reed_slot_update_overwrites_when_active(void)
{
    /* Incoming frame has active=1 -- slot must be updated. */
    uint8_t current_active  = 0;
    uint8_t incoming_active = 1;

    if (logic_reed_slot_should_update(incoming_active))
    {
        current_active = incoming_active;
    }

    CU_ASSERT_EQUAL(current_active, 1);
}

/******************************************************************************
 * main
 ******************************************************************************/
int main(void)
{
    CU_pSuite hb_suite      = NULL;
    CU_pSuite state_suite   = NULL;
    CU_pSuite dev_suite     = NULL;
    CU_pSuite reed_suite    = NULL;
    CU_pSuite alert_suite   = NULL;
    CU_pSuite history_suite = NULL;
    CU_pSuite batt_suite    = NULL;
    CU_pSuite const_suite   = NULL;
    CU_pSuite phantom_suite = NULL;

    if (CUE_SUCCESS != CU_initialize_registry()) { return CU_get_error(); }

    hb_suite = CU_add_suite("heartbeat_online", NULL, NULL);
    CU_add_test(hb_suite, "never_seen",          test_hb_never_seen_is_offline);
    CU_add_test(hb_suite, "recent",              test_hb_recent_is_online);
    CU_add_test(hb_suite, "at_timeout",          test_hb_at_timeout_is_offline);
    CU_add_test(hb_suite, "just_before_timeout", test_hb_just_before_timeout_is_online);
    CU_add_test(hb_suite, "long_ago",            test_hb_long_ago_is_offline);

    state_suite = CU_add_suite("heartbeat_state_change", NULL, NULL);
    CU_add_test(state_suite, "offline_to_online", test_hb_state_changed_offline_to_online);
    CU_add_test(state_suite, "online_to_offline", test_hb_state_changed_online_to_offline);
    CU_add_test(state_suite, "unchanged_online",  test_hb_state_unchanged_online);
    CU_add_test(state_suite, "unchanged_offline", test_hb_state_unchanged_offline);

    dev_suite = CU_add_suite("device_index", NULL, NULL);
    CU_add_test(dev_suite, "valid_pir",      test_dev_idx_valid_pir);
    CU_add_test(dev_suite, "valid_motor",    test_dev_idx_valid_motor);
    CU_add_test(dev_suite, "invalid_neg",    test_dev_idx_invalid_neg);
    CU_add_test(dev_suite, "invalid_count",  test_dev_idx_invalid_dev_count);
    CU_add_test(dev_suite, "dev_count",      test_dev_count_value);

    reed_suite = CU_add_suite("reed_slots", NULL, NULL);
    CU_add_test(reed_suite, "valid_0",     test_reed_slot_valid_0);
    CU_add_test(reed_suite, "valid_5",     test_reed_slot_valid_5);
    CU_add_test(reed_suite, "invalid_neg", test_reed_slot_invalid_neg);
    CU_add_test(reed_suite, "invalid_6",   test_reed_slot_invalid_6);
    CU_add_test(reed_suite, "max_reeds",   test_max_reeds_value);

    alert_suite = CU_add_suite("alert_severity", NULL, NULL);
    CU_add_test(alert_suite, "low",     test_severity_low);
    CU_add_test(alert_suite, "med",     test_severity_med);
    CU_add_test(alert_suite, "high",    test_severity_high);
    CU_add_test(alert_suite, "unknown", test_severity_unknown);
    CU_add_test(alert_suite, "values",  test_severity_values);

    history_suite = CU_add_suite("history_ring", NULL, NULL);
    CU_add_test(history_suite, "zero",             test_history_idx_zero);
    CU_add_test(history_suite, "wraps",            test_history_idx_wraps);
    CU_add_test(history_suite, "mid",              test_history_idx_mid);
    CU_add_test(history_suite, "just_before_wrap", test_history_idx_just_before_wrap);
    CU_add_test(history_suite, "buf_size",         test_history_buf_size);

    batt_suite = CU_add_suite("battery_sentinel", NULL, NULL);
    CU_add_test(batt_suite, "valid_100", test_batt_valid_100);
    CU_add_test(batt_suite, "valid_0",   test_batt_valid_0);
    CU_add_test(batt_suite, "invalid",   test_batt_invalid_neg1);

    const_suite = CU_add_suite("constants", NULL, NULL);
    CU_add_test(const_suite, "device_count",   test_device_count);
    CU_add_test(const_suite, "alert_buf_size", test_alert_buf_size);
    CU_add_test(const_suite, "room_buf_size",  test_room_buf_size);
    CU_add_test(const_suite, "alert_msg_size", test_alert_msg_size);

    phantom_suite = CU_add_suite("phantom_widget_fix", NULL, NULL);
    CU_add_test(phantom_suite, "update_when_active",      test_reed_slot_update_when_active);
    CU_add_test(phantom_suite, "no_update_when_inactive", test_reed_slot_no_update_when_inactive);
    CU_add_test(phantom_suite, "preserves_previous",      test_reed_slot_no_update_preserves_previous);
    CU_add_test(phantom_suite, "overwrites_when_active",  test_reed_slot_update_overwrites_when_active);

    /* Run with basic runner then emit JUnit XML to the absolute path baked  */
    /* in by CMake so the file always lands in build/ regardless of ctest cwd */
    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();

    FILE *f = fopen(JUNIT_OUTPUT_PATH, "w");
    if (f) {
        int failures = CU_get_number_of_failures();
        int tests    = CU_get_number_of_tests_run();
        fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        fprintf(f, "<testsuite name=\"controller_tests\" tests=\"%d\" failures=\"%d\">\n",
                tests, failures);
        CU_pSuite s = CU_get_registry()->pSuite;
        while (s) {
            CU_pTest t = s->pTest;
            while (t) {
                fprintf(f, "  <testcase name=\"%s.%s\"/>\n", s->pName, t->pName);
                t = t->pNext;
            }
            s = s->pNext;
        }
        fprintf(f, "</testsuite>\n");
        fclose(f);
    }

    CU_cleanup_registry();
    return CU_get_number_of_failures() > 0 ? 1 : 0;
}
