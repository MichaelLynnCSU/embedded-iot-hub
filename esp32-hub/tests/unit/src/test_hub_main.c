#include "unity.h"
#include "hub_logic.h"
#include "pir_window.h"

void setUp(void) {}
void tearDown(void) {}

/******************************************************************************
 * Timing constants
 * WDT fix: DRAIN_INTERVAL_MS and WIFI_POLL_INTERVAL_MS must be < WDT timeout
 ******************************************************************************/
void test_drain_interval_less_than_wdt(void)
{
    TEST_ASSERT_LESS_THAN_UINT32(5000u, DRAIN_INTERVAL_MS);
}

void test_wifi_poll_interval_less_than_wdt(void)
{
    TEST_ASSERT_LESS_THAN_UINT32(5000u, WIFI_POLL_INTERVAL_MS);
}

void test_tcp_send_interval_less_than_wdt(void)
{
    TEST_ASSERT_LESS_THAN_UINT32(5000u, TCP_SEND_INTERVAL_MS);
}

void test_stats_interval_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(60000u, STATS_INTERVAL_MS);
}

void test_aws_send_interval_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(300000u, AWS_SEND_INTERVAL_MS);
}

void test_bb_connect_timeout_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(2000u, BB_CONNECT_TIMEOUT_MS);
}

void test_c3_connect_timeout_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(10000u, C3_CONNECT_TIMEOUT_MS);
}

/******************************************************************************
 * Reed slot constants
 ******************************************************************************/
void test_max_reeds(void)
{
    TEST_ASSERT_EQUAL_INT(6, MAX_REEDS);
}

void test_reed_offline_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(150000u, REED_OFFLINE_MS);
}

void test_reed_remove_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(3600000u, REED_REMOVE_MS);
}

void test_reed_offline_s(void)
{
    TEST_ASSERT_EQUAL_INT(150, REED_OFFLINE_S);
}

void test_cooldown_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(10000u, COOLDOWN_MS);
}

/******************************************************************************
 * MFG data byte indices
 * Hub scanner reads fixed byte positions -- changes break all BLE parsing
 ******************************************************************************/
void test_pir_batt_idx(void)
{
    TEST_ASSERT_EQUAL_INT(6, MFG_PIR_BATT_IDX);
}

void test_pir_min_len(void)
{
    TEST_ASSERT_EQUAL_INT(6, MFG_PIR_MIN_LEN);
}

void test_reed_state_idx(void)
{
    TEST_ASSERT_EQUAL_INT(1, MFG_REED_STATE_IDX);
}

void test_reed_batt_idx(void)
{
    TEST_ASSERT_EQUAL_INT(2, MFG_REED_BATT_IDX);
}

void test_light_state_idx(void)
{
    TEST_ASSERT_EQUAL_INT(2, MFG_LIGHT_STATE_IDX);
}

void test_light_min_len(void)
{
    TEST_ASSERT_EQUAL_INT(2, MFG_LIGHT_MIN_LEN);
}

void test_lock_state_idx(void)
{
    TEST_ASSERT_EQUAL_INT(1, MFG_LOCK_STATE_IDX);
}

void test_lock_batt_idx(void)
{
    TEST_ASSERT_EQUAL_INT(2, MFG_LOCK_BATT_IDX);
}

void test_lock_min_len(void)
{
    TEST_ASSERT_EQUAL_INT(3, MFG_LOCK_MIN_LEN);
}

/******************************************************************************
 * PIR count big-endian unpacking
 ******************************************************************************/
void test_pir_unpack_zero(void)
{
    uint8_t mfg[7] = {0xFF, 0xFF, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT32(0, hub_unpack_pir_count(mfg));
}

void test_pir_unpack_max(void)
{
    uint8_t mfg[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x64};
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFF, hub_unpack_pir_count(mfg));
}

void test_pir_unpack_known(void)
{
    uint8_t mfg[7] = {0xFF, 0xFF, 0x01, 0x02, 0x03, 0x04, 0x64};
    TEST_ASSERT_EQUAL_UINT32(0x01020304, hub_unpack_pir_count(mfg));
}

void test_pir_unpack_big_endian_byte_order(void)
{
    uint8_t mfg[7] = {0xFF, 0xFF, 0x00, 0x00, 0x01, 0x00, 0};
    TEST_ASSERT_EQUAL_UINT32(256u, hub_unpack_pir_count(mfg));
}

/******************************************************************************
 * Reed slot age logic
 ******************************************************************************/
void test_reed_offline_flag_active(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, hub_reed_offline_flag(100));
}

void test_reed_offline_flag_at_threshold(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, hub_reed_offline_flag(150));
}

void test_reed_offline_flag_over_threshold(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, hub_reed_offline_flag(151));
}

void test_reed_offline_flag_zero(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, hub_reed_offline_flag(0));
}

void test_reed_offline_flag_just_below(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, hub_reed_offline_flag(149));
}

void test_reed_is_offline_false(void)
{
    TEST_ASSERT_FALSE(hub_reed_is_offline(149000u));
}

void test_reed_is_offline_true(void)
{
    TEST_ASSERT_TRUE(hub_reed_is_offline(150001u));
}

void test_reed_should_remove_false(void)
{
    TEST_ASSERT_FALSE(hub_reed_should_remove(3599999u));
}

void test_reed_should_remove_true(void)
{
    TEST_ASSERT_TRUE(hub_reed_should_remove(3600001u));
}

/******************************************************************************
 * Reed name prefix matching
 ******************************************************************************/
void test_reed_name_match(void)
{
    TEST_ASSERT_TRUE(hub_is_reed_name("ReedSensor1"));
    TEST_ASSERT_TRUE(hub_is_reed_name("ReedSensor99"));
}

void test_reed_name_no_match(void)
{
    TEST_ASSERT_FALSE(hub_is_reed_name("SmartLock"));
    TEST_ASSERT_FALSE(hub_is_reed_name("LightNF"));
    TEST_ASSERT_FALSE(hub_is_reed_name("PIR_Motion"));
    TEST_ASSERT_FALSE(hub_is_reed_name(""));
}

void test_reed_name_prefix(void)
{
    TEST_ASSERT_EQUAL_STRING("ReedSensor", REED_NAME_PREFIX);
}

void test_reed_name_prefix_len(void)
{
    TEST_ASSERT_EQUAL_INT(strlen(REED_NAME_PREFIX), REED_NAME_PREFIX_LEN);
}

/******************************************************************************
 * WiFi backoff table
 ******************************************************************************/
void test_wifi_backoff_table_size(void)
{
    TEST_ASSERT_EQUAL_INT(5, WIFI_BACKOFF_TABLE_SIZE);
}

void test_wifi_backoff_values(void)
{
    TEST_ASSERT_EQUAL_INT(2,  hub_wifi_backoff_sec[0]);
    TEST_ASSERT_EQUAL_INT(5,  hub_wifi_backoff_sec[1]);
    TEST_ASSERT_EQUAL_INT(10, hub_wifi_backoff_sec[2]);
    TEST_ASSERT_EQUAL_INT(30, hub_wifi_backoff_sec[3]);
    TEST_ASSERT_EQUAL_INT(60, hub_wifi_backoff_sec[4]);
}

void test_wifi_backoff_monotonically_increasing(void)
{
    for (int i = 1; i < WIFI_BACKOFF_TABLE_SIZE; i++)
    {
        TEST_ASSERT_GREATER_THAN_INT(hub_wifi_backoff_sec[i-1],
                                     hub_wifi_backoff_sec[i]);
    }
}

/******************************************************************************
 * Default values
 ******************************************************************************/
void test_default_avg_temp(void)
{
    TEST_ASSERT_EQUAL_INT(25, DEFAULT_AVG_TEMP);
}

void test_default_aws_low(void)
{
    TEST_ASSERT_EQUAL_INT(20, DEFAULT_AWS_LOW);
}

void test_default_aws_high(void)
{
    TEST_ASSERT_EQUAL_INT(35, DEFAULT_AWS_HIGH);
}

void test_default_temp_in_range(void)
{
    TEST_ASSERT_GREATER_OR_EQUAL_INT(DEFAULT_AWS_LOW,  DEFAULT_AVG_TEMP);
    TEST_ASSERT_LESS_OR_EQUAL_INT(DEFAULT_AWS_HIGH, DEFAULT_AVG_TEMP);
}

void test_block_count_max(void)
{
    TEST_ASSERT_EQUAL_INT(5, BLOCK_COUNT_MAX);
}

void test_default_aws_motor(void)
{
    TEST_ASSERT_EQUAL_INT(0, DEFAULT_AWS_MOTOR);
}

void test_default_motion_count(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, DEFAULT_MOTION_COUNT);
}

/******************************************************************************
 * TCP state machine constants
 ******************************************************************************/
void test_tcp_state_disconnected(void)
{
    TEST_ASSERT_EQUAL_INT(0, TCP_STATE_DISCONNECTED);
}

void test_tcp_state_connecting(void)
{
    TEST_ASSERT_EQUAL_INT(1, TCP_STATE_CONNECTING);
}

void test_tcp_state_connected(void)
{
    TEST_ASSERT_EQUAL_INT(2, TCP_STATE_CONNECTED);
}

void test_sock_invalid(void)
{
    TEST_ASSERT_EQUAL_INT(-1, SOCK_INVALID);
}

void test_tcp_states_are_ordered(void)
{
    TEST_ASSERT_LESS_THAN_INT(TCP_STATE_CONNECTING,  TCP_STATE_DISCONNECTED);
    TEST_ASSERT_LESS_THAN_INT(TCP_STATE_CONNECTED,   TCP_STATE_CONNECTING);
}

/******************************************************************************
 * PIR count byte indices
 ******************************************************************************/
void test_pir_count_byte_indices(void)
{
    TEST_ASSERT_EQUAL_INT(2, PIR_COUNT_BYTE0);
    TEST_ASSERT_EQUAL_INT(3, PIR_COUNT_BYTE1);
    TEST_ASSERT_EQUAL_INT(4, PIR_COUNT_BYTE2);
    TEST_ASSERT_EQUAL_INT(5, PIR_COUNT_BYTE3);
}

void test_pir_count_bytes_contiguous(void)
{
    TEST_ASSERT_EQUAL_INT(PIR_COUNT_BYTE0 + 1, PIR_COUNT_BYTE1);
    TEST_ASSERT_EQUAL_INT(PIR_COUNT_BYTE1 + 1, PIR_COUNT_BYTE2);
    TEST_ASSERT_EQUAL_INT(PIR_COUNT_BYTE2 + 1, PIR_COUNT_BYTE3);
}

/******************************************************************************
 * PIR window constants
 ******************************************************************************/
void test_pir_window_sec(void)
{
    TEST_ASSERT_EQUAL_UINT32(60u, PIR_WINDOW_SEC);
}

void test_pir_window_threshold(void)
{
    TEST_ASSERT_EQUAL_UINT32(2u, PIR_WINDOW_THRESHOLD);
}

void test_pir_hold_sec(void)
{
    TEST_ASSERT_EQUAL_UINT32(600u, PIR_HOLD_SEC);
}

void test_max_pirs(void)
{
    TEST_ASSERT_EQUAL_UINT32(5u, MAX_PIRS);
}

/******************************************************************************
 * pir_window slot bounds
 ******************************************************************************/
void test_pir_window_get_out_of_range_returns_zero(void)
{
    TEST_ASSERT_EQUAL_INT(0, pir_window_get_occupied(-1));
    TEST_ASSERT_EQUAL_INT(0, pir_window_get_occupied(MAX_PIRS));
    TEST_ASSERT_EQUAL_INT(0, pir_window_get_occupied(99));
}

void test_pir_window_update_out_of_range_no_crash(void)
{
    pir_window_update(-1,      1000u, 1);
    pir_window_update(MAX_PIRS, 1000u, 1);
}

/******************************************************************************
 * pir_window occupancy logic
 ******************************************************************************/
void test_pir_window_no_events_not_occupied(void)
{
    TEST_ASSERT_EQUAL_INT(0, pir_window_get_occupied(0));
}

void test_pir_window_single_event_below_threshold(void)
{
    pir_window_update(1, 1000u, 1);
    pir_window_update(1, 1000u, 0);
    TEST_ASSERT_EQUAL_INT(0, pir_window_get_occupied(1));
}

void test_pir_window_two_events_triggers_occupied(void)
{
    uint32_t t = 10000u;
    pir_window_update(2, t,        1);
    pir_window_update(2, t + 1000, 1);
    TEST_ASSERT_EQUAL_INT(1, pir_window_get_occupied(2));
}

void test_pir_window_hold_persists_after_events_stop(void)
{
    uint32_t t = 20000u;
    pir_window_update(3, t,        1);
    pir_window_update(3, t + 1000, 1);
    pir_window_update(3, t + 300000u, 0);
    TEST_ASSERT_EQUAL_INT(1, pir_window_get_occupied(3));
}

void test_pir_window_hold_expires(void)
{
    uint32_t t = 30000u;
    pir_window_update(4, t,        1);
    pir_window_update(4, t + 1000, 1);
    pir_window_update(4, t + 601000u, 0);
    TEST_ASSERT_EQUAL_INT(0, pir_window_get_occupied(4));
}

void test_pir_window_occ_zero_does_not_add_event(void)
{
    uint32_t t = 40000u;
    pir_window_update(0, t,       0);
    pir_window_update(0, t + 100, 0);
    pir_window_update(0, t + 200, 0);
    TEST_ASSERT_EQUAL_INT(0, pir_window_get_occupied(0));
}

void test_pir_window_slots_independent(void)
{
    uint32_t t = 50000u;
    pir_window_update(0, t,        1);
    pir_window_update(0, t + 1000, 1);
    TEST_ASSERT_EQUAL_INT(1, pir_window_get_occupied(0));
    TEST_ASSERT_EQUAL_INT(0, pir_window_get_occupied(1));
}

/*----------------------------------------------------------------------------*/

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_drain_interval_less_than_wdt);
    RUN_TEST(test_wifi_poll_interval_less_than_wdt);
    RUN_TEST(test_tcp_send_interval_less_than_wdt);
    RUN_TEST(test_stats_interval_ms);
    RUN_TEST(test_aws_send_interval_ms);
    RUN_TEST(test_bb_connect_timeout_ms);
    RUN_TEST(test_c3_connect_timeout_ms);
    RUN_TEST(test_max_reeds);
    RUN_TEST(test_reed_offline_ms);
    RUN_TEST(test_reed_remove_ms);
    RUN_TEST(test_reed_offline_s);
    RUN_TEST(test_cooldown_ms);
    RUN_TEST(test_pir_batt_idx);
    RUN_TEST(test_pir_min_len);
    RUN_TEST(test_reed_state_idx);
    RUN_TEST(test_reed_batt_idx);
    RUN_TEST(test_light_state_idx);
    RUN_TEST(test_light_min_len);
    RUN_TEST(test_lock_state_idx);
    RUN_TEST(test_lock_batt_idx);
    RUN_TEST(test_lock_min_len);
    RUN_TEST(test_pir_unpack_zero);
    RUN_TEST(test_pir_unpack_max);
    RUN_TEST(test_pir_unpack_known);
    RUN_TEST(test_pir_unpack_big_endian_byte_order);
    RUN_TEST(test_reed_offline_flag_active);
    RUN_TEST(test_reed_offline_flag_at_threshold);
    RUN_TEST(test_reed_offline_flag_over_threshold);
    RUN_TEST(test_reed_offline_flag_zero);
    RUN_TEST(test_reed_offline_flag_just_below);
    RUN_TEST(test_reed_is_offline_false);
    RUN_TEST(test_reed_is_offline_true);
    RUN_TEST(test_reed_should_remove_false);
    RUN_TEST(test_reed_should_remove_true);
    RUN_TEST(test_reed_name_match);
    RUN_TEST(test_reed_name_no_match);
    RUN_TEST(test_reed_name_prefix);
    RUN_TEST(test_reed_name_prefix_len);
    RUN_TEST(test_wifi_backoff_table_size);
    RUN_TEST(test_wifi_backoff_values);
    RUN_TEST(test_wifi_backoff_monotonically_increasing);
    RUN_TEST(test_default_avg_temp);
    RUN_TEST(test_default_aws_low);
    RUN_TEST(test_default_aws_high);
    RUN_TEST(test_default_temp_in_range);
    RUN_TEST(test_block_count_max);
    RUN_TEST(test_default_aws_motor);
    RUN_TEST(test_default_motion_count);
    RUN_TEST(test_tcp_state_disconnected);
    RUN_TEST(test_tcp_state_connecting);
    RUN_TEST(test_tcp_state_connected);
    RUN_TEST(test_sock_invalid);
    RUN_TEST(test_tcp_states_are_ordered);
    RUN_TEST(test_pir_count_byte_indices);
    RUN_TEST(test_pir_count_bytes_contiguous);
    RUN_TEST(test_pir_window_sec);
    RUN_TEST(test_pir_window_threshold);
    RUN_TEST(test_pir_hold_sec);
    RUN_TEST(test_max_pirs);
    RUN_TEST(test_pir_window_get_out_of_range_returns_zero);
    RUN_TEST(test_pir_window_update_out_of_range_no_crash);
    RUN_TEST(test_pir_window_no_events_not_occupied);
    RUN_TEST(test_pir_window_single_event_below_threshold);
    RUN_TEST(test_pir_window_two_events_triggers_occupied);
    RUN_TEST(test_pir_window_hold_persists_after_events_stop);
    RUN_TEST(test_pir_window_hold_expires);
    RUN_TEST(test_pir_window_occ_zero_does_not_add_event);
    RUN_TEST(test_pir_window_slots_independent);

    return UNITY_END();
}
