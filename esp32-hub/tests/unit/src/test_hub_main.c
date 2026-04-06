#include "unity.h"
#include "hub_logic.h"

void setUp(void) {}
void tearDown(void) {}

/******************************************************************************
 * Timing constants
 * WDT fix: DRAIN_INTERVAL_MS and WIFI_POLL_INTERVAL_MS must be < WDT timeout
 ******************************************************************************/
void test_drain_interval_less_than_wdt(void)
{
    /* WDT timeout is 5s on hub IDF */
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
    RUN_TEST(test_reed_is_offline_false);
    RUN_TEST(test_reed_is_offline_true);
    RUN_TEST(test_reed_should_remove_false);
    RUN_TEST(test_reed_should_remove_true);
    RUN_TEST(test_reed_name_match);
    RUN_TEST(test_reed_name_no_match);
    RUN_TEST(test_wifi_backoff_table_size);
    RUN_TEST(test_wifi_backoff_values);
    RUN_TEST(test_wifi_backoff_monotonically_increasing);
    RUN_TEST(test_default_avg_temp);
    RUN_TEST(test_default_aws_low);
    RUN_TEST(test_default_aws_high);
    RUN_TEST(test_default_temp_in_range);
    RUN_TEST(test_block_count_max);

    return UNITY_END();
}
