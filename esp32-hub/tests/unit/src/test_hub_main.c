#include "ble_manager.h"
#include "ble_temp.h"
#include "ble_light.h"
#include "ble_lock.h"
#include "ble_reed.h"
#include "ble_pir.h"
#include "pi_controller.h"
#include "pi_stubs.h"
#include "motor_sm.h"
#include "unity.h"
#include "hub_logic.h"
#include "pir_window.h"


extern uint32_t g_stub_tick_ms;

void setUp(void)
{
    g_stub_tick_ms = 0;
    ble_pir_preinit();
    ble_reed_preinit();
    ble_temp_preinit();
}
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

/******************************************************************************
 * config.h constants -- PIR/temp slot table, PI defaults, PWM ceiling
 ******************************************************************************/
void test_pir_name_prefix(void)
{
    TEST_ASSERT_EQUAL_STRING("PIR_Motion", PIR_NAME_PREFIX);
}

void test_pir_name_prefix_len(void)
{
    TEST_ASSERT_EQUAL_UINT32(10u, PIR_NAME_PREFIX_LEN);
}

void test_pir_offline_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(150000u, PIR_OFFLINE_MS);
}

void test_pir_remove_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(3600000u, PIR_REMOVE_MS);
}

void test_pir_offline_s(void)
{
    TEST_ASSERT_EQUAL_INT(300, PIR_OFFLINE_S);
}

void test_max_temps(void)
{
    TEST_ASSERT_EQUAL_UINT32(4u, MAX_TEMPS);
}

void test_temp_name_prefix(void)
{
    TEST_ASSERT_EQUAL_STRING("TempSensor", TEMP_NAME_PREFIX);
}

void test_temp_name_prefix_len(void)
{
    TEST_ASSERT_EQUAL_UINT32(10u, TEMP_NAME_PREFIX_LEN);
}

void test_temp_offline_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(150000u, TEMP_OFFLINE_MS);
}

void test_temp_remove_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(3600000u, TEMP_REMOVE_MS);
}

void test_pwm_duty_max(void)
{
    TEST_ASSERT_EQUAL_INT(1023, PWM_DUTY_MAX);
}

void test_default_aws_kp(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, DEFAULT_GATEWAY_KP);
}

void test_default_aws_ki(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.05f, DEFAULT_GATEWAY_KI);
}

void test_default_aws_kd(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, DEFAULT_GATEWAY_KD);
}

void test_default_aws_setpoint(void)
{
    TEST_ASSERT_EQUAL_INT(25, DEFAULT_GATEWAY_SETPOINT);
}

/******************************************************************************
 * motor_sm -- state machine transitions
 ******************************************************************************/
static void motor_sm_force_idle(void)
{
    bool c = false, d = false;
    /* Drive to RUNNING then past full min-run and cooldown to get back to IDLE */
    uint32_t t = 9000000u; /* far future -- avoids colliding with other tests */
    run_motor_sm(50.0f, t, &c, &d);
    run_motor_sm(0.0f,  t + MIN_RUN_MS + 1u, &c, &d);
    run_motor_sm(0.0f,  t + MIN_RUN_MS + (5UL*60UL*1000UL) + 1u, &c, &d);
}

void test_motor_sm_initial_state_is_idle(void)
{
    motor_sm_force_idle();
    TEST_ASSERT_EQUAL_UINT8(MOTOR_IDLE, motor_sm_get_state());
}

void test_motor_sm_idle_zero_pwm_stays_idle(void)
{
    motor_sm_force_idle();
    bool connect = false, disconnect = false;
    run_motor_sm(0.0f, 100000u, &connect, &disconnect);
    TEST_ASSERT_EQUAL_UINT8(MOTOR_IDLE, motor_sm_get_state());
    TEST_ASSERT_FALSE(connect);
}

void test_motor_sm_idle_positive_pwm_starts_running(void)
{
    motor_sm_force_idle();
    bool connect = false, disconnect = false;
    float pwm = run_motor_sm(50.0f, 200000u, &connect, &disconnect);
    TEST_ASSERT_EQUAL_UINT8(MOTOR_RUNNING, motor_sm_get_state());
    TEST_ASSERT_TRUE(connect);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, pwm);
}

void test_motor_sm_running_min_run_enforced(void)
{
    motor_sm_force_idle();
    bool connect = false, disconnect = false;
    uint32_t t = 300000u;
    run_motor_sm(50.0f, t, &connect, &disconnect);
    float pwm = run_motor_sm(0.0f, t + 1000u, &connect, &disconnect);
    TEST_ASSERT_EQUAL_UINT8(MOTOR_RUNNING, motor_sm_get_state());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, pwm);
}

void test_motor_sm_running_to_cooldown_after_min_run(void)
{
    motor_sm_force_idle();
    bool connect = false, disconnect = false;
    uint32_t t = 400000u;
    run_motor_sm(50.0f, t, &connect, &disconnect);
    float pwm = run_motor_sm(0.0f, t + MIN_RUN_MS + 1u, &connect, &disconnect);
    TEST_ASSERT_EQUAL_UINT8(MOTOR_COOLDOWN, motor_sm_get_state());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pwm);
}

void test_motor_sm_cooldown_to_idle_after_cooldown_ms(void)
{
    motor_sm_force_idle();
    bool connect = false, disconnect = false;
    uint32_t t = 500000u;
    run_motor_sm(50.0f, t, &connect, &disconnect);
    run_motor_sm(0.0f,  t + MIN_RUN_MS + 1u, &connect, &disconnect);
    run_motor_sm(0.0f,  t + MIN_RUN_MS + (5UL*60UL*1000UL) + 1u,
                 &connect, &disconnect);
    TEST_ASSERT_EQUAL_UINT8(MOTOR_IDLE, motor_sm_get_state());
    TEST_ASSERT_TRUE(disconnect);
}

void test_motor_sm_cooldown_pwm_is_zero(void)
{
    motor_sm_force_idle();
    bool connect = false, disconnect = false;
    uint32_t t = 600000u;
    run_motor_sm(50.0f, t, &connect, &disconnect);
    run_motor_sm(0.0f,  t + MIN_RUN_MS + 1u, &connect, &disconnect);
    float pwm = run_motor_sm(0.0f, t + MIN_RUN_MS + 2u, &connect, &disconnect);
    TEST_ASSERT_EQUAL_UINT8(MOTOR_COOLDOWN, motor_sm_get_state());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pwm);
}

void test_motor_sm_timing_constants(void)
{
    /* MIN_RUN_MS = 10 min, COOLDOWN_MS = 5 min -- from motor_sm.h */
    TEST_ASSERT_EQUAL_UINT32(600000u,  MIN_RUN_MS);
    TEST_ASSERT_EQUAL_UINT32(300000u,  (5UL * 60UL * 1000UL));
}

/******************************************************************************
 * pi_controller -- output clamping, proportional term, anti-windup
 ******************************************************************************/
void test_pi_output_max(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, PWM_OUT_MAX);
}

void test_pi_output_min(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, PWM_OUT_MIN);
}

void test_pi_at_setpoint_output_zero(void)
{
    /* error=0, integral accumulates but kp*0=0, ki*integral grows slowly */
    g_stub_kp       = 1.0f;
    g_stub_ki       = 0.0f;
    g_stub_kd       = 0.0f;
    g_stub_setpoint = 25;
    g_stub_avg_temp = 25;
    float out = run_pi_controller(1000u);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out);
}

void test_pi_positive_error_produces_positive_output(void)
{
    g_stub_kp       = 1.0f;
    g_stub_ki       = 0.0f;
    g_stub_kd       = 0.0f;
    g_stub_setpoint = 30;
    g_stub_avg_temp = 25;
    float out = run_pi_controller(2000u);
    TEST_ASSERT_GREATER_THAN(0.0f, out);
}

void test_pi_negative_error_clamps_to_zero(void)
{
    g_stub_kp       = 1.0f;
    g_stub_ki       = 0.0f;
    g_stub_kd       = 0.0f;
    g_stub_setpoint = 20;
    g_stub_avg_temp = 25;
    float out = run_pi_controller(3000u);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out);
}

void test_pi_output_clamps_at_max(void)
{
    g_stub_kp       = 100.0f;
    g_stub_ki       = 0.0f;
    g_stub_kd       = 0.0f;
    g_stub_setpoint = 50;
    g_stub_avg_temp = 25;
    float out = run_pi_controller(4000u);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, PWM_OUT_MAX, out);
}

void test_pi_proportional_scales_with_error(void)
{
    g_stub_kp       = 2.0f;
    g_stub_ki       = 0.0f;
    g_stub_kd       = 0.0f;
    g_stub_setpoint = 27;
    g_stub_avg_temp = 25;
    /* error=2, kp=2, p_term=4, no integral, output=4 */
    float out = run_pi_controller(5000u);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 4.0f, out);
}

/******************************************************************************
 * ble_pir -- slot table bounds and empty table behaviour
 ******************************************************************************/
void test_ble_pir_get_count_empty(void)
{
    ble_pir_preinit();
    TEST_ASSERT_EQUAL_INT(0, ble_pir_get_count());
}

void test_ble_pir_get_slot_info_oob_negative(void)
{
    ble_pir_preinit();
    TEST_ASSERT_FALSE(ble_pir_get_slot_info(-1, NULL, NULL, NULL));
}

void test_ble_pir_get_slot_info_oob_max(void)
{
    ble_pir_preinit();
    TEST_ASSERT_FALSE(ble_pir_get_slot_info(MAX_PIRS, NULL, NULL, NULL));
}

void test_ble_pir_get_slot_info_empty_slot_returns_false(void)
{
    ble_pir_preinit();
    TEST_ASSERT_FALSE(ble_pir_get_slot_info(0, NULL, NULL, NULL));
}

void test_ble_pir_handle_null_mfg_no_crash(void)
{
    ble_pir_preinit();
    uint8_t mac[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    ble_pir_handle(NULL, 8, mac, "PIR_Motion");
    TEST_ASSERT_EQUAL_INT(0, ble_pir_get_count());
}

void test_ble_pir_handle_short_mfg_no_crash(void)
{
    ble_pir_preinit();
    uint8_t mac[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    uint8_t mfg[3] = {0xFF, 0xFF, 0x00};
    ble_pir_handle(mfg, 3, mac, "PIR_Motion");
    TEST_ASSERT_EQUAL_INT(0, ble_pir_get_count());
}

void test_ble_pir_handle_valid_allocates_slot(void)
{
    ble_pir_preinit();
    uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
    uint8_t mfg[8] = {0xFF,0xFF, 0x00,0x00,0x00,0x05, 0x64, 0x01};
    ble_pir_handle(mfg, 8, mac, "PIR_Motion");
    TEST_ASSERT_EQUAL_INT(1, ble_pir_get_count());
}

void test_ble_pir_handle_valid_stores_count_and_batt(void)
{
    ble_pir_preinit();
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    uint8_t mfg[8] = {0xFF,0xFF, 0x00,0x00,0x00,0x07, 0x50, 0x00};
    ble_pir_handle(mfg, 8, mac, "PIR_Motion");
    uint32_t count = 0;
    int      batt  = 0;
    TEST_ASSERT_TRUE(ble_pir_get_slot_info(0, &count, &batt, NULL));
    TEST_ASSERT_EQUAL_UINT32(7u, count);
    TEST_ASSERT_EQUAL_INT(80, batt);
}

void test_ble_pir_handle_same_mac_updates_slot(void)
{
    ble_pir_preinit();
    uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0x01};
    uint8_t mfg1[8] = {0xFF,0xFF, 0x00,0x00,0x00,0x01, 0x64, 0x00};
    uint8_t mfg2[8] = {0xFF,0xFF, 0x00,0x00,0x00,0x02, 0x50, 0x00};
    ble_pir_handle(mfg1, 8, mac, "PIR_Motion");
    ble_pir_handle(mfg2, 8, mac, "PIR_Motion");
    TEST_ASSERT_EQUAL_INT(1, ble_pir_get_count());
    uint32_t count = 0;
    ble_pir_get_slot_info(0, &count, NULL, NULL);
    TEST_ASSERT_EQUAL_UINT32(2u, count);
}

void test_ble_pir_expire_no_crash_on_empty_table(void)
{
    ble_pir_preinit();
    ble_expire_pir_slots();
    TEST_ASSERT_EQUAL_INT(0, ble_pir_get_count());
}
void test_ble_pir_expire_active_to_offline(void)
{
    uint8_t mfg[8] = {0xAB, 0x00, 0,0,0,1, 70, 0};
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    g_stub_tick_ms = 1000u;
    ble_pir_handle(mfg, 8, mac, "PIR_Motion");
    g_stub_tick_ms = 1000u + PIR_OFFLINE_MS + 1000u;
    ble_expire_pir_slots();
    /* OFFLINE -- slot still visible */
    TEST_ASSERT_EQUAL_INT(1, ble_pir_get_count());
}

void test_ble_pir_expire_offline_to_empty(void)
{
    uint8_t mfg[8] = {0xAB, 0x00, 0,0,0,1, 70, 0};
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    g_stub_tick_ms = 1000u;
    ble_pir_handle(mfg, 8, mac, "PIR_Motion");
    g_stub_tick_ms = 1000u + PIR_REMOVE_MS + 1000u;
    ble_expire_pir_slots();
    /* EMPTY -- slot cleared */
    TEST_ASSERT_EQUAL_INT(0, ble_pir_get_count());
}


/******************************************************************************
 * ble_reed -- slot table bounds and basic behaviour
 ******************************************************************************/
void test_ble_reed_get_count_empty(void)
{
    ble_reed_preinit();
    TEST_ASSERT_EQUAL_INT(0, ble_get_reed_count());
}

void test_ble_reed_get_slot_info_oob_negative(void)
{
    ble_reed_preinit();
    TEST_ASSERT_FALSE(ble_get_reed_slot_info(-1, NULL, NULL, NULL, NULL, NULL));
}

void test_ble_reed_get_slot_info_oob_max(void)
{
    ble_reed_preinit();
    TEST_ASSERT_FALSE(ble_get_reed_slot_info(MAX_REEDS, NULL, NULL, NULL, NULL, NULL));
}

void test_ble_reed_get_slot_info_empty_returns_false(void)
{
    ble_reed_preinit();
    TEST_ASSERT_FALSE(ble_get_reed_slot_info(0, NULL, NULL, NULL, NULL, NULL));
}

void test_ble_reed_handle_null_mfg_still_allocates(void)
{
    ble_reed_preinit();
    uint8_t mac[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    ble_reed_handle(NULL, 0, mac, "ReedSensor1");
    TEST_ASSERT_EQUAL_INT(1, ble_get_reed_count());
}

void test_ble_reed_handle_valid_allocates_slot(void)
{
    ble_reed_preinit();
    uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
    uint8_t mfg[3] = {0xAB, 0x01, 0x64};
    ble_reed_handle(mfg, 3, mac, "ReedSensor1");
    TEST_ASSERT_EQUAL_INT(1, ble_get_reed_count());
}

void test_ble_reed_handle_valid_stores_state_and_batt(void)
{
    ble_reed_preinit();
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    uint8_t mfg[3] = {0xAB, 0x01, 0x50};
    ble_reed_handle(mfg, 3, mac, "ReedSensor1");
    uint8_t  state = 0xFF;
    int      batt  = 0;
    TEST_ASSERT_TRUE(ble_get_reed_slot_info(0, NULL, &batt, NULL, &state, NULL));
    TEST_ASSERT_EQUAL_UINT8(0x01, state);
    TEST_ASSERT_EQUAL_INT(0x50, batt);
}

void test_ble_reed_handle_same_mac_updates_slot(void)
{
    ble_reed_preinit();
    uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0x01};
    uint8_t mfg1[3] = {0xAB, 0x00, 0x64};
    uint8_t mfg2[3] = {0xAB, 0x01, 0x50};
    ble_reed_handle(mfg1, 3, mac, "ReedSensor1");
    ble_reed_handle(mfg2, 3, mac, "ReedSensor1");
    TEST_ASSERT_EQUAL_INT(1, ble_get_reed_count());
    uint8_t state = 0xFF;
    ble_get_reed_slot_info(0, NULL, NULL, NULL, &state, NULL);
    TEST_ASSERT_EQUAL_UINT8(0x01, state);
}

void test_ble_reed_expire_no_crash_on_empty_table(void)
{
    ble_reed_preinit();
    ble_expire_reed_slots();
    TEST_ASSERT_EQUAL_INT(0, ble_get_reed_count());
}
void test_ble_reed_expire_active_to_offline(void)
{
    uint8_t mfg[3] = {0xAC, 0x00, 70};
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    g_stub_tick_ms = 1000u;
    ble_reed_handle(mfg, 3, mac, "ReedSensor1");
    g_stub_tick_ms = 1000u + REED_OFFLINE_MS + 1000u;
    ble_expire_reed_slots();
    /* OFFLINE -- slot still visible */
    TEST_ASSERT_EQUAL_INT(1, ble_get_reed_count());
}

void test_ble_reed_expire_offline_to_empty(void)
{
    uint8_t mfg[3] = {0xAC, 0x00, 70};
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    g_stub_tick_ms = 1000u;
    ble_reed_handle(mfg, 3, mac, "ReedSensor1");
    g_stub_tick_ms = 1000u + REED_REMOVE_MS + 1000u;
    ble_expire_reed_slots();
    /* EMPTY -- slot cleared */
    TEST_ASSERT_EQUAL_INT(0, ble_get_reed_count());
}

void test_ble_reed_recover_offline_to_active(void)
{
    uint8_t mfg[3] = {0xAC, 0x00, 70};
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    g_stub_tick_ms = 1000u;
    ble_reed_handle(mfg, 3, mac, "ReedSensor1");
    /* Go offline */
    g_stub_tick_ms = 1000u + REED_OFFLINE_MS + 1000u;
    ble_expire_reed_slots();
    TEST_ASSERT_EQUAL_INT(1, ble_get_reed_count());
    /* Recover -- new adv received */
    g_stub_tick_ms = 1000u + REED_OFFLINE_MS + 2000u;
    ble_reed_handle(mfg, 3, mac, "ReedSensor1");
    TEST_ASSERT_EQUAL_INT(1, ble_get_reed_count());
}


void test_ble_reed_two_different_macs_fill_two_slots(void)
{
    ble_reed_preinit();
    uint8_t mac1[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    uint8_t mac2[6] = {0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
    uint8_t mfg[3]  = {0xAB, 0x00, 0x64};
    ble_reed_handle(mfg, 3, mac1, "ReedSensor1");
    ble_reed_handle(mfg, 3, mac2, "ReedSensor2");
    TEST_ASSERT_EQUAL_INT(2, ble_get_reed_count());
}

/******************************************************************************
 * ble_temp -- slot table bounds and temperature decode
 ******************************************************************************/
void test_ble_temp_get_count_empty(void)
{
    ble_temp_preinit();
    TEST_ASSERT_EQUAL_INT(0, ble_get_temp_count());
}

void test_ble_temp_get_slot_info_oob_negative(void)
{
    ble_temp_preinit();
    TEST_ASSERT_FALSE(ble_get_temp_slot_info(-1, NULL, NULL, NULL, NULL, NULL));
}

void test_ble_temp_get_slot_info_oob_max(void)
{
    ble_temp_preinit();
    TEST_ASSERT_FALSE(ble_get_temp_slot_info(MAX_TEMPS, NULL, NULL, NULL, NULL, NULL));
}

void test_ble_temp_get_slot_info_empty_returns_false(void)
{
    ble_temp_preinit();
    TEST_ASSERT_FALSE(ble_get_temp_slot_info(0, NULL, NULL, NULL, NULL, NULL));
}

void test_ble_temp_handle_null_mfg_no_crash(void)
{
    ble_temp_preinit();
    uint8_t mac[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    ble_temp_handle(NULL, 4, mac, "TempSensor1");
    TEST_ASSERT_EQUAL_INT(0, ble_get_temp_count());
}

void test_ble_temp_handle_short_mfg_no_crash(void)
{
    ble_temp_preinit();
    uint8_t mac[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    uint8_t mfg[2] = {0xAE, 0x00};
    ble_temp_handle(mfg, 2, mac, "TempSensor1");
    TEST_ASSERT_EQUAL_INT(0, ble_get_temp_count());
}

void test_ble_temp_handle_valid_allocates_slot(void)
{
    ble_temp_preinit();
    uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
    /* temp=250 decidegC (25.0°C) little-endian: 0xFA, 0x00 */
    uint8_t mfg[4] = {0xAE, 0xFA, 0x00, 0x64};
    ble_temp_handle(mfg, 4, mac, "TempSensor1");
    TEST_ASSERT_EQUAL_INT(1, ble_get_temp_count());
}

void test_ble_temp_handle_valid_stores_temp_and_batt(void)
{
    ble_temp_preinit();
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    /* temp=250 decidegC little-endian, batt=80 */
    uint8_t mfg[4] = {0xAE, 0xFA, 0x00, 0x50};
    ble_temp_handle(mfg, 4, mac, "TempSensor1");
    int16_t temp = 0;
    int     batt = 0;
    TEST_ASSERT_TRUE(ble_get_temp_slot_info(0, NULL, &temp, &batt, NULL, NULL));
    TEST_ASSERT_EQUAL_INT16(250, temp);
    TEST_ASSERT_EQUAL_INT(80, batt);
}

void test_ble_temp_handle_negative_temp(void)
{
    ble_temp_preinit();
    uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0x01};
    /* temp=-100 decidegC (-10.0°C): int16 0xFF9C little-endian: 0x9C, 0xFF */
    uint8_t mfg[4] = {0xAE, 0x9C, 0xFF, 0x64};
    ble_temp_handle(mfg, 4, mac, "TempSensor1");
    int16_t temp = 0;
    TEST_ASSERT_TRUE(ble_get_temp_slot_info(0, NULL, &temp, NULL, NULL, NULL));
    TEST_ASSERT_EQUAL_INT16(-100, temp);
}

void test_ble_temp_handle_same_mac_updates_slot(void)
{
    ble_temp_preinit();
    uint8_t mac[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0x02};
    uint8_t mfg1[4] = {0xAE, 0xFA, 0x00, 0x64};
    uint8_t mfg2[4] = {0xAE, 0x04, 0x01, 0x50}; /* temp=260 decidegC */
    ble_temp_handle(mfg1, 4, mac, "TempSensor1");
    ble_temp_handle(mfg2, 4, mac, "TempSensor1");
    TEST_ASSERT_EQUAL_INT(1, ble_get_temp_count());
    int16_t temp = 0;
    ble_get_temp_slot_info(0, NULL, &temp, NULL, NULL, NULL);
    TEST_ASSERT_EQUAL_INT16(260, temp);
}

void test_ble_temp_expire_no_crash_on_empty_table(void)
{
    ble_temp_preinit();
    ble_expire_temp_slots();
    TEST_ASSERT_EQUAL_INT(0, ble_get_temp_count());
}
void test_ble_temp_expire_active_to_offline(void)
{
    uint8_t mfg[4] = {0xAE, 0xFD, 0x00, 60};
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    g_stub_tick_ms = 1000u;
    ble_temp_handle(mfg, 4, mac, "TempSensor1");
    g_stub_tick_ms = 1000u + TEMP_OFFLINE_MS + 1000u;
    ble_expire_temp_slots();
    /* OFFLINE -- slot still visible */
    TEST_ASSERT_EQUAL_INT(1, ble_get_temp_count());
}

void test_ble_temp_expire_offline_to_empty(void)
{
    uint8_t mfg[4] = {0xAE, 0xFD, 0x00, 60};
    uint8_t mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    g_stub_tick_ms = 1000u;
    ble_temp_handle(mfg, 4, mac, "TempSensor1");
    g_stub_tick_ms = 1000u + TEMP_REMOVE_MS + 1000u;
    ble_expire_temp_slots();
    /* EMPTY -- slot cleared */
    TEST_ASSERT_EQUAL_INT(0, ble_get_temp_count());
}


void test_ble_temp_two_different_macs_fill_two_slots(void)
{
    ble_temp_preinit();
    uint8_t mac1[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    uint8_t mac2[6] = {0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
    uint8_t mfg[4]  = {0xAE, 0xFA, 0x00, 0x64};
    ble_temp_handle(mfg, 4, mac1, "TempSensor1");
    ble_temp_handle(mfg, 4, mac2, "TempSensor2");
    TEST_ASSERT_EQUAL_INT(2, ble_get_temp_count());
}

/******************************************************************************
 * ble_light -- getter/setter and pending state logic
 ******************************************************************************/
void test_ble_light_initial_state_is_zero(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, ble_light_get_state());
}

void test_ble_light_update_adv_stores_state(void)
{
    ble_light_update_adv(1);
    TEST_ASSERT_EQUAL_UINT8(1, ble_light_get_state());
    ble_light_update_adv(0);
    TEST_ASSERT_EQUAL_UINT8(0, ble_light_get_state());
}

void test_ble_light_send_command_not_connected_queues(void)
{
    /* Not connected -- must queue without crash */
    ble_send_light_command(1);
    /* No assert needed -- just must not crash */
    ble_send_light_command(0);
}

/******************************************************************************
 * ble_lock -- getter/setter and pending state logic
 ******************************************************************************/
void test_ble_lock_initial_state_is_zero(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, ble_lock_get_state());
}

void test_ble_lock_initial_batt_is_minus_one(void)
{
    TEST_ASSERT_EQUAL_INT(-1, ble_lock_get_batt());
}

void test_ble_lock_update_adv_stores_state_and_batt(void)
{
    ble_lock_update_adv(1, 75);
    TEST_ASSERT_EQUAL_UINT8(1, ble_lock_get_state());
    TEST_ASSERT_EQUAL_INT(75, ble_lock_get_batt());
}

void test_ble_lock_update_adv_state_zero(void)
{
    ble_lock_update_adv(0, 50);
    TEST_ASSERT_EQUAL_UINT8(0, ble_lock_get_state());
    TEST_ASSERT_EQUAL_INT(50, ble_lock_get_batt());
}

void test_ble_lock_send_command_not_connected_queues(void)
{
    /* Not connected -- must queue without crash */
    ble_send_lock_command(1);
    ble_send_lock_command(0);
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
    /* config.h -- PIR slot table */
    RUN_TEST(test_pir_name_prefix);
    RUN_TEST(test_pir_name_prefix_len);
    RUN_TEST(test_pir_offline_ms);
    RUN_TEST(test_pir_remove_ms);
    RUN_TEST(test_pir_offline_s);

    /* config.h -- temp slot table */
    RUN_TEST(test_max_temps);
    RUN_TEST(test_temp_name_prefix);
    RUN_TEST(test_temp_name_prefix_len);
    RUN_TEST(test_temp_offline_ms);
    RUN_TEST(test_temp_remove_ms);

    /* config.h -- PI defaults and PWM */
    RUN_TEST(test_pwm_duty_max);
    RUN_TEST(test_default_aws_kp);
    RUN_TEST(test_default_aws_ki);
    RUN_TEST(test_default_aws_kd);
    RUN_TEST(test_default_aws_setpoint);

    /* motor_sm */
    RUN_TEST(test_motor_sm_initial_state_is_idle);
    RUN_TEST(test_motor_sm_idle_zero_pwm_stays_idle);
    RUN_TEST(test_motor_sm_idle_positive_pwm_starts_running);
    RUN_TEST(test_motor_sm_running_min_run_enforced);
    RUN_TEST(test_motor_sm_running_to_cooldown_after_min_run);
    RUN_TEST(test_motor_sm_cooldown_to_idle_after_cooldown_ms);
    RUN_TEST(test_motor_sm_cooldown_pwm_is_zero);
    RUN_TEST(test_motor_sm_timing_constants);

    /* pi_controller */
    RUN_TEST(test_pi_output_max);
    RUN_TEST(test_pi_output_min);
    RUN_TEST(test_pi_at_setpoint_output_zero);
    RUN_TEST(test_pi_positive_error_produces_positive_output);
    RUN_TEST(test_pi_negative_error_clamps_to_zero);
    RUN_TEST(test_pi_output_clamps_at_max);
    RUN_TEST(test_pi_proportional_scales_with_error);

    /* ble_pir */
    RUN_TEST(test_ble_pir_get_count_empty);
    RUN_TEST(test_ble_pir_get_slot_info_oob_negative);
    RUN_TEST(test_ble_pir_get_slot_info_oob_max);
    RUN_TEST(test_ble_pir_get_slot_info_empty_slot_returns_false);
    RUN_TEST(test_ble_pir_handle_null_mfg_no_crash);
    RUN_TEST(test_ble_pir_handle_short_mfg_no_crash);
    RUN_TEST(test_ble_pir_handle_valid_allocates_slot);
    RUN_TEST(test_ble_pir_handle_valid_stores_count_and_batt);
    RUN_TEST(test_ble_pir_handle_same_mac_updates_slot);
    RUN_TEST(test_ble_pir_expire_no_crash_on_empty_table);

    /* ble_reed */
    RUN_TEST(test_ble_reed_get_count_empty);
    RUN_TEST(test_ble_reed_get_slot_info_oob_negative);
    RUN_TEST(test_ble_reed_get_slot_info_oob_max);
    RUN_TEST(test_ble_reed_get_slot_info_empty_returns_false);
    RUN_TEST(test_ble_reed_handle_null_mfg_still_allocates);
    RUN_TEST(test_ble_reed_handle_valid_allocates_slot);
    RUN_TEST(test_ble_reed_handle_valid_stores_state_and_batt);
    RUN_TEST(test_ble_reed_handle_same_mac_updates_slot);
    RUN_TEST(test_ble_reed_expire_no_crash_on_empty_table);
    RUN_TEST(test_ble_reed_two_different_macs_fill_two_slots);

    /* ble_temp */
    RUN_TEST(test_ble_temp_get_count_empty);
    RUN_TEST(test_ble_temp_get_slot_info_oob_negative);
    RUN_TEST(test_ble_temp_get_slot_info_oob_max);
    RUN_TEST(test_ble_temp_get_slot_info_empty_returns_false);
    RUN_TEST(test_ble_temp_handle_null_mfg_no_crash);
    RUN_TEST(test_ble_temp_handle_short_mfg_no_crash);
    RUN_TEST(test_ble_temp_handle_valid_allocates_slot);
    RUN_TEST(test_ble_temp_handle_valid_stores_temp_and_batt);
    RUN_TEST(test_ble_temp_handle_negative_temp);
    RUN_TEST(test_ble_temp_handle_same_mac_updates_slot);
    RUN_TEST(test_ble_temp_expire_no_crash_on_empty_table);
    RUN_TEST(test_ble_temp_two_different_macs_fill_two_slots);

    /* ble_light */
    RUN_TEST(test_ble_light_initial_state_is_zero);
    RUN_TEST(test_ble_light_update_adv_stores_state);
    RUN_TEST(test_ble_light_send_command_not_connected_queues);

    /* ble_lock */
    RUN_TEST(test_ble_lock_initial_state_is_zero);
    RUN_TEST(test_ble_lock_initial_batt_is_minus_one);
    RUN_TEST(test_ble_lock_update_adv_stores_state_and_batt);
    RUN_TEST(test_ble_lock_update_adv_state_zero);
    RUN_TEST(test_ble_lock_send_command_not_connected_queues);

    RUN_TEST(test_ble_pir_expire_active_to_offline);
    RUN_TEST(test_ble_pir_expire_offline_to_empty);
    RUN_TEST(test_ble_reed_expire_active_to_offline);
    RUN_TEST(test_ble_reed_expire_offline_to_empty);
    RUN_TEST(test_ble_reed_recover_offline_to_active);
    RUN_TEST(test_ble_temp_expire_active_to_offline);
    RUN_TEST(test_ble_temp_expire_offline_to_empty);
    return UNITY_END();
}
