#include <stdint.h>
void trinity_log_event(const char *p) { (void)p; }
void trinity_wdt_kick(void) {}
void trinity_wdt_add(void)  {}

/* Controllable PI stub values -- set from test before calling run_pi_controller() */
float g_stub_kp       = 1.0f;
float g_stub_ki       = 0.05f;
float g_stub_kd       = 0.0f;
int   g_stub_setpoint = 25;
int   g_stub_avg_temp = 25;

float aws_get_kp(void)      { return g_stub_kp; }
float aws_get_ki(void)      { return g_stub_ki; }
float aws_get_kd(void)      { return g_stub_kd; }
int   aws_get_setpoint(void){ return g_stub_setpoint; }
int   uart_get_avg_temp(void){ return g_stub_avg_temp; }

/* ble_internal stubs */
void stamp_device(int idx) { (void)idx; }
uint16_t ble_get_device_age_s(int idx) { (void)idx; return 0; }

/* vroom bus stubs */
void bus_publish_pir(uint8_t slot, uint32_t count, int batt)
{ (void)slot;(void)count;(void)batt; }
