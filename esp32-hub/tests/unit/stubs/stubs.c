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
