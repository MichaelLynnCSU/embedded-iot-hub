#include "motor.h"

/*
 * motor_stub.c
 * Test replacement for motor.c.
 * Records the last motor_drive() command so tests can assert
 * that the correct motor direction was requested.
 */

int     g_stub_motor_init_ret  =  0;  /* 0 = success */
int     g_stub_motor_last_cmd  = -1;  /* -1 = never driven */
uint8_t g_stub_motor_drive_count = 0;

int motor_init(void)
{
    return g_stub_motor_init_ret;
}

void motor_drive(uint8_t cmd)
{
    g_stub_motor_last_cmd = (int)cmd;
    g_stub_motor_drive_count++;
}
