#include "lock_hw.h"

/*
 * lock_hw_stub.c
 * Test replacement for lock_hw_zephyr.c.
 * No GPIO touched. g_stub_led_state records the last set_led() call
 * so tests can assert LED behaviour without hardware.
 */

int g_stub_led_state    = -1;  /* -1 = never set */
int g_stub_hw_init_ret  =  0;  /* 0 = success    */

int lock_hw_init(void)
{
    return g_stub_hw_init_ret;
}

void lock_hw_set_led(int state)
{
    g_stub_led_state = state;
}
