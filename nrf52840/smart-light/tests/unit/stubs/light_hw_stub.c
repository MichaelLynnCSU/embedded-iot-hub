#include "light_hw.h"

/*
 * light_hw_stub.c
 * Test replacement for light_hw_zephyr.c.
 * No GPIO touched. g_stub_light_state records the last light_hw_set()
 * call so tests can assert output state without hardware.
 */

int     g_stub_light_state   = -1;  /* -1 = never set    */
int     g_stub_hw_init_ret   =  0;  /* 0  = success      */
int     g_stub_set_call_count =  0;

int light_hw_init(void)
{
    return g_stub_hw_init_ret;
}

void light_hw_set(uint8_t state)
{
    g_stub_light_state = (int)state;
    g_stub_set_call_count++;
}
