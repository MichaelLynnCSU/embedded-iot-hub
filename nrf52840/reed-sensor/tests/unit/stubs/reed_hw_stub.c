#include "reed_hw.h"

int g_stub_hw_read_ret = 1;
int g_stub_hw_init_ret = 0;
int g_stub_led_state   = 0;

int reed_hw_init(void)
{
    return g_stub_hw_init_ret;
}

int reed_hw_read(void)
{
    return g_stub_hw_read_ret;
}

void reed_hw_set_led(int state)
{
    g_stub_led_state = state;
}
