#include "temp_hw.h"

int16_t g_stub_hw_raw = 0;
int     g_stub_hw_ret = 0;

int temp_hw_init(void)
{
    return g_stub_hw_ret;
}

int temp_hw_read_raw(int16_t *out_raw)
{
    if (g_stub_hw_ret != 0) return g_stub_hw_ret;
    *out_raw = g_stub_hw_raw;
    return 0;
}
