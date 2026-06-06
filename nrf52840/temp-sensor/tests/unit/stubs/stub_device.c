#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

bool g_stub_device_ready = true;

static struct device_state g_state_ready    = { .initialized = 1, .init_res = 0 };
static struct device_state g_state_notready = { .initialized = 0, .init_res = 1 };

static struct device g_stub_device_impl = {
    .name  = "stub_adc",
    .state = &g_state_ready,
};

struct device *g_stub_device_ptr = &g_stub_device_impl;

void stub_set_device_ready(bool ready)
{
    g_stub_device_ready = ready;
    g_stub_device_impl.state = ready ? &g_state_ready : &g_state_notready;
}
