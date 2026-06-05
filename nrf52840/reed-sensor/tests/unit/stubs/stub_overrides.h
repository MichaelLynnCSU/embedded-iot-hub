#ifndef STUB_OVERRIDES_H
#define STUB_OVERRIDES_H

#include <stdint.h>
#include <stdbool.h>

struct device;
extern bool           g_stub_device_ready;
extern struct device *g_stub_device_ptr;

void stub_set_device_ready(bool ready);

#ifdef DEVICE_DT_GET_ANY
#undef DEVICE_DT_GET_ANY
#endif
#define DEVICE_DT_GET_ANY(compat) (g_stub_device_ptr)

#endif /* STUB_OVERRIDES_H */
