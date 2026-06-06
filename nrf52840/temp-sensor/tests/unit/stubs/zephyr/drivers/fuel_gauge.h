#ifndef STUB_FUEL_GAUGE_H
#define STUB_FUEL_GAUGE_H

#include <stdint.h>
#include <zephyr/device.h>

enum fuel_gauge_property {
    FUEL_GAUGE_VOLTAGE = 0,
    FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE,
};

union fuel_gauge_prop_val {
    int      voltage;
    uint8_t  relative_state_of_charge;
    uint32_t raw;
};

int fuel_gauge_get_prop(const struct device *dev,
                        enum fuel_gauge_property prop,
                        union fuel_gauge_prop_val *val);

extern struct device *g_stub_device_ptr;
#undef  DEVICE_DT_GET_ANY
#define DEVICE_DT_GET_ANY(compat) (g_stub_device_ptr)

#endif /* STUB_FUEL_GAUGE_H */
