#ifndef STUB_GPIO_H
#define STUB_GPIO_H

/*
 * Stub: zephyr/drivers/gpio.h
 * light_hw_zephyr.c uses gpio_pin_configure(), gpio_pin_configure_dt(),
 * gpio_pin_set(), gpio_pin_set_dt(), device_is_ready(), GPIO_OUTPUT_INACTIVE,
 * GPIO_DT_SPEC_GET, and gpio_dt_spec.
 *
 * light_hw_zephyr.c is NOT compiled into the unit test build -- light_hw_stub.c
 * replaces it entirely. This header exists only so any transitive include of
 * zephyr/drivers/gpio.h resolves without pulling in real Zephyr drivers.
 */

#include <stdint.h>
#include <stdbool.h>

struct device { int dummy; };

struct gpio_dt_spec {
    const struct device *port;
    uint32_t             pin;
    uint32_t             dt_flags;
};

#define GPIO_OUTPUT_INACTIVE  0x01
#define GPIO_OUTPUT_ACTIVE    0x02
#define GPIO_INPUT            0x04

#define GPIO_DT_SPEC_GET(node, prop)  { NULL, 0, 0 }

static inline bool device_is_ready(const struct device *dev)
{ (void)dev; return true; }

static inline int gpio_pin_configure(const struct device *dev,
                                      uint32_t pin, uint32_t flags)
{ (void)dev;(void)pin;(void)flags; return 0; }

static inline int gpio_pin_configure_dt(const struct gpio_dt_spec *s,
                                         uint32_t flags)
{ (void)s;(void)flags; return 0; }

static inline int gpio_pin_set(const struct device *dev,
                                uint32_t pin, int val)
{ (void)dev;(void)pin;(void)val; return 0; }

static inline int gpio_pin_set_dt(const struct gpio_dt_spec *s, int val)
{ (void)s;(void)val; return 0; }

#endif /* STUB_GPIO_H */
