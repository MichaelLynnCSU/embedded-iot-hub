/******************************************************************************
 * \file    light_hw_zephyr.c
 * \brief   Zephyr GPIO binding for smart light relay and status LED.
 *
 * \details Implements light_hw.h using Zephyr GPIO API.
 *          All Zephyr driver includes isolated here.
 *          main.c calls through light_hw.h and has no GPIO dependency.
 *          relay_set() in main.c is a thin wrapper over light_hw_set().
 ******************************************************************************/

#include "light_hw.h"
#include "main.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(smartlight_main, LOG_LEVEL_DBG);

#define RELAY_PORT  DT_NODELABEL(gpio0)
#define LIGHT_NODE  DT_ALIAS(led0)

static const struct gpio_dt_spec g_light     = GPIO_DT_SPEC_GET(LIGHT_NODE, gpios);
static const struct device       *g_relay_dev = NULL;

/*----------------------------------------------------------------------------*/

int light_hw_init(void)
{
    g_relay_dev = DEVICE_DT_GET(RELAY_PORT);

    if (!device_is_ready(g_relay_dev))
    {
        printk("[GPIO] Relay device not ready\n");
        return -ENODEV;
    }

    if (gpio_pin_configure(g_relay_dev, RELAY_PIN, GPIO_OUTPUT_INACTIVE))
    {
        printk("[GPIO] Relay pin configure failed\n");
        return -EIO;
    }

    if (!device_is_ready(g_light.port))
    {
        printk("[GPIO] Light LED not ready\n");
        return -ENODEV;
    }

    if (gpio_pin_configure_dt(&g_light, GPIO_OUTPUT_INACTIVE))
    {
        printk("[GPIO] Light LED configure failed\n");
        return -EIO;
    }

    printk("[GPIO] Light/Relay initialized (OFF)\n");
    return 0;
}

/*----------------------------------------------------------------------------*/

void light_hw_set(uint8_t state)
{
    gpio_pin_set(g_relay_dev, RELAY_PIN, state);
    gpio_pin_set_dt(&g_light, state);
}
