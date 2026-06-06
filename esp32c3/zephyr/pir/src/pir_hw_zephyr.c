/******************************************************************************
 * \file    pir_hw_zephyr.c
 * \brief   Zephyr GPIO binding for AM312 PIR sensor.
 *
 * \details Implements pir_hw.h using Zephyr GPIO API.
 *          All Zephyr GPIO includes isolated here.
 *          main.c calls through pir_hw.h and has no GPIO dependency.
 ******************************************************************************/

#include "pir_hw.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(pir_main, LOG_LEVEL_INF);

#define PIR_PIN   3
#define GPIO_NODE DT_NODELABEL(gpio0)

static const struct device *g_gpio;

/*----------------------------------------------------------------------------*/

int pir_hw_init(void)
{
    g_gpio = DEVICE_DT_GET(GPIO_NODE);
    if (!device_is_ready(g_gpio))
    {
        LOG_ERR("GPIO not ready");
        return -ENODEV;
    }
    return gpio_pin_configure(g_gpio, PIR_PIN, GPIO_INPUT);
}

/*----------------------------------------------------------------------------*/

int pir_hw_read(void)
{
    if (!g_gpio) return -ENODEV;
    return gpio_pin_get(g_gpio, PIR_PIN);
}
