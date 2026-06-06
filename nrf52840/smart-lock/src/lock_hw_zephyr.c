/******************************************************************************
 * \file    lock_hw_zephyr.c
 * \brief   Zephyr GPIO binding for smart lock status LED.
 *
 * \details Implements lock_hw.h using Zephyr GPIO API.
 *          All Zephyr driver includes isolated here.
 *          main.c calls through lock_hw.h and has no GPIO dependency.
 ******************************************************************************/

#include "lock_hw.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(smartlock_main, LOG_LEVEL_DBG);

#define LIGHT_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec g_led = GPIO_DT_SPEC_GET(LIGHT_NODE, gpios);

/*----------------------------------------------------------------------------*/

int lock_hw_init(void)
{
    if (!device_is_ready(g_led.port))
    {
        LOG_ERR("LED not ready");
        return -ENODEV;
    }
    return gpio_pin_configure_dt(&g_led, GPIO_OUTPUT_INACTIVE);
}

/*----------------------------------------------------------------------------*/

void lock_hw_set_led(int state)
{
    gpio_pin_set_dt(&g_led, state);
}
