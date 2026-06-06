/******************************************************************************
 * \file    reed_hw_zephyr.c
 * \brief   Zephyr GPIO binding for reed switch and debug LED.
 *
 * \details Implements reed_hw.h using Zephyr GPIO API.
 *          All Zephyr driver includes are isolated here.
 *          main.c calls through reed_hw.h and has no GPIO dependency.
 ******************************************************************************/

#include "reed_hw.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(reed_main, LOG_LEVEL_INF);

#define LED_NODE  DT_ALIAS(led0)
#define REED_PORT DT_NODELABEL(gpio0)
#define REED_PIN  11

static const struct gpio_dt_spec g_led =
    GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct device *g_gpio0;

/*----------------------------------------------------------------------------*/

int reed_hw_init(void)
{
    int err = 0;

    g_gpio0 = DEVICE_DT_GET(REED_PORT);

    if (!device_is_ready(g_gpio0))    { LOG_ERR("GPIO0 not ready");  return -ENODEV; }
    if (!device_is_ready(g_led.port)) { LOG_ERR("LED not ready");    return -ENODEV; }

    err = gpio_pin_configure_dt(&g_led, GPIO_OUTPUT_INACTIVE);
    if (0 != err) { LOG_ERR("LED configure failed (err=%d)", err); return err; }

    err = gpio_pin_configure(g_gpio0, REED_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (0 != err) { LOG_ERR("Reed pin configure failed (err=%d)", err); return err; }

    return 0;
}

/*----------------------------------------------------------------------------*/

int reed_hw_read(void)
{
    return gpio_pin_get(g_gpio0, REED_PIN);
}

/*----------------------------------------------------------------------------*/

void reed_hw_set_led(int state)
{
    int err = gpio_pin_set_dt(&g_led, state);
    if (0 != err) { LOG_WRN("LED set failed (err=%d)", err); }
}
