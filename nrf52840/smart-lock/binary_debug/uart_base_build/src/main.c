#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* Use the onboard LED alias from the Devicetree */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void) {
    gpio_is_ready_dt(&led);
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    while (1) {
        printk("alive\n");
        gpio_pin_toggle_dt(&led);
        k_sleep(K_SECONDS(1));
    }
}
