#ifndef STUB_DRIVER_GPIO_H
#define STUB_DRIVER_GPIO_H
#include <stdint.h>
typedef int gpio_num_t;
typedef int gpio_mode_t;
#define GPIO_NUM_2      2
#define GPIO_NUM_3      3
#define GPIO_NUM_4      4
#define GPIO_MODE_OUTPUT 1
#define ESP_OK          0
typedef int esp_err_t;
static inline esp_err_t gpio_set_direction(gpio_num_t p, gpio_mode_t m) { (void)p;(void)m; return ESP_OK; }
static inline esp_err_t gpio_set_level(gpio_num_t p, uint32_t v) { (void)p;(void)v; return ESP_OK; }
#endif
