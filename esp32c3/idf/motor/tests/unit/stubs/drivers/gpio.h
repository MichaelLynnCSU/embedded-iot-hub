#ifndef STUB_DRIVER_GPIO_H
#define STUB_DRIVER_GPIO_H
/* Stub: driver/gpio.h -- motor_control.c uses gpio_set_direction/level */
#include <stdint.h>

typedef int gpio_num_t;
typedef int gpio_mode_t;

#define GPIO_NUM_2  2
#define GPIO_NUM_3  3
#define GPIO_NUM_4  4
#define GPIO_MODE_OUTPUT 1
#define ESP_OK  0

typedef int esp_err_t;

static inline esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode)
{ (void)pin; (void)mode; return ESP_OK; }

static inline esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level)
{ (void)pin; (void)level; return ESP_OK; }

#endif /* STUB_DRIVER_GPIO_H */
