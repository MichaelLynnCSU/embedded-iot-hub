#ifndef STUB_DRIVER_LEDC_H
#define STUB_DRIVER_LEDC_H
/* Stub: driver/ledc.h -- motor_control.c uses LEDC for PWM */
#include <stdint.h>

#ifndef ESP_OK
#define ESP_OK 0
typedef int esp_err_t;
#endif

typedef int ledc_timer_t;
typedef int ledc_channel_t;
typedef int ledc_mode_t;
typedef int ledc_timer_bit_t;
typedef int ledc_clk_cfg_t;

#define LEDC_TIMER_0          0
#define LEDC_CHANNEL_0        0
#define LEDC_LOW_SPEED_MODE   0
#define LEDC_TIMER_13_BIT     13
#define LEDC_AUTO_CLK         0

typedef struct {
    ledc_mode_t      speed_mode;
    ledc_timer_t     timer_num;
    ledc_timer_bit_t duty_resolution;
    uint32_t         freq_hz;
    ledc_clk_cfg_t   clk_cfg;
} ledc_timer_config_t;

typedef struct {
    ledc_mode_t    speed_mode;
    ledc_channel_t channel;
    ledc_timer_t   timer_sel;
    int            gpio_num;
    uint32_t       duty;
} ledc_channel_config_t;

static inline esp_err_t ledc_timer_config(const ledc_timer_config_t *c)
{ (void)c; return ESP_OK; }
static inline esp_err_t ledc_channel_config(const ledc_channel_config_t *c)
{ (void)c; return ESP_OK; }
static inline esp_err_t ledc_set_duty(ledc_mode_t m, ledc_channel_t ch, uint32_t d)
{ (void)m;(void)ch;(void)d; return ESP_OK; }
static inline esp_err_t ledc_update_duty(ledc_mode_t m, ledc_channel_t ch)
{ (void)m;(void)ch; return ESP_OK; }

#endif /* STUB_DRIVER_LEDC_H */
