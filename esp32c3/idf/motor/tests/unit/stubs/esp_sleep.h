#ifndef STUB_ESP_SLEEP_H
#define STUB_ESP_SLEEP_H
/* Stub: esp_sleep.h -- motor_control.c calls these before deep sleep */
#include <stdint.h>
static inline void esp_sleep_enable_timer_wakeup(uint64_t us) { (void)us; }
static inline void esp_deep_sleep_start(void) { /* stub -- noreturn in production */ }
#endif /* STUB_ESP_SLEEP_H */
