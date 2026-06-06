#ifndef STUB_FREERTOS_H
#define STUB_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
#define portTICK_PERIOD_MS  1
#define pdMS_TO_TICKS(ms)   (ms)
#define pdFALSE             0
#define pdTRUE              1
#define BIT0                (1 << 0)
#define WIFI_CONNECTED_BIT  BIT0
typedef uint32_t EventBits_t;
typedef void *   EventGroupHandle_t;
extern EventGroupHandle_t g_wifi_eg;
static inline EventBits_t xEventGroupWaitBits(EventGroupHandle_t eg,
    EventBits_t bits, int clear, int all, uint32_t ticks)
{ (void)eg;(void)bits;(void)clear;(void)all;(void)ticks; return WIFI_CONNECTED_BIT; }
static inline uint32_t xTaskGetTickCount(void) { return 0; }
#endif
