#ifndef STUB_FREERTOS_EVENT_GROUPS_H
#define STUB_FREERTOS_EVENT_GROUPS_H
#include "FreeRTOS.h"
#include <stddef.h>
typedef void * EventGroupHandle_t;
typedef uint32_t EventBits_t;
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
static inline EventGroupHandle_t xEventGroupCreate(void) { return NULL; }
static inline EventBits_t xEventGroupWaitBits(EventGroupHandle_t eg,
    EventBits_t bits, int clear, int all, uint32_t ticks)
{ (void)eg;(void)bits;(void)clear;(void)all;(void)ticks; return bits; }
#endif
