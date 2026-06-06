#ifndef STUB_FREERTOS_H
#define STUB_FREERTOS_H
#include <stdint.h>
#define pdMS_TO_TICKS(ms)   (ms)
#define portMAX_DELAY       0xFFFFFFFFUL
#define portTICK_PERIOD_MS  1

typedef uint32_t TickType_t;

/* Controlled by stubs.c -- tests write this to inject time */
extern uint32_t g_stub_tick_ms;

static inline uint32_t xTaskGetTickCount(void) { return g_stub_tick_ms; }

#endif /* STUB_FREERTOS_H */
#include "portmacro.h"
