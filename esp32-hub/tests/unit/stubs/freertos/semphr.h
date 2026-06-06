#ifndef STUB_FREERTOS_SEMPHR_H
#define STUB_FREERTOS_SEMPHR_H
#include "FreeRTOS.h"
#include <stddef.h>
typedef void *  SemaphoreHandle_t;
typedef uint8_t StaticSemaphore_t;
#define portMAX_DELAY   0xFFFFFFFF
#define configASSERT(x) (void)(x)
static inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *b)
{ (void)b; return (void*)1; }
static inline int xSemaphoreTake(SemaphoreHandle_t s, uint32_t t)
{ (void)s;(void)t; return 1; }
static inline int xSemaphoreGive(SemaphoreHandle_t s)
{ (void)s; return 1; }
#endif
