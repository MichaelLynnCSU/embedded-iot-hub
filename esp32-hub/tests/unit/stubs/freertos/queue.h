#ifndef STUB_FREERTOS_QUEUE_H
#define STUB_FREERTOS_QUEUE_H
#include <stdint.h>
#include <stddef.h>
typedef void * QueueHandle_t;
static inline QueueHandle_t xQueueCreate(uint32_t len, uint32_t size)
{ (void)len;(void)size; return NULL; }
static inline int xQueueOverwrite(QueueHandle_t q, const void *p)
{ (void)q;(void)p; return 1; }
static inline int xQueueReceive(QueueHandle_t q, void *p, uint32_t t)
{ (void)q;(void)p;(void)t; return 0; }
#endif
