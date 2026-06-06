#ifndef STUB_FREERTOS_QUEUE_H
#define STUB_FREERTOS_QUEUE_H

#include <stdint.h>
#include <stddef.h>

#define pdTRUE  1
#define pdFALSE 0

typedef void *QueueHandle_t;

static inline QueueHandle_t xQueueCreate(uint32_t len, uint32_t size)
{ (void)len;(void)size; return NULL; }
static inline int xQueueOverwrite(QueueHandle_t q, const void *p)
{ (void)q;(void)p; return pdTRUE; }
static inline int xQueueReceive(QueueHandle_t q, void *p, uint32_t t)
{ (void)q;(void)p;(void)t; return pdFALSE; }
static inline int xQueueSend(QueueHandle_t q, const void *p, uint32_t t)
{ (void)q;(void)p;(void)t; return pdTRUE; }

#endif /* STUB_FREERTOS_QUEUE_H */
