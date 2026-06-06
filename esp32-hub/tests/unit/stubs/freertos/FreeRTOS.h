#ifndef STUB_FREERTOS_H
#define STUB_FREERTOS_H
#include <stdint.h>
#define pdMS_TO_TICKS(ms)   (ms)
#define portTICK_PERIOD_MS  1
typedef uint32_t TickType_t;
static inline uint32_t xTaskGetTickCount(void) { return 0; }
#endif
