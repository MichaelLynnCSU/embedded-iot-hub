#ifndef STUB_FREERTOS_TASK_H
#define STUB_FREERTOS_TASK_H
#include "FreeRTOS.h"
static inline void vTaskDelay(uint32_t t) { (void)t; }
#endif
