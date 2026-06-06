#ifndef STUB_EVENT_GROUPS_H
#define STUB_EVENT_GROUPS_H
/* Stub: freertos/event_groups.h */
#include "FreeRTOS.h"
#include <stddef.h>
typedef void * EventGroupHandle_t;
#define BIT0  (1 << 0)
static inline EventGroupHandle_t xEventGroupCreate(void) { return NULL; }
#endif /* STUB_EVENT_GROUPS_H */
