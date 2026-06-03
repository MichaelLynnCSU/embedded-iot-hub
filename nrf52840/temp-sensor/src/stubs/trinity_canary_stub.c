#include "trinity_log.h"
#include "trinity_private.h"
volatile uint32_t g_init_stage    __attribute__((section(".noinit")));
volatile uint32_t g_canary_snapshot = 0;
volatile uint32_t g_noinit_guard    = 0;
void trinity_canary_set_booted(void) {}
