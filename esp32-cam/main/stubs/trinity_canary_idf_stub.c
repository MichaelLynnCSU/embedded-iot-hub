#include "trinity_log.h"
#include "trinity_private_idf.h"
#include "esp_attr.h"
RTC_NOINIT_ATTR uint32_t g_pre_init_canary;
RTC_NOINIT_ATTR uint32_t g_canary_snapshot;
void trinity_canary_set_booted(void) {}
