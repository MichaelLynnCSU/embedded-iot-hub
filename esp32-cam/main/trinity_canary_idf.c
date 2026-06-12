/******************************************************************************
 * \file trinity_canary_idf.c
 * \brief Trinity pre-init canary -- ESP-IDF (motor + hub).
 *
 * \details Fully standalone. Owns RTC_NOINIT_ATTR canary variables.
 *          __attribute__((constructor)) runs before app_main and before
 *          any component init that could crash.
 *
 *          RTC_NOINIT_ATTR survives warm resets (software reset, WDT reset)
 *          but is cleared on cold power-on -- correctly reported as cold
 *          boot by trinity_log_dump_previous().
 ******************************************************************************/

#include "trinity_log.h"
#include "trinity_private_idf.h"
#include "esp_attr.h"

RTC_NOINIT_ATTR uint32_t g_pre_init_canary;
RTC_NOINIT_ATTR uint32_t g_canary_snapshot;

static void __attribute__((constructor)) canary_constructor(void)
{
    g_canary_snapshot = g_pre_init_canary;
    g_pre_init_canary = TRINITY_CANARY_ALIVE;
}

void trinity_canary_set_booted(void)
{
    g_pre_init_canary = TRINITY_CANARY_BOOTED;
}
