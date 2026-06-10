/******************************************************************************
 * \file trinity_private_idf.h
 * \brief Trinity internal interface -- ESP-IDF (motor + hub). Not for app code.
 *
 * \details Dependency direction (strict -- never invert):
 *            wdt    -- standalone
 *            canary -- standalone
 *            nvs    -- depends on canary (RTC vars)
 *            stats  -- depends on nvs via trinity_log_event only
 *            panic  -- depends on nvs via trinity_log_event only
 *
 * \note    No write_panic exposure needed on IDF -- the panic handler calls
 *          trinity_log_event() directly. IDF doesn't have the Zephyr
 *          mutex-in-fault-handler deadlock risk in the same way.
 ******************************************************************************/

#ifndef TRINITY_PRIVATE_IDF_H_
#define TRINITY_PRIVATE_IDF_H_

#include <stdint.h>

/************************** CANARY GLOBALS (owned by trinity_canary_idf.c) ****/

/* RTC_NOINIT_ATTR -- survives warm resets, cleared on cold power-on */
extern uint32_t g_pre_init_canary;
extern uint32_t g_canary_snapshot;

/**
 * \brief  Mark Trinity init complete. Called by trinity_log_init().
 */
void trinity_canary_set_booted(void);

#endif /* TRINITY_PRIVATE_IDF_H_ */
