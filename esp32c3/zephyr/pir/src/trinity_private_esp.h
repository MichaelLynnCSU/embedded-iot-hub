/******************************************************************************
 * \file trinity_private_esp.h
 * \brief Trinity internal interface -- ESP32-C3 Zephyr. Not for app code.
 *
 * \details Exposes symbols shared across Trinity's split .c files.
 *
 *          Dependency direction (strict -- never invert):
 *            wdt    -- standalone
 *            canary -- standalone
 *            flash  -- depends on canary (noinit vars)
 *            stats  -- depends on flash via trinity_log_event only
 *            fault  -- depends on flash via write_panic + canary g_init_stage
 *
 *          No flash mutex on ESP32-C3 Zephyr -- flash driver serializes
 *          internally. write_entry() calls write_internal() directly.
 ******************************************************************************/

#ifndef TRINITY_PRIVATE_ESP_H_
#define TRINITY_PRIVATE_ESP_H_

#include <stdint.h>

/************************** CANARY GLOBALS (owned by trinity_canary_esp.c) ****/

extern volatile uint32_t g_init_stage;
extern volatile uint32_t g_canary_snapshot;
extern volatile uint32_t g_noinit_guard;

/************************** FLASH INTERNALS (owned by trinity_flash_esp.c) ****/

/**
 * \brief  Lock-free flash write -- fault handler ONLY.
 *
 * \details No mutex on ESP32-C3 -- flash driver handles serialization.
 *          Validates TRINITY_INIT_MAGIC before touching flash.
 *
 * \warning Only call from k_sys_fatal_error_handler.
 */
void write_panic(const char *p_msg, uint16_t len);

#endif /* TRINITY_PRIVATE_ESP_H_ */
