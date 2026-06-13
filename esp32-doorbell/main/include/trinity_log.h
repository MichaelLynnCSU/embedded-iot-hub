/******************************************************************************
 * \file trinity_log.h
 * \brief Trinity crash logger public API -- ESP-IDF (motor + hub).
 ******************************************************************************/
#ifndef TRINITY_LOG_H_
#define TRINITY_LOG_H_

#include <stdint.h>

/**************************** CANARY VALUES ***********************************/
#define TRINITY_CANARY_ALIVE   0xCA11AB1EU
#define TRINITY_CANARY_BOOTED  0xB007ED00U

/**************************** CORE NVS LOG API ********************************/
void trinity_log_init(void);
void trinity_log_dump_previous(void);
void trinity_log_erase(void);
void trinity_log_event(const char *p_msg);

/**************************** WATCHDOG ****************************************/
void trinity_wdt_init(void);
void trinity_wdt_add(void);
void trinity_wdt_kick(void);

/**************************** HEAP AND TASK STATS *****************************/
void trinity_log_heap_stats(void);
void trinity_log_task_stats(void);

/**************************** STACK MONITORING ********************************/
/**
 * \brief Persist a low-stack warning to the Trinity NVS fault log.
 *
 * Called automatically by trinity_wdt_kick() (via check_stack_hwm()) whenever
 * a monitored task's stack high-water-mark drops below
 * CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS.  The record is visible via
 * trinity_log_dump_previous() on the next boot, surviving any subsequent crash
 * before the overflow actually fires.
 *
 * The task name is captured internally via pcTaskGetName(NULL) so callers
 * cannot supply a stale or mismatched pointer.  Only the measured headroom
 * crosses the API boundary.
 *
 * NVS write rate: trinity_wdt_kick() gates calls to this function to once
 * per boot per task via a static flag.  Do not add additional call-site
 * gates; the deduplication lives in the caller, not here.
 *
 * \param hwm_words  Remaining stack headroom in WORDS (1 word = 4 bytes on
 *                   all ESP32 variants -- Xtensa and RISC-V alike).
 *                   Do NOT pass bytes; the logged record will be wrong.
 */
void trinity_log_record_low_stack(uint32_t hwm_words);

/**************************** BUILD INFO **************************************/
void trinity_build_info_print(void);

/**************************** INTERNAL (called by trinity_nvs_idf.c) **********/
void trinity_canary_set_booted(void);

#endif /* TRINITY_LOG_H_ */
