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

/**************************** BUILD INFO **************************************/

void trinity_build_info_print(void);

/**************************** INTERNAL (called by trinity_nvs_idf.c) **********/

void trinity_canary_set_booted(void);

#endif /* TRINITY_LOG_H_ */
