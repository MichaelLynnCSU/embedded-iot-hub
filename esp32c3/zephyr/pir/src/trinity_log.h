/******************************************************************************
 * \file trinity_log.h
 * \brief Trinity crash logger public API -- ESP32-C3 Zephyr.
 *
 *          Typical boot sequence:
 *          1. trinity_log_dump_previous()  -- safe to call early (no WDT issue)
 *          2. trinity_log_init()           -- init flash + heap alloc
 *          3. trinity_log_event("EVENT: BOOT\n")
 *          4. trinity_wdt_init()           -- bench: no-op; field: arm WDT
 *          5. trinity_wdt_kick()
 *          6. trinity_log_event() as needed
 ******************************************************************************/

#ifndef INCLUDE_TRINITY_LOG_H_
#define INCLUDE_TRINITY_LOG_H_

#include <zephyr/kernel.h>

/**************************** CANARY VALUES ***********************************/

#define TRINITY_CANARY_ALIVE   0xCA11AB1EU
#define TRINITY_CANARY_BOOTED  0xB007ED00U

/**************************** INIT STAGE COOKIE *******************************/

#define TRINITY_STAGE_RESET         0x0000u
#define TRINITY_STAGE_GPIO          0x0101u
#define TRINITY_STAGE_BATTERY       0x0102u
#define TRINITY_STAGE_NVS_INIT      0x0201u
#define TRINITY_STAGE_NVS_LOAD      0x0202u
#define TRINITY_STAGE_LOG_INIT      0x0203u
#define TRINITY_STAGE_WDT_INIT      0x0301u
#define TRINITY_STAGE_BT_ENABLE     0x0302u
#define TRINITY_STAGE_BT_ADV        0x0303u
#define TRINITY_STAGE_MAIN_LOOP     0x0304u

extern volatile uint32_t g_init_stage;

/**************************** CORE FLASH LOG API ******************************/

int  trinity_log_init(void);
void trinity_log_dump_previous(void);
int  trinity_log_erase(void);
void trinity_log_event(const char *p_msg);
void trinity_log_event(const char *p_msg);
void trinity_log_flush(void);     /* drain deferred log before sleep or __noreturn paths */
/**************************** WATCHDOG ****************************************/

void trinity_wdt_init(void);
void trinity_wdt_kick(void);

/**************************** HEAP AND TASK STATS *****************************/

void trinity_log_heap_stats(void);
void trinity_log_task_stats(void);

/**************************** INTERNAL INIT (called by trinity_flash_esp.c) **/

void trinity_canary_set_booted(void);
int  trinity_log_stats_init(void);

/**************************** NVS PERSISTENCE *********************************/

int      trinity_nvs_init(void);
uint32_t trinity_nvs_read_motion_count(void);
int      trinity_nvs_write_motion_count(uint32_t val);

#endif /* INCLUDE_TRINITY_LOG_H_ */
