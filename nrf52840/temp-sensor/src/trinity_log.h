/******************************************************************************
 * \file trinity_log.h
 * \brief Trinity crash logger public API -- nRF52840 Zephyr.
 *
 *          Typical boot sequence (field mode):
 *          1. Read + clear NRF_POWER->RESETREAS into local var
 *          2. trinity_log_dump_previous()       -- bench only
 *          3. g_init_stage = TRINITY_STAGE_*    -- before each risky call
 *          4. trinity_log_init()                -- init flash + heap alloc
 *          5. trinity_wdt_init()                -- arm watchdog
 *          6. trinity_wdt_kick()                -- feed before dump
 *          7. trinity_log_dump_previous_deferred() -- field/USB only
 *          8. trinity_log_boot_reason(reset_reason)
 *          9. trinity_log_event() as needed
 *
 *          Flash mutex:
 *            trinity_flash_lock() / trinity_flash_unlock() must wrap any
 *            flash write outside Trinity (e.g. settings_save_one()).
 *            trinity_log_event() acquires mutex internally.
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

/* Defined in trinity_canary.c -- write from main() before each risky call */
extern volatile uint32_t g_init_stage;

/**************************** CORE FLASH LOG API ******************************/

int  trinity_log_init(void);
void trinity_log_dump_previous(void);
void trinity_log_dump_previous_deferred(void);
int  trinity_log_erase(void);
void trinity_log_event(const char *p_msg);

/**************************** BOOT REASON *************************************/

/**
 * \brief  Log nRF52840 reset reason. Call after trinity_log_init().
 * \param  resetreas  Value read from NRF_POWER->RESETREAS at top of main()
 *                    before it was cleared.
 */
void trinity_log_boot_reason(uint32_t resetreas);

/**************************** FLASH MUTEX *************************************/

void trinity_flash_lock(void);
void trinity_flash_unlock(void);

/**************************** WATCHDOG ****************************************/

void trinity_wdt_init(void);
void trinity_wdt_kick(void);

/**************************** HEAP AND TASK STATS *****************************/

void trinity_log_heap_stats(void);
void trinity_log_task_stats(void);

/**************************** INTERNAL INIT (called by trinity_flash.c) ******/

/* Called by trinity_log_init() -- do not call from application code */
void trinity_canary_set_booted(void);
int  trinity_log_stats_init(void);

/**************************** LEGACY ALIASES **********************************/

#define flash_log_init          trinity_log_init
#define flash_log_dump_previous trinity_log_dump_previous
#define flash_log_erase         trinity_log_erase
#define flash_log_event         trinity_log_event

/**************************** RESET REASON CLASSIFIER ************************/

typedef enum {
    TRINITY_BOOT_COLD_POWER_ON = 0,
    TRINITY_BOOT_RESET_PIN,
    TRINITY_BOOT_WATCHDOG,
    TRINITY_BOOT_SOFT_RESET,
    TRINITY_BOOT_BROWNOUT,
    TRINITY_BOOT_UNKNOWN,
} TRINITY_BOOT_REASON_E;

extern volatile uint32_t g_noinit_guard;
extern volatile uint32_t g_canary_snapshot;

TRINITY_BOOT_REASON_E trinity_classify_reset(uint32_t resetreas);

#endif /* INCLUDE_TRINITY_LOG_H_ */
