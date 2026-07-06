/******************************************************************************
 * \file    trinity_log.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief   Unified Trinity crash and event logger interface -- all boards.
 *
 * \details Trinity is a portable crash-survive logging system covering
 *          10 boards across 5 real API families. One header, lives at
 *          repo root (mirrors beaglebone/include/ pattern), included via
 *          each board's build-system include path -- no per-board copies.
 *
 *          Select platform at build time via exactly ONE chip flag:
 *
 *          Zephyr (prj.conf or west build -D):
 *            CONFIG_TRINITY_CHIP_NRF52840=y     -- reed-sensor, smart-lock,
 *                                                   smart-light, temp-sensor
 *            CONFIG_TRINITY_CHIP_ESP32C3=y      -- pir (ESP32-C3 Zephyr)
 *
 *          IDF (CMakeLists.txt target_compile_definitions):
 *            TRINITY_CHIP_ESP32C3_IDF=1         -- motor (ESP32-C3 IDF)
 *            TRINITY_CHIP_ESP32_HUB_IDF=1       -- hub
 *            TRINITY_CHIP_ESP32_CAM_IDF=1       -- cam
 *            TRINITY_CHIP_ESP32_DOORBELL_IDF=1  -- doorbell
 *
 *          STM32 HAL (CMakeLists.txt target_compile_definitions):
 *            TRINITY_CHIP_STM32_F4=1            -- stm32-blackpill (bare metal)
 *            TRINITY_CHIP_STM32_F1=1            -- stm32-bluepill (FreeRTOS)
 *
 * \note    STORAGE BACKEND BY BOARD (corrected -- these are NOT identical):
 *            nRF52840   -- Zephyr flash_area circular log (raw NVMC)
 *            ESP32 (all)-- NVS + coredump
 *            STM32 F411 -- RTC backup registers (BKP0-3). NOT FRAM, despite
 *                          legacy comments/coverage matrices calling it FRAM.
 *            STM32 F103 -- real FM24CL16B FRAM chip over I2C, partitioned
 *                          (~16KB crash region). This is the only board that
 *                          actually uses FRAM for the crash record.
 *
 * \warning ORPHANED HEADER NOTICE: nrf52840/smart-lock/binary_debug/
 *          {rtt,usbc,com}_base_build/src/trinity_log.h previously declared
 *          the STM32 API (trinity_uart_log/rtc_store/hard_fault_handler)
 *          inside an nRF52840 Zephyr folder. Confirmed dead -- the adjacent
 *          trinity_log_zephyr_nrf.c never calls any of those functions.
 *          Those copies should be deleted, not migrated, when adopting
 *          this header.
 ******************************************************************************/

#ifndef INCLUDE_TRINITY_LOG_H_
#define INCLUDE_TRINITY_LOG_H_

/******************************************************************************
 * Platform detection guard -- exactly one flag must be set.
 *****************************************************************************/
#if defined(CONFIG_TRINITY_CHIP_NRF52840)      || \
    defined(CONFIG_TRINITY_CHIP_ESP32C3)       || \
    defined(TRINITY_CHIP_ESP32C3_IDF)          || \
    defined(TRINITY_CHIP_ESP32_HUB_IDF)        || \
    defined(TRINITY_CHIP_ESP32_CAM_IDF)        || \
    defined(TRINITY_CHIP_ESP32_DOORBELL_IDF)   || \
    defined(TRINITY_CHIP_STM32_F4)             || \
    defined(TRINITY_CHIP_STM32_F1)
   /* At least one chip flag is set -- OK */
#else
   #error "Trinity: no chip flag defined. Set exactly one TRINITY_CHIP_* flag."
#endif

/******************************************************************************
 * Platform-specific includes required by the public API types below.
 *****************************************************************************/
#if defined(CONFIG_TRINITY_CHIP_NRF52840) || defined(CONFIG_TRINITY_CHIP_ESP32C3)
   #include <zephyr/kernel.h>
#elif defined(TRINITY_CHIP_STM32_F4) || defined(TRINITY_CHIP_STM32_F1)
   #include <stdint.h>
   #include <stddef.h>
#else
   #include <stdint.h>
#endif

/******************************************************************************
 * CANARY VALUES -- identical across all platforms.
 *****************************************************************************/
#define TRINITY_CANARY_ALIVE   0xCA11AB1EU  /**< Crashed before log_init     */
#define TRINITY_CANARY_BOOTED  0xB007ED00U  /**< log_init completed OK      */

/******************************************************************************
 * INIT STAGE COOKIE -- Zephyr platforms only (nRF52840 + ESP32-C3 Zephyr).
 *****************************************************************************/
#if defined(CONFIG_TRINITY_CHIP_NRF52840) || defined(CONFIG_TRINITY_CHIP_ESP32C3)

#define TRINITY_STAGE_RESET         0x0000u
#define TRINITY_STAGE_GPIO          0x0101u
#define TRINITY_STAGE_BATTERY       0x0102u
#define TRINITY_STAGE_NVS_INIT      0x0201u
#define TRINITY_STAGE_NVS_LOAD      0x0202u
#define TRINITY_STAGE_LOG_INIT      0x0203u
#define TRINITY_STAGE_WDT_INIT      0x0301u
#define TRINITY_STAGE_BT_ENABLE     0x0302u
#define TRINITY_STAGE_BT_ADV        0x0303u
#define TRINITY_STAGE_WIFI_INIT     0x0305u
#define TRINITY_STAGE_MAIN_LOOP     0x0304u

extern volatile uint32_t g_init_stage;

#endif /* Zephyr platforms */

/******************************************************************************
 * COMMON API -- every board implements these.
 *****************************************************************************/
void trinity_log_dump_previous(void);

#if defined(CONFIG_TRINITY_CHIP_NRF52840) || defined(CONFIG_TRINITY_CHIP_ESP32C3)
int  trinity_log_init(void);
int  trinity_log_erase(void);
#else
void trinity_log_init(void);
void trinity_log_erase(void);
#endif

void trinity_log_event(const char *p_msg);
void trinity_wdt_init(void);
void trinity_wdt_kick(void);
void trinity_log_heap_stats(void);
void trinity_log_task_stats(void);

/* Internal init, called by trinity_flash*.c / trinity_nvs*.c -- not for
 * application code, but declared here since multiple .c files share it. */
void trinity_canary_set_booted(void);

/******************************************************************************
 * nRF52840 ZEPHYR ADDITIONS
 *****************************************************************************/
#if defined(CONFIG_TRINITY_CHIP_NRF52840)

typedef enum
{
   TRINITY_BOOT_COLD_POWER_ON = 0,
   TRINITY_BOOT_RESET_PIN,
   TRINITY_BOOT_WATCHDOG,
   TRINITY_BOOT_SOFT_RESET,
   TRINITY_BOOT_BROWNOUT,
   TRINITY_BOOT_UNKNOWN,
} TRINITY_BOOT_REASON_E;

void trinity_log_boot_reason(uint32_t resetreas);
TRINITY_BOOT_REASON_E trinity_classify_reset(uint32_t resetreas);

void trinity_flash_lock(void);
void trinity_flash_unlock(void);

/* Field/USB mode only -- deferred until after WDT init (see trinity_flash.c) */
void trinity_log_dump_previous_deferred(void);

int  trinity_log_stats_init(void);

extern volatile uint32_t g_noinit_guard;
extern volatile uint32_t g_canary_snapshot;

#endif /* CONFIG_TRINITY_CHIP_NRF52840 */

/******************************************************************************
 * ESP32-C3 ZEPHYR ADDITIONS (pir only -- distinct from ESP32 IDF family)
 *****************************************************************************/
#if defined(CONFIG_TRINITY_CHIP_ESP32C3)

void trinity_log_flush(void);   /* drain deferred log before sleep/__noreturn */

int      trinity_nvs_init(void);
uint32_t trinity_nvs_read_motion_count(void);
int      trinity_nvs_write_motion_count(uint32_t val);

#endif /* CONFIG_TRINITY_CHIP_ESP32C3 */

/******************************************************************************
 * ESP32 IDF ADDITIONS (cam, doorbell, hub, esp32c3/motor)
 *****************************************************************************/
#if defined(TRINITY_CHIP_ESP32C3_IDF)     || \
    defined(TRINITY_CHIP_ESP32_HUB_IDF)   || \
    defined(TRINITY_CHIP_ESP32_CAM_IDF)   || \
    defined(TRINITY_CHIP_ESP32_DOORBELL_IDF)

void trinity_wdt_add(void);
void trinity_build_info_print(void);

/**
 * \brief Persist a low-stack warning to the Trinity NVS fault log.
 * \details Called automatically by trinity_wdt_kick() (via check_stack_hwm())
 *          when a monitored task's headroom drops below
 *          CONFIG_TRINITY_STACK_LOW_WATERMARK_WORDS. Task name captured
 *          internally via pcTaskGetName(NULL).
 * \param hwm_words Remaining stack headroom in WORDS (4 bytes/word), not bytes.
 */
void trinity_log_record_low_stack(uint32_t hwm_words);

#endif /* ESP32 IDF platforms (cam/doorbell/hub/motor) */

/******************************************************************************
 * STM32 ADDITIONS (blackpill F411 + bluepill F103)
 *****************************************************************************/
#if defined(TRINITY_CHIP_STM32_F4) || defined(TRINITY_CHIP_STM32_F1)

#define TRINITY_MSG_LEN   80u  /**< Max panic/log message length incl NUL */

/**
 * \brief Error codes stored in the crash record.
 * \details F411: written to RTC backup registers. F103: written to FRAM.
 *          Read back by trinity_log_init() on the next boot.
 */
typedef enum
{
   eTRINITY_ERR_NONE        = 0x00u,
   eTRINITY_ERR_HARDFAULT   = 0x01u,
   eTRINITY_ERR_STACK       = 0x02u,
   eTRINITY_ERR_HEAP        = 0x03u,
   eTRINITY_ERR_ASSERT      = 0x04u,
   eTRINITY_ERR_BROWNOUT    = 0x05u,
   eTRINITY_ERR_WATCHDOG    = 0x06u,
   eTRINITY_ERR_PANIC       = 0x07u,
   eTRINITY_ERR_UNKNOWN     = 0xFFu,
} TRINITY_ERROR_E;

/** Polled, blocking UART write. Safe to call from fault handlers. */
void trinity_uart_log(const char *p_msg);

/**
 * \brief Store an error code to the crash record (survives power loss).
 * \details F411 (RTC backup registers): trinity_rtc_store(err) -- one arg.
 *          F103 (real FRAM chip):        trinity_rtc_store(err, boot_count).
 *          NOTE: function name is legacy -- only F103 is actually FRAM.
 */
#if defined(TRINITY_CHIP_STM32_F4)
void trinity_rtc_store(TRINITY_ERROR_E err);
#else
void trinity_rtc_store(TRINITY_ERROR_E err, uint8_t boot_count);
#endif

void panic_handler(const char *p_reason, TRINITY_ERROR_E err);
void trinity_hard_fault_handler(void);

void *trinity_malloc(size_t size);
void  trinity_free(void **pp_ptr);

/** Watchdog aliases -- STM32 internal naming */
#define trinity_watchdog_init  trinity_wdt_init
#define trinity_watchdog_kick  trinity_wdt_kick

void trinity_build_info_print(void);

#endif /* STM32 platforms */

/******************************************************************************
 * STM32F4 ONLY ADDITIONS (blackpill bare metal, no RTOS)
 *****************************************************************************/
#if defined(TRINITY_CHIP_STM32_F4)

#define STACK_CANARY   0xDEADBEEFul

void trinity_paint_stack(void);
void trinity_check_stack(void);

#endif /* TRINITY_CHIP_STM32_F4 */

/******************************************************************************
 * LEGACY ALIASES -- backward compatibility for existing callers.
 *****************************************************************************/
#define flash_log_init          trinity_log_init
#define flash_log_dump_previous trinity_log_dump_previous
#define flash_log_erase         trinity_log_erase
#define flash_log_event         trinity_log_event
#define crash_log_init          trinity_log_init
#define crash_log_event         trinity_log_event
#define crash_log_dump_previous trinity_log_dump_previous
#define crash_log_erase         trinity_log_erase

#endif /* INCLUDE_TRINITY_LOG_H_ */

/*******************************************************************************
 * TRINITY COVERAGE MATRIX (corrected -- FRAM row split from RTC row)
 *
 * Fault type          nRF52840  ESP32-C3Z  ESP32-C3I  ESP32(all)  F411  F103
 * ─────────────────── ────────  ─────────  ─────────  ──────────  ────  ────
 * HardFault / panic      ✓          ✓          ✓          ✓         ✓     ✓
 * WDT reset              ✓          ✓          ✓          ✓         ✓     ✓
 * Brownout               ✓          ✓          ✓          ✓         ✓     ✓
 * Stack overflow         ✓(MPU)     ✓(MPU)     ✓(RTOS)    ✓         ✓     ✓
 * Heap exhaustion        ✓          ✓          ✓          ✓         ✓     ✓
 * Pre-init crash         ✓          ✓          ✓          ✓         -     -
 * Boot stage cookie      ✓          ✓          -          -         -     -
 * Full fault regs        ✓(CFSR)    ✓(mepc)    ✓(PC)      ✓(PC)     -     -
 * RTC backup regs        -          -          -          -         ✓     -
 * FRAM black box         -          -          -          -         -     ✓
 * Flash concurrency      ✓          -          -          -         -     -
 *
 * Trinity CANNOT catch:
 *   nRF52840/ESP-C3Z: Crashes before PRE_KERNEL_1 -- JLink/OpenOCD only.
 *   IDF:              Crashes before constructor -- ROM boot faults only.
 *   STM32:            No pre-init canary (no .noinit mechanism used).
 *   All:              Stack corruption so severe the fault handler can't run.
 ******************************************************************************/
