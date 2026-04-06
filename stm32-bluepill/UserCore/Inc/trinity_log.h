/******************************************************************************
 * \file trinity_log.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Unified Trinity crash and event logger interface -- all boards.
 *
 * \details Trinity is a portable crash-survive logging system covering 8
 *          boards across 5 platforms. One header, six platform .c files.
 *          Select platform at build time via a single flag:
 *
 *          Zephyr (prj.conf or west build -D):
 *            CONFIG_TRINITY_CHIP_NRF52840=y   -- reed-sensor, smart-lock, smart-light
 *            CONFIG_TRINITY_CHIP_ESP32C3=y    -- pir (ESP32-C3 Zephyr)
 *
 *          IDF (CMakeLists.txt target_compile_definitions):
 *            TRINITY_CHIP_ESP32C3_IDF=1       -- motor (ESP32-C3 IDF)
 *            TRINITY_CHIP_ESP32_HUB_IDF=1     -- hub (ESP32 IDF)
 *            TRINITY_NVS_NAMESPACE="motor_log" or "hub_log"
 *
 *          STM32 HAL (CMakeLists.txt target_compile_definitions):
 *            TRINITY_CHIP_STM32_F4=1          -- stm32-blackpill (bare metal)
 *            TRINITY_CHIP_STM32_F1=1          -- stm32-bluepill (FreeRTOS)
 *
 *          Exactly ONE chip flag must be defined per build.
 *
 * \details Common API (all platforms):
 *            trinity_log_dump_previous()  -- dump last session flash/NVS/FRAM log
 *            trinity_log_init()           -- init logger for new session
 *            trinity_log_erase()          -- erase log storage
 *            trinity_log_event(msg)       -- append event string to log
 *            trinity_wdt_init()           -- arm hardware watchdog
 *            trinity_wdt_kick()           -- feed watchdog
 *            trinity_log_heap_stats()     -- log heap usage
 *            trinity_log_task_stats()     -- log per-thread/task CPU and stack
 *
 *          nRF52840 Zephyr additions:
 *            trinity_log_boot_reason(resetreas) -- decode + log RESETREAS
 *            trinity_flash_lock()               -- acquire NVMC mutex
 *            trinity_flash_unlock()             -- release NVMC mutex
 *            g_init_stage                       -- .noinit boot stage cookie
 *            TRINITY_STAGE_* constants
 *
 *          IDF additions:
 *            trinity_wdt_add()    -- register calling task with task WDT
 *            trinity_build_info_print() -- log build version string
 *
 *          STM32 additions:
 *            TRINITY_ERROR_E enum
 *            panic_handler(reason, err)
 *            trinity_uart_log(msg)
 *            trinity_rtc_store(err, ...)
 *            trinity_hard_fault_handler()
 *            trinity_watchdog_init()  -- alias for trinity_wdt_init()
 *            trinity_watchdog_kick()  -- alias for trinity_wdt_kick()
 *            trinity_malloc(size)
 *            trinity_free(pp_ptr)
 *            trinity_paint_stack()    -- STM32F4 only
 *            trinity_check_stack()    -- STM32F4 only
 *
 * \note    Pre-init canary values (all platforms):
 *            TRINITY_CANARY_ALIVE  (0xCA11AB1E) -- crashed before log_init
 *            TRINITY_CANARY_BOOTED (0xB007ED00) -- log_init completed OK
 *            anything else                       -- cold boot / first flash
 *
 * \note    Zephyr: canary lives in .noinit ELF section, written at PRE_KERNEL_1.
 *          IDF: canary lives in RTC_NOINIT_ATTR, written by constructor.
 *          STM32: no canary system -- FRAM crash log used instead.
 *
 * \note    trinity_log_event() is safe to call concurrently on all platforms:
 *          Zephyr nRF: serialized by g_flash_mutex (NVMC single-writer).
 *          Zephyr ESP:  flash driver serializes internally, no extra mutex.
 *          IDF:         serialized by g_log_mutex (FreeRTOS mutex).
 *          STM32:       polled UART, bare-metal single-threaded or RTOS hook.
 ******************************************************************************/

#ifndef INCLUDE_TRINITY_LOG_H_
#define INCLUDE_TRINITY_LOG_H_

/******************************************************************************
 * Platform detection guard -- exactly one flag must be set.
 *****************************************************************************/
#if defined(CONFIG_TRINITY_CHIP_NRF52840) || \
    defined(CONFIG_TRINITY_CHIP_ESP32C3)  || \
    defined(TRINITY_CHIP_ESP32C3_IDF)     || \
    defined(TRINITY_CHIP_ESP32_HUB_IDF)   || \
    defined(TRINITY_CHIP_STM32_F4)        || \
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
#endif

/******************************************************************************
 * CANARY VALUES -- identical across all platforms.
 *****************************************************************************/

/** Written at earliest possible pre-main point. Trinity not yet init. */
#define TRINITY_CANARY_ALIVE   0xCA11AB1EU

/** Written at end of trinity_log_init(). Full init completed. */
#define TRINITY_CANARY_BOOTED  0xB007ED00U

/******************************************************************************
 * INIT STAGE COOKIE -- Zephyr platforms only.
 *
 * Written to g_init_stage (.noinit) before each risky init call in main().
 * Read and printed by trinity_log_dump_previous() when PRE_INIT_CRASH fires.
 * 0x0000 = crashed before main() wrote its first cookie.
 *****************************************************************************/
#if defined(CONFIG_TRINITY_CHIP_NRF52840) || defined(CONFIG_TRINITY_CHIP_ESP32C3)

#define TRINITY_STAGE_RESET         0x0000u  /**< Cleared after successful dump    */
#define TRINITY_STAGE_GPIO          0x0101u  /**< About to init GPIO / PWM / LED   */
#define TRINITY_STAGE_BATTERY       0x0102u  /**< About to call battery_init()     */
#define TRINITY_STAGE_NVS_INIT      0x0201u  /**< About to settings_subsys_init()  */
#define TRINITY_STAGE_NVS_LOAD      0x0202u  /**< About to load settings           */
#define TRINITY_STAGE_LOG_INIT      0x0203u  /**< About to trinity_log_init()      */
#define TRINITY_STAGE_WDT_INIT      0x0301u  /**< About to trinity_wdt_init()      */
#define TRINITY_STAGE_BT_ENABLE     0x0302u  /**< About to bt_enable()             */
#define TRINITY_STAGE_BT_ADV        0x0303u  /**< About to bt_le_adv_start()       */
#define TRINITY_STAGE_WIFI_INIT     0x0305u  /**< About to wifi_init() (ESP alias) */
#define TRINITY_STAGE_MAIN_LOOP     0x0304u  /**< Entered main while(1)            */

/**
 * Boot stage cookie in .noinit RAM.
 * Defined in trinity_log_zephyr_*.c; declared here for writes from main().
 * Read and cleared by trinity_log_dump_previous().
 */
extern volatile uint32_t g_init_stage;

#endif /* Zephyr platforms */

/******************************************************************************
 * COMMON API -- all platforms.
 *****************************************************************************/

/**
 * \brief  Dump previous session log and report pre-init crash if detected.
 * \details Call at the very top of main()/app_main() before anything else.
 */
void trinity_log_dump_previous(void);

/**
 * \brief  Initialize Trinity for the current session.
 * \details Zephyr: opens flash partition, inits mutex, allocates stat buffer.
 *          IDF: creates log mutex, logs boot reason via esp_reset_reason().
 *          STM32: reads FRAM history, logs UART, classifies reset cause.
 * \return 0 on success, negative errno on failure (Zephyr only).
 *         IDF and STM32 return void.
 */
#if defined(CONFIG_TRINITY_CHIP_NRF52840) || defined(CONFIG_TRINITY_CHIP_ESP32C3)
int  trinity_log_init(void);
#else
void trinity_log_init(void);
#endif

/**
 * \brief  Erase the persistent log storage.
 * \return 0 on success (Zephyr only).
 */
#if defined(CONFIG_TRINITY_CHIP_NRF52840) || defined(CONFIG_TRINITY_CHIP_ESP32C3)
int  trinity_log_erase(void);
#else
void trinity_log_erase(void);
#endif

/**
 * \brief  Append a timestamped event string to the persistent log.
 * \param  p_msg  Null-terminated event string. May contain newline.
 * \details Thread-safe on all platforms. Never crashes the system.
 *          No-op if called before trinity_log_init() completes.
 */
void trinity_log_event(const char *p_msg);

/**
 * \brief  Arm the hardware watchdog.
 * \details Zephyr nRF: arms Zephyr WDT driver (bench: feeds bootloader WDT).
 *          Zephyr ESP: arms Zephyr WDT driver (bench: skips entirely).
 *          IDF: reconfigures esp_task_wdt with panic=true.
 *          STM32: arms IWDG via HAL.
 */
void trinity_wdt_init(void);

/**
 * \brief  Feed (kick) the hardware watchdog.
 * \details Call at least every 3s from the main loop (Zephyr/STM32) or
 *          from each task loop (IDF). Safe to call before trinity_wdt_init().
 */
void trinity_wdt_kick(void);

/**
 * \brief  Log heap usage statistics to the persistent log and console.
 * \details Zephyr: uses Zephyr sys_heap_runtime_stats_get().
 *          IDF: uses esp_get_free_heap_size() / esp_get_minimum_free_heap_size().
 *          STM32F4: uses newlib mallinfo().
 *          STM32F1: uses FreeRTOS xPortGetFreeHeapSize().
 */
void trinity_log_heap_stats(void);

/**
 * \brief  Log per-thread/task CPU% and stack high-water marks.
 * \details Zephyr: collect-then-write pattern via thread_analyzer_run().
 *          IDF: snapshot via uxTaskGetSystemState() (no sched lock needed).
 *          STM32F1: via vTaskGetRunTimeStats() + uxTaskGetSystemState().
 *          STM32F4: not available (bare metal, no RTOS).
 */
void trinity_log_task_stats(void);

/******************************************************************************
 * nRF52840 ZEPHYR ADDITIONS
 *****************************************************************************/
#if defined(CONFIG_TRINITY_CHIP_NRF52840)

/**
 * \brief  Decode and log the nRF52840 reset reason.
 * \details Writes: EVENT: BOOT | RESETREAS: 0xXXXXXXXX | REASON: <label>
 *          Raw hex always included so combined causes (DOG|REPOR) are visible.
 *          Must be called AFTER trinity_log_init().
 * \param  resetreas  Value captured from NRF_POWER->RESETREAS at the top of
 *                    main() before clearing it. main() must do:
 *                      uint32_t reset_reason = NRF_POWER->RESETREAS;
 *                      NRF_POWER->RESETREAS  = 0xFFFFFFFF;
 *                    Do NOT pass the live register value.
 */
void trinity_log_boot_reason(uint32_t resetreas);

/**
 * \brief  Acquire the Trinity flash mutex before any non-Trinity flash write.
 * \details Required because nRF52840 NVMC cannot handle concurrent access.
 *          Wraps settings_save_one() or any flash write concurrent with
 *          trinity_log_event(). No-op before trinity_log_init() completes.
 *          Must NOT be held when calling trinity_log_event().
 */
void trinity_flash_lock(void);

/**
 * \brief  Release the Trinity flash mutex after a non-Trinity flash write.
 */
void trinity_flash_unlock(void);

#endif /* CONFIG_TRINITY_CHIP_NRF52840 */

/******************************************************************************
 * IDF ADDITIONS (ESP32-C3 motor + ESP32 hub)
 *****************************************************************************/
#if defined(TRINITY_CHIP_ESP32C3_IDF) || defined(TRINITY_CHIP_ESP32_HUB_IDF)

/**
 * \brief  Register the calling task with the ESP task watchdog.
 * \details Must be called once from each FreeRTOS task on startup.
 *          No-op in bench mode.
 */
void trinity_wdt_add(void);

/**
 * \brief  Print the build version string to the ESP log console.
 * \details Implemented in build_info.c on IDF boards.
 */
void trinity_build_info_print(void);

#endif /* IDF platforms */

/******************************************************************************
 * STM32 ADDITIONS (blackpill F411 + bluepill F103)
 *****************************************************************************/
#if defined(TRINITY_CHIP_STM32_F4) || defined(TRINITY_CHIP_STM32_F1)

#define TRINITY_MSG_LEN   80u  /**< Max panic/log message length incl NUL */

/**
 * \brief Error codes stored in FRAM crash log.
 * \details Written by trinity_rtc_store() and read by trinity_log_init()
 *          on the next boot. Used in panic_handler(), fault handlers,
 *          and FreeRTOS hooks.
 */
typedef enum
{
   eTRINITY_ERR_NONE        = 0x00u, /*!< Clean boot -- no prior fault        */
   eTRINITY_ERR_HARDFAULT   = 0x01u, /*!< ARM HardFault exception             */
   eTRINITY_ERR_STACK       = 0x02u, /*!< Stack canary or FreeRTOS overflow   */
   eTRINITY_ERR_HEAP        = 0x03u, /*!< malloc / FreeRTOS heap failure      */
   eTRINITY_ERR_ASSERT      = 0x04u, /*!< Assertion failure                   */
   eTRINITY_ERR_BROWNOUT    = 0x05u, /*!< Brown-out / power-loss reset        */
   eTRINITY_ERR_WATCHDOG    = 0x06u, /*!< Watchdog timeout reset              */
   eTRINITY_ERR_PANIC       = 0x07u, /*!< Generic panic_handler() call        */
   eTRINITY_ERR_UNKNOWN     = 0xFFu, /*!< Unclassified reset cause            */
} TRINITY_ERROR_E;

/**
 * \brief  Write a null-terminated string to UART (polled, blocking).
 * \details Safe to call from fault handlers. F411 uses UART1 via g_huart1.
 *          F103 uses UART1 via huart1. F411 USB mode queues via log_enqueue().
 */
void trinity_uart_log(const char *p_msg);

/**
 * \brief  Store an error code to FRAM crash log (survives power loss).
 * \details F411: trinity_rtc_store(err) -- one arg.
 *          F103: trinity_rtc_store(err, boot_count) -- two args.
 */
#if defined(TRINITY_CHIP_STM32_F4)
void trinity_rtc_store(TRINITY_ERROR_E err);
#else
void trinity_rtc_store(TRINITY_ERROR_E err, uint8_t boot_count);
#endif

/**
 * \brief  Panic handler -- log reason, store to FRAM, then halt or reset.
 * \details Never returns. Bench: BKPT + while(1). Field/USB: NVIC_SystemReset.
 */
void panic_handler(const char *p_reason, TRINITY_ERROR_E err);

/**
 * \brief  ARM HardFault handler entry point.
 * \details Call from HardFault_Handler() in stm32*_it.c.
 *          Logs to UART, stores to FRAM, then halts or resets.
 */
void trinity_hard_fault_handler(void);

/**
 * \brief  malloc wrapper that panics on NULL return.
 * \return Allocated memory. Never returns NULL.
 */
void *trinity_malloc(size_t size);

/**
 * \brief  free wrapper that NULLs the pointer after freeing.
 */
void trinity_free(void **pp_ptr);

/** Watchdog aliases -- STM32 uses trinity_watchdog_* naming internally */
#define trinity_watchdog_init  trinity_wdt_init
#define trinity_watchdog_kick  trinity_wdt_kick

#endif /* STM32 platforms */

/******************************************************************************
 * STM32F4 ONLY ADDITIONS (blackpill bare metal)
 *****************************************************************************/
#if defined(TRINITY_CHIP_STM32_F4)

#define STACK_CANARY   0xDEADBEEFul  /**< Magic value written at stack base */

/**
 * \brief  Write the stack canary at the base of the main stack.
 * \details Call once from main() after trinity_log_init().
 */
void trinity_paint_stack(void);

/**
 * \brief  Check stack canary -- panic if guard word corrupted.
 * \details Call from heartbeat_tick() in the main loop.
 */
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
 * TRINITY COVERAGE MATRIX
 *
 * Fault type          nRF52840  ESP32-C3Z  ESP32-C3I  ESP32hub  F411  F103
 * ─────────────────── ────────  ─────────  ─────────  ────────  ────  ────
 * HardFault / panic      ✓          ✓          ✓          ✓       ✓     ✓
 * WDT reset              ✓          ✓          ✓          ✓       ✓     ✓
 * Brownout               ✓          ✓          ✓          ✓       ✓     ✓
 * Stack overflow         ✓(MPU)     ✓(MPU)     ✓(RTOS)    ✓       ✓     ✓
 * Heap exhaustion        ✓          ✓          ✓          ✓       ✓     ✓
 * Pre-init crash         ✓          ✓          ✓          ✓       -     -
 * Boot stage cookie      ✓          ✓          -          -       -     -
 * Full fault regs        ✓(CFSR)    ✓(mepc)    ✓(PC)      ✓(PC)   -     -
 * FRAM black box         -          -          -          -       ✓     ✓
 * Flash concurrency      ✓          -          -          -       -     -
 *
 * Trinity CANNOT catch:
 *   nRF52840/ESP: Crashes before PRE_KERNEL_1 -- JLink/OpenOCD only.
 *   IDF:          Crashes before constructor -- ROM boot faults only.
 *   STM32:        No pre-init canary (no .noinit mechanism used).
 *   All:          Stack corruption so severe the fault handler cannot run.
 ******************************************************************************/
