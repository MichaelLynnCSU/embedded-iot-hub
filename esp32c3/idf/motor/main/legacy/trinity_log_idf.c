/******************************************************************************
 * \file trinity_log_idf.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Trinity crash logger -- ESP32-C3 motor + ESP32 hub (ESP-IDF).
 *
 * \details Platform: ESP32-C3 (motor) and ESP32 (hub), ESP-IDF, FreeRTOS.
 *          Storage:  NVS string buffer (namespace set via CMakeLists.txt).
 *          Console:  ESP_LOG* macros (UART or USB CDC).
 *          Fault:    esp_panic_handler_user() override.
 *          Canary:   RTC_NOINIT_ATTR + __attribute__((constructor)).
 *          WDT:      esp_task_wdt, per-task registration, trigger_panic=true.
 *
 *          CMakeLists.txt requirements per board:
 *            motor:
 *              target_compile_definitions(${COMPONENT_LIB} PRIVATE
 *                  TRINITY_CHIP_ESP32C3_IDF=1
 *                  TRINITY_NVS_NAMESPACE="motor_log")
 *            hub:
 *              target_compile_definitions(${COMPONENT_LIB} PRIVATE
 *                  TRINITY_CHIP_ESP32_HUB_IDF=1
 *                  TRINITY_NVS_NAMESPACE="hub_log")
 *
 *          sdkconfig.defaults requirements (both boards):
 *            CONFIG_ESP_TASK_WDT_EN=y
 *            CONFIG_ESP_TASK_WDT_PANIC=y   ← CRITICAL: without this, WDT
 *                                            fires a warning but does NOT
 *                                            restart or call panic handler.
 *            CONFIG_ESP_TASK_WDT_TIMEOUT_S=5
 *            CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
 *            CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y
 *            CONFIG_FREERTOS_USE_TRACE_FACILITY=y
 *
 * \note    NVS namespace (2026-03-28):
 *          Previously hardcoded as "motor_log" and "hub_log" in separate .c
 *          files. Now a single file with TRINITY_NVS_NAMESPACE defined by
 *          CMakeLists.txt per board. Default falls back to "trinity_log" if
 *          not defined -- build will warn but compile.
 *
 * \note    Canary snapshot fix (2026-03-28):
 *          canary_constructor() copies g_pre_init_canary into g_canary_snapshot
 *          before overwriting with CANARY_ALIVE. dump_previous() reads the
 *          snapshot so it sees the previous session's true value.
 *          Same fix as all Zephyr siblings, using RTC_NOINIT_ATTR instead
 *          of .noinit ELF section.
 *
 * \note    Mutex fix (2026-03-28):
 *          trinity_log_event() protects the static NVS buffer and the full
 *          NVS read-modify-write sequence with g_log_mutex. The hub runs
 *          5 concurrent tasks; the motor runs 2. Both hit trinity_log_event()
 *          concurrently. Without the mutex, NVS writes corrupt each other.
 *          g_log_mutex created in trinity_log_init() before any task starts.
 *
 * \note    No sched-lock risk on IDF (2026-03-28):
 *          uxTaskGetSystemState() is safe to call from task context.
 *          No k_thread_foreach() spinlock involved -- the sched-lock
 *          violation that required collect-then-write on Zephyr does not
 *          apply here.
 ******************************************************************************/

#include "trinity_log.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_attr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_private/panic_internal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

/******************************** CONSTANTS ***********************************/

/* NVS namespace -- set per board via CMakeLists.txt compile definition.
 * Default warns at build time but compiles cleanly. */
#ifndef TRINITY_NVS_NAMESPACE
   #warning "TRINITY_NVS_NAMESPACE not defined -- defaulting to trinity_log"
   #define TRINITY_NVS_NAMESPACE  "trinity_log"
#endif

#define NVS_LOG_KEY          "events"
#define MAX_LOG_SIZE         2048
#define CRASH_BUF_SIZE       64
#define BOOT_BUF_SIZE        80
#define EVENT_ENTRY_SIZE     128
#define MSG_CLEAN_SIZE       96
#define MS_PER_US            1000ULL
#define STAT_BUF_SIZE        80u
#define WDT_TIMEOUT_S        5u

static const char *TAG = "TRINITY";

/************************** RTC NOINIT VARIABLES ******************************/

/* g_pre_init_canary: written CANARY_ALIVE by canary_constructor() and
 * upgraded to CANARY_BOOTED by trinity_log_init(). Survives warm resets.
 * Cleared on cold power-on (RTC RAM reset). */
static RTC_NOINIT_ATTR uint32_t g_pre_init_canary;

/* g_canary_snapshot: copy of g_pre_init_canary saved BEFORE overwrite.
 * Holds the previous session's canary value unmodified. */
static RTC_NOINIT_ATTR uint32_t g_canary_snapshot;

/**
 * \brief  Write alive canary at the earliest point before app_main.
 *
 * \details __attribute__((constructor)) runs after C runtime init but before
 *          app_main and before any component init that could crash. Saves
 *          previous canary into g_canary_snapshot before overwriting.
 *          On cold power-on, RTC memory is 0 -- correctly classified as
 *          cold boot by dump_previous().
 */
static void __attribute__((constructor)) canary_constructor(void)
{
    g_canary_snapshot = g_pre_init_canary;
    g_pre_init_canary = TRINITY_CANARY_ALIVE;
}

/************************** LOG MUTEX *****************************************/

/* Serializes static NVS buffer and NVS read-modify-write against concurrent
 * task callers. Created in trinity_log_init() before any task starts.
 * trinity_log_event() returns immediately if called before init. */
static SemaphoreHandle_t g_log_mutex = NULL;

/******************************************************************************
 * ESP panic handler override -- called on exceptions, task WDT timeout
 * (CONFIG_ESP_TASK_WDT_PANIC=y required), interrupt WDT, stack overflow,
 * abort(), and assert() failures.
 ******************************************************************************/
void esp_panic_handler_user(panic_info_t *p_info)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
   while (1) {}

#else
   char buf[CRASH_BUF_SIZE] = {0};

   if ((NULL != p_info) && (NULL != p_info->addr))
   {
      (void)snprintf(buf, sizeof(buf),
                     "EVENT: CRASH | PC: 0x%08lx\n",
                     (unsigned long)(uintptr_t)p_info->addr);
      trinity_log_event(buf);
   }
   else
   {
      trinity_log_event("EVENT: CRASH | PC: unknown\n");
   }

#if defined(CONFIG_TRINITY_MODE_USB) && CONFIG_TRINITY_MODE_USB
   vTaskDelay(pdMS_TO_TICKS(500));
#endif

   esp_restart();
#endif
}

/******************************************************************************
 * \brief  Append a timestamped event to the NVS log buffer.
 *
 * \details Acquires g_log_mutex to serialize the static buffer and NVS
 *          read-modify-write against concurrent task callers.
 *          Returns immediately (no-op) if called before trinity_log_init().
 *          Never crashes the system -- all failure paths return silently.
 ******************************************************************************/
void trinity_log_event(const char *p_msg)
{
   nvs_handle_t handle    = 0;
   size_t       len       = 0;
   size_t       cur_len   = 0;
   size_t       entry_len = 0;
   size_t       mlen      = 0;
   uint32_t     ms        = 0;

   static char buf[MAX_LOG_SIZE];
   char entry[EVENT_ENTRY_SIZE]   = {0};
   char msg_clean[MSG_CLEAN_SIZE] = {0};

   if (NULL == g_log_mutex) { return; }
   if (pdTRUE != xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(1000))) { return; }

   if (ESP_OK != nvs_open(TRINITY_NVS_NAMESPACE, NVS_READWRITE, &handle))
   {
      xSemaphoreGive(g_log_mutex);
      return;
   }

   (void)memset(buf, 0, sizeof(buf));
   len = sizeof(buf);
   (void)nvs_get_str(handle, NVS_LOG_KEY, buf, &len);

   ms = (uint32_t)(esp_timer_get_time() / MS_PER_US);

   (void)strncpy(msg_clean, p_msg, sizeof(msg_clean) - 1);
   msg_clean[sizeof(msg_clean) - 1] = '\0';
   mlen = strlen(msg_clean);
   if ((mlen > 0) && ('\n' == msg_clean[mlen - 1])) { msg_clean[mlen - 1] = '\0'; }

   (void)snprintf(entry, sizeof(entry), "[T+%lums] %s\n", (unsigned long)ms, msg_clean);

   cur_len   = strlen(buf);
   entry_len = strlen(entry);

   if ((cur_len + entry_len + 1u) >= MAX_LOG_SIZE)
   {
      (void)memmove(buf, buf + (MAX_LOG_SIZE / 2), MAX_LOG_SIZE / 2);
      (void)memset(buf + (MAX_LOG_SIZE / 2), 0, MAX_LOG_SIZE / 2);
      cur_len = strlen(buf);
   }

   (void)strncat(buf, entry, MAX_LOG_SIZE - cur_len - 1u);
   (void)nvs_set_str(handle, NVS_LOG_KEY, buf);
   (void)nvs_commit(handle);
   nvs_close(handle);

   xSemaphoreGive(g_log_mutex);
}

/******************************************************************************
 * \brief  Dump previous session NVS log and report canary state.
 *
 * \details Reads g_canary_snapshot (saved before overwrite by constructor).
 *          Must be called before trinity_log_init().
 ******************************************************************************/
void trinity_log_dump_previous(void)
{
   nvs_handle_t handle            = 0;
   char         buf[MAX_LOG_SIZE] = {0};
   size_t       len               = 0;
   esp_err_t    err               = ESP_OK;

   uint32_t canary   = g_canary_snapshot;
   g_canary_snapshot = 0U;

   if (TRINITY_CANARY_ALIVE == canary)
   {
      ESP_LOGW(TAG, "[TRINITY] EVENT: BOOT | REASON: PRE_INIT_CRASH");
      ESP_LOGW(TAG, "[TRINITY] NOTE: Crashed before trinity_log_init.");
      ESP_LOGW(TAG, "[TRINITY] NOTE: Component init fault -- no NVS log.");
   }
   else if (TRINITY_CANARY_BOOTED == canary)
   {
      ESP_LOGW(TAG, "[TRINITY] EVENT: BOOT | REASON: POST_INIT_CRASH");
      ESP_LOGW(TAG, "[TRINITY] NOTE: Crashed after trinity_log_init.");
      ESP_LOGW(TAG, "[TRINITY] NOTE: NVS log should contain fault details.");
   }
   else
   {
      ESP_LOGI(TAG, "[TRINITY] NOTE: Cold power-on or first flash.");
   }

   ESP_LOGI(TAG, "=== Previous Session Logs ===");

   if (ESP_OK != nvs_open(TRINITY_NVS_NAMESPACE, NVS_READONLY, &handle))
   {
      ESP_LOGI(TAG, "  (no logs found)");
      ESP_LOGI(TAG, "=== End of Previous Logs ===");
      return;
   }

   len = sizeof(buf);
   err = nvs_get_str(handle, NVS_LOG_KEY, buf, &len);
   nvs_close(handle);

   if ((ESP_OK == err) && (0 < strlen(buf))) { ESP_LOGI(TAG, "\n%s", buf); }
   else                                       { ESP_LOGI(TAG, "  (no logs found)"); }

   ESP_LOGI(TAG, "=== End of Previous Logs ===");
}

/******************************************************************************
 * \brief  Initialize Trinity logger and record boot reason.
 *
 * \details Creates g_log_mutex before writing the first log entry.
 *          Must be called before any task that calls trinity_log_event().
 *          Writes CANARY_BOOTED on success.
 ******************************************************************************/
void trinity_log_init(void)
{
   char               buf[BOOT_BUF_SIZE] = {0};
   esp_reset_reason_t reason             = ESP_RST_UNKNOWN;

   g_log_mutex = xSemaphoreCreateMutex();
   if (NULL == g_log_mutex)
   {
      ESP_LOGE(TAG, "[TRINITY] Failed to create log mutex -- logging disabled");
      return;
   }

   trinity_build_info_print();
   reason = esp_reset_reason();

   switch (reason)
   {
      case ESP_RST_PANIC:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: CRASH\n");
         break;
      case ESP_RST_BROWNOUT:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: BROWNOUT\n");
         break;
      case ESP_RST_WDT:
      case ESP_RST_TASK_WDT:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: WATCHDOG\n");
         break;
      case ESP_RST_SW:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: SOFT_RESET\n");
         break;
      case ESP_RST_POWERON:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: COLD_POWER_ON\n");
         break;
      default:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: NORMAL\n");
         break;
   }

   trinity_log_event(buf);
   ESP_LOGI(TAG, "%s", buf);

   g_pre_init_canary = TRINITY_CANARY_BOOTED;
}

void trinity_log_erase(void)
{
   nvs_handle_t handle = 0;
   if (ESP_OK != nvs_open(TRINITY_NVS_NAMESPACE, NVS_READWRITE, &handle)) { return; }
   (void)nvs_erase_key(handle, NVS_LOG_KEY);
   (void)nvs_commit(handle);
   nvs_close(handle);
}

/************************** PUBLIC FUNCTIONS -- WATCHDOG **********************/

void trinity_wdt_init(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
   ESP_LOGI(TAG, "[TRINITY] WDT skipped (bench mode)");
   return;
#endif

   esp_task_wdt_config_t cfg = {
      .timeout_ms     = WDT_TIMEOUT_S * 1000u,
      .idle_core_mask = 0,
      .trigger_panic  = true,  /* MUST be true -- without this WDT is silent */
   };
   esp_err_t err = esp_task_wdt_reconfigure(&cfg);

   if (ESP_OK != err)
   {
      ESP_LOGW(TAG, "Task WDT reconfigure failed: %s", esp_err_to_name(err));
      trinity_log_event("EVENT: WDT_INIT_FAIL\n");
      return;
   }

   ESP_LOGI(TAG, "[TRINITY] Task WDT armed (%u s, panic=true)", WDT_TIMEOUT_S);
   trinity_log_event("EVENT: WDT_ARMED\n");
}

void trinity_wdt_add(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
   return;
#endif
   esp_err_t err = esp_task_wdt_add(NULL);
   if (ESP_OK != err)
   {
      ESP_LOGW(TAG, "Task WDT add failed: %s", esp_err_to_name(err));
   }
}

void trinity_wdt_kick(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
   return;
#endif
   (void)esp_task_wdt_reset();
}

/************************** PUBLIC FUNCTIONS -- HEAP STATS ********************/

void trinity_log_heap_stats(void)
{
   size_t free_now = esp_get_free_heap_size();
   size_t free_min = esp_get_minimum_free_heap_size();
   char   msg[STAT_BUF_SIZE] = {0};

   (void)snprintf(msg, sizeof(msg),
                  "Heap: free=%u min_free=%u bytes\n",
                  (unsigned int)free_now,
                  (unsigned int)free_min);
   trinity_log_event(msg);
}

/************************** PUBLIC FUNCTIONS -- TASK STATS ********************/

/**
 * \brief  Log per-task CPU% and stack HWM.
 *
 * \details Uses uxTaskGetSystemState() -- safe from task context, no sched
 *          lock involved. No collect-then-write pattern needed (unlike Zephyr).
 *          Each line written via trinity_log_event() after full snapshot.
 */
void trinity_log_task_stats(void)
{
#if (configGENERATE_RUN_TIME_STATS == 1)
   TaskStatus_t task_array[16]     = {0};
   UBaseType_t  task_count         = 0u;
   uint32_t     total_time         = 0ul;
   UBaseType_t  i                  = 0u;
   uint32_t     cpu_pct            = 0ul;
   char         msg[STAT_BUF_SIZE] = {0};

   task_count = uxTaskGetSystemState(task_array, 16u, &total_time);

   trinity_log_event("[TRINITY] Task stats:\n");

   for (i = 0u; i < task_count; i++)
   {
      cpu_pct = (total_time > 0ul)
                ? (task_array[i].ulRunTimeCounter * 100ul) / total_time
                : 0ul;

      (void)snprintf(msg, sizeof(msg),
                     "[TRINITY] %-16s cpu=%2lu%%  hwm=%u words\n",
                     task_array[i].pcTaskName,
                     (unsigned long)cpu_pct,
                     (unsigned int)task_array[i].usStackHighWaterMark);
      trinity_log_event(msg);
   }
#else
   trinity_log_event("[TRINITY] Task stats: FREERTOS_GENERATE_RUN_TIME_STATS not set\n");
#endif
}
