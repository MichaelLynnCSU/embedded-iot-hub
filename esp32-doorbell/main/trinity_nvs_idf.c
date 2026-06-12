/******************************************************************************
 * \file trinity_nvs_idf.c
 * \brief Trinity NVS log -- ESP-IDF (motor + hub).
 *
 * \details Owns NVS namespace, log mutex, and all log I/O.
 *          Depends on trinity_canary_idf.c for RTC canary variables.
 *
 *          NVS namespace set via CMakeLists.txt compile definition:
 *            motor: TRINITY_NVS_NAMESPACE="motor_log"
 *            hub:   TRINITY_NVS_NAMESPACE="hub_log"
 *          Default fallback "trinity_log" warns at build time.
 *
 *          Mutex fix (2026-03-28):
 *          g_log_mutex serializes static NVS buffer and NVS read-modify-write
 *          against concurrent task callers. Hub has 5 tasks, motor has 2.
 *          Both call trinity_log_event() concurrently. Without mutex, NVS
 *          writes corrupt each other.
 ******************************************************************************/

#include "trinity_log.h"
#include "trinity_private_idf.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

#ifndef TRINITY_NVS_NAMESPACE
   #warning "TRINITY_NVS_NAMESPACE not defined -- defaulting to trinity_log"
   #define TRINITY_NVS_NAMESPACE  "trinity_log"
#endif

#define NVS_LOG_KEY       "events"
#define MAX_LOG_SIZE      2048
#define BOOT_BUF_SIZE     80
#define EVENT_ENTRY_SIZE  128
#define MSG_CLEAN_SIZE    96
#define MS_PER_US         1000ULL

static const char *TAG = "TRINITY_NVS";

static SemaphoreHandle_t g_log_mutex = NULL;

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
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: CRASH\n");     break;
      case ESP_RST_BROWNOUT:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: BROWNOUT\n");  break;
      case ESP_RST_WDT:
      case ESP_RST_TASK_WDT:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: WATCHDOG\n");  break;
      case ESP_RST_SW:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: SOFT_RESET\n"); break;
      case ESP_RST_POWERON:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: COLD_POWER_ON\n"); break;
      default:
         (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | REASON: NORMAL\n");    break;
   }

   trinity_log_event(buf);
   ESP_LOGI(TAG, "%s", buf);

   trinity_canary_set_booted();
}

void trinity_log_erase(void)
{
   nvs_handle_t handle = 0;
   if (ESP_OK != nvs_open(TRINITY_NVS_NAMESPACE, NVS_READWRITE, &handle)) { return; }
   (void)nvs_erase_key(handle, NVS_LOG_KEY);
   (void)nvs_commit(handle);
   nvs_close(handle);
}
