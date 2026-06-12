/******************************************************************************
 * \file trinity_wdt_idf.c
 * \brief Trinity task watchdog -- ESP-IDF (motor + hub).
 *
 * \details Fully standalone. Uses esp_task_wdt with trigger_panic=true.
 *          CONFIG_ESP_TASK_WDT_PANIC=y required in sdkconfig.defaults --
 *          without it the WDT fires a warning but does NOT restart or call
 *          the panic handler.
 *
 *          Bench mode: WDT skipped entirely. USB-JTAG halts the CPU which
 *          would fire the task WDT. OpenOCD is the safety net.
 *
 *          Per-task registration: each task must call trinity_wdt_add() then
 *          trinity_wdt_kick() regularly. Unlike nRF52840 there is no single
 *          channel 0 -- each task registers independently.
 ******************************************************************************/

#include "trinity_log.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#define WDT_TIMEOUT_S  5u

static const char *TAG = "TRINITY_WDT";

void trinity_wdt_init(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH) && CONFIG_TRINITY_MODE_BENCH
   ESP_LOGI(TAG, "[TRINITY] WDT skipped (bench mode)");
   return;
#endif

   esp_task_wdt_config_t cfg = {
      .timeout_ms     = WDT_TIMEOUT_S * 1000u,
      .idle_core_mask = 0,
      .trigger_panic  = true,
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
