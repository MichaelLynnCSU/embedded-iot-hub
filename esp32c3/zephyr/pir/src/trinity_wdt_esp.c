/******************************************************************************
 * \file trinity_wdt_esp.c
 * \brief Trinity watchdog -- ESP32-C3 Zephyr.
 *
 * \details Fully standalone -- no dependencies on other Trinity files.
 *
 *          Bench mode: WDT skipped entirely. OpenOCD/USB-JTAG is the safety
 *          net. wdt_setup(WDT_OPT_PAUSE_HALTED_BY_DBG) resets the chip under
 *          the debugger before console output flushes on ESP32-C3.
 *
 *          Field/USB: wdt_install_timeout() and wdt_setup() work correctly
 *          on ESP32-C3 -- no bootloader WDT conflict unlike nRF52840. The
 *          ESP-IDF bootloader does not arm the TWDT before handoff.
 ******************************************************************************/

#include "trinity_log.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>

#define WDT_TIMEOUT_MS  3000u

static const struct device *g_wdt       = NULL;
static int                  g_wdt_channel = -1;

void trinity_wdt_init(void)
{
#if defined(CONFIG_TRINITY_MODE_BENCH)
   printk("[TRINITY] WDT skipped (bench mode)\n");
   return;
#endif

   struct wdt_timeout_cfg wdt_cfg = {0};
   int                    rc      = 0;

   g_wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));

   if (!device_is_ready(g_wdt))
   {
      printk("[TRINITY] WDT device not ready\n");
      trinity_log_event("EVENT: WDT_INIT_FAIL\n");
      return;
   }

   wdt_cfg.window.min = 0u;
   wdt_cfg.window.max = WDT_TIMEOUT_MS;
   wdt_cfg.callback   = NULL;

   g_wdt_channel = wdt_install_timeout(g_wdt, &wdt_cfg);
   if (0 > g_wdt_channel)
   {
      printk("[TRINITY] WDT install timeout failed: %d\n", g_wdt_channel);
      trinity_log_event("EVENT: WDT_TIMEOUT_FAIL\n");
      return;
   }

   rc = wdt_setup(g_wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
   if (0 != rc)
   {
      printk("[TRINITY] WDT setup failed: %d\n", rc);
      trinity_log_event("EVENT: WDT_SETUP_FAIL\n");
      return;
   }

   printk("[TRINITY] WDT armed (%u ms timeout)\n", WDT_TIMEOUT_MS);
   trinity_log_event("EVENT: WDT_ARMED\n");
}

void trinity_wdt_kick(void)
{
   if ((NULL != g_wdt) && (0 <= g_wdt_channel))
   {
      (void)wdt_feed(g_wdt, g_wdt_channel);
   }
}
