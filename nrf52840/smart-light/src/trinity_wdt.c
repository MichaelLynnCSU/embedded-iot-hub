/******************************************************************************
 * \file trinity_wdt.c
 * \brief Trinity watchdog -- nRF52840 Zephyr.
 *
 * \details Fully standalone -- no dependencies on other Trinity files.
 *          Feed bootloader WDT channel 0 only. Never calls
 *          wdt_install_timeout() or wdt_setup() -- both cause immediate
 *          chip reset on nRF52840 when bootloader WDT is already armed.
 *
 * \note    WDT field fix (2026-03-30):
 *          Confirmed via binary search: wdt_install_timeout() causes an
 *          immediate chip reset on nRF52840 when the bootloader WDT is
 *          already armed, even when -ENOTSUP/-EBUSY/-EPERM are all handled
 *          in the return-value path. The reset fires before any UART TX
 *          drains so it appears as pure silence with no output at all.
 *          wdt_setup() has the same problem.
 *          CONFIG_WDT_DISABLE_AT_BOOT is silently ignored on nRF52840
 *          because HAS_WDT_DISABLE_AT_BOOT is never selected by the nRF
 *          WDT driver Kconfig.
 *          Fix: grab wdt0, assume channel 0, feed immediately, return.
 *          Applies to bench, field, and USB modes identically.
 ******************************************************************************/

#include "trinity_log.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>

static const struct device *g_wdt       = NULL;
static int                  g_wdt_channel = -1;

/**
 * \brief  Initialise Trinity watchdog -- all modes identical on nRF52840.
 *
 * \details Grabs wdt0, assumes channel 0 (bootloader always uses ch0),
 *          feeds immediately. Never calls wdt_install_timeout() or
 *          wdt_setup() -- both cause immediate reset on nRF52840 when
 *          bootloader WDT is already armed (confirmed 2026-03-30).
 */
void trinity_wdt_init(void)
{
   g_wdt         = DEVICE_DT_GET(DT_NODELABEL(wdt0));
   g_wdt_channel = 0;

   if (!device_is_ready(g_wdt))
   {
      printk("[TRINITY] WDT device not ready\n");
      return;
   }

   wdt_feed(g_wdt, 0);

#if defined(CONFIG_TRINITY_MODE_BENCH)
   printk("[TRINITY] WDT bench-kick (bootloader WDT ch0)\n");
#else
   printk("[TRINITY] WDT field-kick (bootloader WDT ch0)\n");
#endif
}

void trinity_wdt_kick(void)
{
   if ((NULL != g_wdt) && (0 <= g_wdt_channel))
   {
      (void)wdt_feed(g_wdt, g_wdt_channel);
   }
}
