/******************************************************************************
 * \file main.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Reed sensor node BLE advertiser for door/window state monitoring.
 *
 * \details Monitors a reed switch via GPIO interrupt and broadcasts
 *          open/closed state over BLE manufacturer data advertisements.
 *          Battery SOC is included in the advertisement payload and
 *          updated every 5 minutes.
 *
 *          Hardware:
 *          - Board:    Pro Micro nRF52840
 *          - Reed pin: GPIO0 pin 11 (active-low, pull-up)
 *          - LED:      DT alias led0
 *          - Battery:  ADC via resistor divider on P0.02
 *
 *          Trinity additions:
 *          - trinity_wdt_init()        — arms HW watchdog (~3s timeout,
 *                                        no-op in BENCH mode — see note)
 *          - trinity_wdt_kick()        — fed every REED_POLL_MS in main loop
 *                                        AND at each boot checkpoint
 *          - trinity_log_heap_stats()  — logged every STATS_INTERVAL_TICKS
 *          - trinity_log_task_stats()  — logged every STATS_INTERVAL_TICKS
 *
 * \note    WDT fix (2026-03-21):
 *          REED_POLL_MS reduced from 10000 ms to 2000 ms. The previous
 *          value was 10 s — more than 3x the WDT timeout (3 s) — so the
 *          watchdog fired on every boot where the reed was stable.
 *          trinity_wdt_kick() is now called BEFORE the timed sem_take so
 *          the WDT is reset even when no interrupt or heartbeat arrives.
 *          BATT_UPDATE_TICKS and STATS_INTERVAL_TICKS updated to preserve
 *          the original 5-minute and 60-second intervals at the new rate.
 *
 * \note    WDT bench fix (2026-03-22):
 *          trinity_wdt_init() is a no-op when CONFIG_TRINITY_MODE_BENCH=y.
 *          On a fresh nRF52840 (after nrfjprog --recover), wdt_setup() with
 *          WDT_OPT_PAUSE_HALTED_BY_DBG causes an immediate reset under
 *          JLink before RTT output is flushed. Guard lives in
 *          trinity_wdt_init() so all projects benefit automatically.
 *          bench.conf also sets CONFIG_WATCHDOG=n as belt-and-suspenders.
 *
 * \note    WDT boot-feed fix (2026-03-22):
 *          Progressive WDT feeding added at each boot checkpoint in both
 *          main() and ble_thread(). The 3s WDT window can be exhausted
 *          during boot by bt_enable() (~1-2s) and settings_load() (NVS).
 *          Each trinity_wdt_kick() after a slow call gives the next stage
 *          a fresh 3s budget. ble_thread() runs concurrently with main()
 *          so it must feed independently — the main loop kicks do not
 *          cover BLE thread blocking time.
 *
 * \note    BLE adv race fix (2026-03-23):
 *          Replaced ble_adv_restart() (bt_le_adv_stop + sleep + bt_le_adv_start)
 *          with bt_le_adv_update_data(). The stop/start pattern raced against
 *          the BLE radio ISR (prepare_cb in lll_adv.c:1041) which was already
 *          scheduled at hardware level when stop was called. Under rapid reed
 *          toggling the ISR fired mid-teardown, hit a controller assertion, and
 *          threw KERNEL_OOPS. bt_le_adv_update_data() updates the payload
 *          atomically while advertising continues — no stop/start, no race.
 *          Also fixes a secondary race: g_mfg_data[] was written from thread
 *          context and read by the radio ISR with no lock. The update_data API
 *          handles synchronization internally.
 *          BLE_ADV_RESTART_MS define removed (no longer needed).
 *
 * \note    BLE thread WDT fix (2026-03-23):
 *          ble_thread while(1) was blocking on K_FOREVER with no WDT kick.
 *          In field mode with WDT active, any period of reed inactivity
 *          longer than 3s would fire the watchdog. Changed to bounded
 *          K_TIMEOUT_ABS_TICKS(REED_POLL_MS) — same pattern as main loop —
 *          with trinity_wdt_kick() called unconditionally at top of loop.
 *
 * \note    RESETREAS clear fix (2026-03-27):
 *          NRF_POWER->RESETREAS is a latched OR-history register -- it is NOT
 *          cleared automatically on reset. Old firmware never cleared it, so
 *          stale DOG bits from previous sessions survived across flash cycles
 *          and power cycles, causing the first boot with new firmware to report
 *          WATCHDOG even on a clean cold power-on. Confirmed: first boot after
 *          flashing showed RESETREAS=DOG|REPOR, misclassified as WATCHDOG.
 *          Fix: read RESETREAS into reset_reason at the very top of main(),
 *          then immediately write 0xFFFFFFFF to clear all latched bits before
 *          any other code reads the register. Subsequent boots now report
 *          accurate per-boot reset causes. Was already present in this file;
 *          note added for consistency across all three boards.
 *
 * \note    RESETREAS raw hex logging (2026-03-27):
 *          Reset reason log entry now includes both the decoded label (for
 *          quick human scanning) and the raw RESETREAS hex value (for full
 *          forensic fallback). Format:
 *            EVENT: BOOT | RESETREAS: 0xXXXXXXXX | REASON: <label>
 *          Protects against Nordic adding new bits in future silicon,
 *          misclassification edge cases, and combined-reset events
 *          (e.g. DOG|POR) that a priority-ordered decoder would hide.
 *          Zero runtime cost -- same log entry, wider format.
 *          Old log_boot_reason() static function removed; replaced with
 *          decode_reset_reason() helper matching smart-lock and smart-light.
 *          trinity_log_init() return value now checked before logging,
 *          consistent with smart-lock.
 ******************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <nrf.h>
#include <string.h>
#include "trinity_log.h"
#include "battery.h"
#include "main.h"

LOG_MODULE_REGISTER(reed_sensor, LOG_LEVEL_INF);

#define LED_NODE                DT_ALIAS(led0)
#define REED_PORT               DT_NODELABEL(gpio0)
#define REED_PIN                11

#define HEARTBEAT_INTERVAL_SEC  120
#define BLE_STACK_SIZE          4096
#define BLE_PRIORITY            5
#define REED_DEBOUNCE_MS        50

static const struct gpio_dt_spec g_led =
   GPIO_DT_SPEC_GET(LED_NODE, gpios);     /**< LED gpio spec */
static const struct device *g_gpio0;      /**< gpio0 device handle */

K_SEM_DEFINE(g_reed_changed_sem, 0, 1);
K_MSGQ_DEFINE(g_ble_msgq, sizeof(uint8_t), 8, 4);

static uint8_t g_mfg_data[MFG_DATA_SIZE] =
   {MFG_COMPANY_ID, 0x00, 0x00};          /**< BLE manufacturer payload */

static struct bt_data g_adv_data[] =
{
   BT_DATA_BYTES(BT_DATA_FLAGS,
      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
   BT_DATA(BT_DATA_NAME_COMPLETE,
      CONFIG_BT_DEVICE_NAME,
      sizeof(CONFIG_BT_DEVICE_NAME) - 1),
   BT_DATA(BT_DATA_MANUFACTURER_DATA,
      g_mfg_data,
      sizeof(g_mfg_data))
};

static struct bt_le_adv_param g_adv_param =
{
   .id           = BT_ID_DEFAULT,
   .options      = BT_LE_ADV_OPT_USE_IDENTITY,
   .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
   .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
};

static struct gpio_callback g_reed_cb_data; /**< reed gpio callback data */



static uint8_t battery_read_soc(void)
{
   int mv = battery_read_mv();
   if (0 > mv) { LOG_WRN("[BATT] Read failed (err=%d)", mv); return 0; }
   uint8_t soc = mv_to_soc(mv);
   LOG_INF("[BATT] %d mV -> %d%%", mv, soc);
   return soc;
}

static void heartbeat_handler(struct k_timer *p_timer)
{
   int ret = 0;
   uint8_t state;

   (void)p_timer;

   state = (uint8_t)gpio_pin_get(g_gpio0, REED_PIN);
   ret = k_msgq_put(&g_ble_msgq, &state, K_NO_WAIT);
   if (0 != ret) { LOG_WRN("[HB] msgq full, state drop (err=%d)", ret); }
   else          { LOG_INF("[HB] Sent"); }
}
K_TIMER_DEFINE(g_heartbeat_timer, heartbeat_handler, NULL);

static void reed_interrupt_handler(const struct device *p_dev,
                                   struct gpio_callback *p_cb,
                                   uint32_t pins)
{
   (void)p_dev;
   (void)p_cb;
   (void)pins;

   k_sem_give(&g_reed_changed_sem);
}

/**
 * \brief  Update BLE advertisement payload with current state and battery.
 *
 * \details Uses bt_le_adv_update_data() to atomically swap the manufacturer
 *          data while advertising continues. Does NOT stop/restart the
 *          advertiser — avoids the prepare_cb race that caused KERNEL_OOPS
 *          when the radio ISR fired mid-teardown during rapid reed toggling.
 *          g_mfg_data[] must be updated by the caller before this call.
 */
static int ble_broadcast(void)
{
   int err = bt_le_adv_update_data(g_adv_data,
                                    ARRAY_SIZE(g_adv_data),
                                    NULL, 0);
   if (0 != err)
   {
      LOG_ERR("[BLE] Adv update failed (err=%d)", err);
   }
   else
   {
      LOG_INF("[BLE] Broadcasting state=%d batt=%d%%",
              g_mfg_data[1], g_mfg_data[2]);
   }
   return err;
}

void ble_thread(void *p_unused1, void *p_unused2, void *p_unused3)
{
   int err = 0;
   uint8_t state = 0;
   uint32_t update_count = 0;

   (void)p_unused1;
   (void)p_unused2;
   (void)p_unused3;

   LOG_INF("[BLE] Thread started");

   err = bt_enable(NULL);
   if (0 != err) { LOG_ERR("[BLE] Init failed (err=%d)", err); return; }
   trinity_wdt_kick(); /* ← feed after bt_enable (~1-2s) */

   LOG_INF("[BLE] Bluetooth enabled");

   if (IS_ENABLED(CONFIG_BT_SETTINGS))
   {
      err = settings_load();
      if (0 != err) { LOG_WRN("[BLE] Settings load failed (err=%d)", err); }
      trinity_wdt_kick(); /* ← feed after settings_load (NVS read) */
   }

   g_mfg_data[2] = battery_read_soc();
   LOG_INF("[BLE] Initial batt=%d%%", g_mfg_data[2]);

   err = bt_le_adv_start(&g_adv_param,
                          g_adv_data,
                          ARRAY_SIZE(g_adv_data),
                          NULL, 0);
   if (0 != err) { LOG_ERR("[BLE] Advertising start failed (err=%d)", err); return; }

   LOG_INF("[BLE] Advertising active | %s", CONFIG_BT_DEVICE_NAME);

   k_timer_start(&g_heartbeat_timer,
                 K_SECONDS(HEARTBEAT_INTERVAL_SEC),
                 K_SECONDS(HEARTBEAT_INTERVAL_SEC));

   while (1)
   {
      /* FIX: kick before blocking — gives fresh 3s WDT budget.
       * Previous code used K_FOREVER here — WDT fired whenever
       * the reed was undisturbed for > 3s in field mode.        */
      trinity_wdt_kick();

      (void)k_msgq_get(&g_ble_msgq, &state,
                        K_TIMEOUT_ABS_TICKS(
                           k_uptime_ticks() +
                           k_ms_to_ticks_ceil32(REED_POLL_MS)));

      trinity_wdt_kick(); /* ← feed after unblock, before adv update */

      update_count++;
      LOG_INF("[BLE] Update #%u state=%d (%s) batt=%d%%",
              update_count, state,
              state ? "OPEN" : "CLOSED",
              g_mfg_data[2]);

      g_mfg_data[1] = state;
      (void)ble_broadcast();
   }
}

static int gpio_init(void)
{
   int err = 0;

   g_gpio0 = DEVICE_DT_GET(REED_PORT);

   if (!device_is_ready(g_gpio0))     { LOG_ERR("GPIO0 not ready");  return -ENODEV; }
   if (!device_is_ready(g_led.port))  { LOG_ERR("LED not ready");    return -ENODEV; }

   err = gpio_pin_configure_dt(&g_led, GPIO_OUTPUT_INACTIVE);
   if (0 != err) { LOG_ERR("LED configure failed (err=%d)", err); return err; }

   err = gpio_pin_configure(g_gpio0, REED_PIN, GPIO_INPUT | GPIO_PULL_UP);
   if (0 != err) { LOG_ERR("Reed pin configure failed (err=%d)", err); return err; }

   err = gpio_pin_interrupt_configure(g_gpio0, REED_PIN, GPIO_INT_EDGE_BOTH);
   if (0 != err) { LOG_ERR("Reed interrupt configure failed (err=%d)", err); return err; }

   gpio_init_callback(&g_reed_cb_data, reed_interrupt_handler, BIT(REED_PIN));

   err = gpio_add_callback(g_gpio0, &g_reed_cb_data);
   if (0 != err) { LOG_ERR("Reed callback add failed (err=%d)", err); return err; }

   return 0;
}

static void reed_monitor_loop(int last_state)
{
   int err = 0;
   int val = 0;
   int batt_tick  = 0;
   int stats_tick = 0;
   uint8_t cur   = 0;
   uint8_t state = 0;

   while (1)
   {
      /* ---- WDT kick at top of loop — before any blocking call ---- */
      trinity_wdt_kick();

      (void)k_sem_take(&g_reed_changed_sem,
                        K_TIMEOUT_ABS_TICKS(
                           k_uptime_ticks() +
                           k_ms_to_ticks_ceil32(REED_POLL_MS)));

      batt_tick++;
      if (batt_tick >= BATT_UPDATE_TICKS)
      {
         batt_tick = 0;
         g_mfg_data[2] = battery_read_soc();
         cur = (uint8_t)gpio_pin_get(g_gpio0, REED_PIN);
         err = k_msgq_put(&g_ble_msgq, &cur, K_NO_WAIT);
         if (0 != err) { LOG_WRN("[BATT] msgq full, batt update drop (err=%d)", err); }
      }

      stats_tick++;
      if (stats_tick >= STATS_INTERVAL_TICKS)
      {
         stats_tick = 0;
         trinity_log_heap_stats();
         trinity_log_task_stats();
      }

      k_sleep(K_MSEC(REED_DEBOUNCE_MS));

      val = gpio_pin_get(g_gpio0, REED_PIN);
      if (0 > val)
      {
         LOG_WRN("[REED] Pin read failed (err=%d)", val);
      }
      else if (val != last_state)
      {
         LOG_INF("[REED] Changed: %d (%s)", val, val ? "OPEN" : "CLOSED");

         trinity_log_event(val ?
            "EVENT: REED_OPEN\n" : "EVENT: REED_CLOSED\n");

         err = gpio_pin_set_dt(&g_led, !val);
         if (0 != err) { LOG_WRN("[REED] LED set failed (err=%d)", err); }

         state = (uint8_t)val;
         err = k_msgq_put(&g_ble_msgq, &state, K_NO_WAIT);
         if (0 != err) { LOG_WRN("[REED] msgq full, state drop (err=%d)", err); }

         last_state = val;
      }
   }
}

int main(void)
{
   int err = 0;
   int initial = 0;
   uint8_t init_state;
   uint32_t reset_reason = 0;
   k_tid_t tid;

   static K_THREAD_STACK_DEFINE(ble_stack, BLE_STACK_SIZE);
   static struct k_thread ble_thread_data;

   reset_reason = NRF_POWER->RESETREAS;
   NRF_POWER->RESETREAS = 0xFFFFFFFF;

#if !defined(CONFIG_TRINITY_MODE_BENCH)
   g_init_stage = TRINITY_STAGE_WDT_INIT;
   trinity_wdt_init();
   trinity_wdt_kick();
#endif

   LOG_INF("=== ReedSensor Boot ===");
   trinity_log_dump_previous();

   if (trinity_log_init() == 0) {
      trinity_log_boot_reason(reset_reason);
   }
   trinity_wdt_kick();

   trinity_log_dump_previous_deferred();

#if defined(CONFIG_TRINITY_MODE_BENCH)
   g_init_stage = TRINITY_STAGE_WDT_INIT;
   trinity_wdt_init();
   trinity_wdt_kick();
#endif

   err = gpio_init();
   if (0 != err) { return err; }
   trinity_wdt_kick();

   if (0 != battery_init()) { LOG_WRN("Battery init failed, SOC will read 0"); }
   trinity_wdt_kick();

   k_sleep(K_MSEC(100));

   initial = gpio_pin_get(g_gpio0, REED_PIN);
   if (0 > initial) { LOG_ERR("Reed initial read failed (err=%d)", initial); return initial; }

   LOG_INF("Initial state: %d (%s)", initial, initial ? "OPEN" : "CLOSED");

   err = gpio_pin_set_dt(&g_led, (initial ? 0 : 1));
   if (0 != err) { LOG_WRN("LED set failed (err=%d)", err); }

   init_state = (uint8_t)initial;
   err = k_msgq_put(&g_ble_msgq, &init_state, K_NO_WAIT);
   if (0 != err) { LOG_WRN("Initial state msgq put failed (err=%d)", err); }

   tid = k_thread_create(&ble_thread_data,
                          ble_stack, BLE_STACK_SIZE,
                          ble_thread,
                          NULL, NULL, NULL,
                          BLE_PRIORITY, 0, K_NO_WAIT);
   if (NULL == tid) { LOG_ERR("BLE thread create failed"); return -ENOMEM; }
   k_thread_name_set(tid, "ble_thread");

   trinity_wdt_kick();

   LOG_INF("BLE thread spawned");
   LOG_INF("Monitoring reed switch");

   reed_monitor_loop(initial);

   return 0;
}
