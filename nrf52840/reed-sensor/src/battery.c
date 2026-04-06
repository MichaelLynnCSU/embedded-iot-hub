/******************************************************************************
 * \file battery.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Battery voltage measurement for nRF52840 reed sensor node.
 *
 * \details Reads battery voltage via ADC through a fixed resistor divider.
 *          No software enable pin -- divider is always connected.
 *
 *          Hardware:
 *          - ADC channel: zephyr_user node index 0, P0.02 (AIN0)
 *          - Divider: Vbat → 100k → 100k → ┬ → P0.02(AIN0)
 *                                           └ → 100k → GND
 *          - R1 = 200k (two 100k in series), R2 = 100k
 *          - Ratio: R2/(R1+R2) = 100k/300k = 1/3
 *          - Reconstruct: adc_pin_mV × 3 = Vbat_mV
 *          - ADC: 12-bit, ADC_GAIN_1_6, ADC_REF_INTERNAL (0.6V)
 *          - Max ADC input: 3.6V (0.6V / (1/6) gain)
 *          - CR2032 fresh ~3000mV → ADC pin ~1000mV → raw ~1137 → ×3
 *          - Quiescent draw: ~10uA at 3V (3V/300k) -- acceptable for
 *            CR2032 (220mAh typical = ~2.5 years at 10uA continuous)
 *
 * \note    Enable pin removed (2026-03-29):
 *          Original design used P0.29 as a high-side enable switch.
 *          P0.29 was wired through a resistor to P0.31 (AIN7), not to
 *          Vbat, so the enable had no effect. With the enable inactive,
 *          the ADC pin floated and saturated at raw=4095 (3.6V full
 *          scale), producing ghost voltages (~8580mV after ×3).
 *          Fix: removed enable pin entirely. Divider is hardwired
 *          between Vbat and GND -- always on, no GPIO needed.
 *
 * \note    ADC settle time (2026-03-29):
 *          Increased from 1ms to 10ms. CR2032 has high internal
 *          impedance under load -- 1ms is insufficient for the ADC
 *          input capacitance to charge through 200k+100k = 300k.
 *          At 10ms the reading is stable across temperature.
 ******************************************************************************/

#include "battery.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include "trinity_log.h"

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define DIVIDER_RATIO_NUM    1    /**< R2 = 100k                              */
#define DIVIDER_RATIO_DEN    3    /**< R1+R2 = 200k+100k = 300k              */
#define VBAT_BUF_SIZE        48   /**< vbat log message buffer size           */
#define ADC_SETTLE_MS        10   /**< ADC input settle time (ms)             */

static const struct adc_dt_spec g_adc_channel =
   ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

int battery_init(void)
{
   int err = 0;

   if (!adc_is_ready_dt(&g_adc_channel))
   {
      LOG_ERR("ADC not ready");
      return -ENODEV;
   }

   err = adc_channel_setup_dt(&g_adc_channel);
   if (0 != err)
   {
      LOG_ERR("ADC channel setup failed (err=%d)", err);
      return err;
   }

   return 0;
}

int battery_read_mv(void)
{
   int     err     = 0;
   int16_t raw     = 0;
   int32_t mv      = 0;
   int     vbat_mv = 0;

   struct adc_sequence seq =
   {
      .buffer      = &raw,
      .buffer_size = sizeof(raw),
   };

   (void)adc_sequence_init_dt(&g_adc_channel, &seq);

   /* Settle: allow ADC input cap to charge through 300k divider */
   k_sleep(K_MSEC(ADC_SETTLE_MS));

   err = adc_read_dt(&g_adc_channel, &seq);
   if (0 != err)
   {
      LOG_ERR("ADC read failed (err=%d)", err);
      return -EIO;
   }

   mv = (int32_t)raw;
   (void)adc_raw_to_millivolts_dt(&g_adc_channel, &mv);

   /* Reconstruct Vbat: ADC pin sees Vbat/3, multiply back by 3 */
   vbat_mv = (int)(mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM);

   LOG_INF("raw=%d adc_pin_mv=%d vbat_mv=%d", raw, (int)mv, vbat_mv);

   return vbat_mv;
}

void battery_print_status(void)
{
   int  vbat_mv                 = 0;
   char vbat_buf[VBAT_BUF_SIZE] = {0};

   vbat_mv = battery_read_mv();
   if (0 > vbat_mv) { LOG_ERR("Battery read failed"); return; }

   (void)snprintf(vbat_buf, sizeof(vbat_buf),
                  "EVENT: BOOT | VBAT: %d mV\n", vbat_mv);
   trinity_log_event(vbat_buf);
   LOG_INF("VBAT: %d mV", vbat_mv);
}
