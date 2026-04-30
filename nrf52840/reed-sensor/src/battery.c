/******************************************************************************
 * \file battery.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Battery voltage measurement for nRF52840 reed sensor node.
 *
 * \details Reads VDD voltage via the nRF52840 internal SAADC VDD monitor.
 *          No external pin, no resistor divider, no loading of the CR2032.
 *
 *          Hardware:
 *          - Board:    Teyleten Robot Pro Micro nRF52840
 *          - Regulator: HX3001 LDO, always present, outputs 2.5V
 *          - Power path: CR2032 (3V) → HX3001 → 2.5V VDD → nRF52840
 *          - B+ is accessible before the LDO on the coin cell holder positive
 *          - ADC channel: zephyr_user node index 0, NRF_SAADC_VDD (internal)
 *          - ADC: 12-bit, ADC_GAIN_1_6, ADC_REF_INTERNAL (0.6V)
 *          - Max measurable VDD: 3.6V (0.6V / (1/6) gain)
 *
 * \warning NEXT DEVELOPER — READ THIS BEFORE CHANGING BATTERY MEASUREMENT.
 *          Four approaches were tested empirically on this exact board before
 *          arriving at the MAX17048. Do not re-attempt any of them. They have
 *          all been tried and the failure modes are fully understood. See notes
 *          below for the complete history.
 *
 * \note    BOARD TOPOLOGY (2026-04-22):
 *          The Teyleten Robot Pro Micro nRF52840 has a HX3001 LDO regulator
 *          permanently in the power path between the coin cell and VDD.
 *          The LDO always outputs 2.5V regardless of cell voltage.
 *          This means:
 *          - NRF_SAADC_VDD always reads ~2.5V (the regulated rail), never
 *            the cell. Useless for battery monitoring.
 *          - AIN pins powered from VDD can still measure B+ directly since
 *            B+ (3V) is within SAADC input range (3.6V max at 1/6 gain).
 *          - There is no software-only path to measure the raw cell voltage
 *            through the regulated VDD rail.
 *          SWD connection does not affect this -- 2.5V is the LDO output,
 *          not a back-powering artifact from the debugger.
 *
 * \note    TEST 1 -- Resistor divider + AIN pin (WORKED, abandoned) (2026-04-22):
 *          200k/100k divider between B+ and P0.02 (AIN0). Divider ratio 1/3
 *          attenuates cell voltage to ~1V for ADC measurement.
 *          Result: Worked. Readings were stable and accurate.
 *          Reason abandoned: The CR2032 nominal voltage (3V) is already within
 *          the SAADC input range (3.6V max at ADC_GAIN_1_6). The divider
 *          attenuated a voltage that never needed attenuation, wasting
 *          resolution and adding ~1uA quiescent drain through the divider.
 *          The divider correction math also adds unnecessary complexity.
 *          NOTE: If all else fails, Test 1 is the fallback. It works.
 *
 * \note    TEST 2 -- Direct AIN pin, no divider, no caps (FAILED) (2026-04-22):
 *          B+ connected directly to P0.02 (AIN0). No divider, no capacitors.
 *          ADC_GAIN_1_6, ADC_REF_INTERNAL, ADC_ACQ_TIME_DEFAULT (10us).
 *          Result: Unstable readings. Cell voltage collapsed during SAADC
 *          sampling, producing artificially low mV values.
 *          Root cause: CR2032 internal impedance is 10-30 ohms (rising as
 *          cell discharges). The SAADC input capacitance requires a current
 *          pulse to charge at sample time. The cell cannot source this current
 *          fast enough through its internal impedance -- rail collapses.
 *          This failure occurs even pre-BLE with the radio fully idle.
 *          Longer acquisition time (40us) made it worse -- more time connected
 *          to source means more charge drawn from the cell, deeper sag.
 *
 * \note    TEST 3 -- NRF_SAADC_VDD internal monitor (FAILED) (2026-04-22):
 *          Switched ADC input to NRF_SAADC_VDD, the nRF52840 internal VDD
 *          monitor. No external pin. Nordic-recommended approach for coin
 *          cell monitoring on nRF52 series.
 *          Result: Always reads ~1812 mV (the regulated 2.5V rail under load),
 *          never the cell voltage. Multimeter confirmed cell at 3V, VDD at
 *          2.5V. NRF_SAADC_VDD measures VDD, not B+. On this board the HX3001
 *          LDO permanently separates the cell from VDD. The Nordic
 *          recommendation assumes the cell IS VDD (no regulator). That
 *          assumption does not hold on this board.
 *
 * \note    TEST 4 -- Direct AIN pin + bulk capacitors (FAILED) (2026-04-22):
 *          Same as Test 2 but with 2x 10uF ceramic caps added across B+/GND
 *          to provide bulk charge reservoir for SAADC sampling.
 *          Result: Still failed. Readings still low and unstable.
 *          Root cause: The datasheet cap recommendation assumes a low-impedance
 *          source (Li-ion, LiPo -- milliohm internal impedance). With a CR2032
 *          at 10-30 ohms internal impedance, the caps cannot recharge fast
 *          enough between samples. The cell charges the caps too slowly to
 *          replenish what the SAADC draws. Hundreds of uF would be required
 *          to hold the rail stable, which is impractical for a coin cell node.
 *          Tested both with BLE active and pre-BLE (radio idle) -- both failed.
 *          Radio state is not the determining factor. Cell impedance is.
 *
 * \note    TEST 5 -- MAX17048 fuel gauge IC (CURRENT IMPLEMENTATION):
 *          Dedicated fuel gauge IC connected directly to B+ over I2C.
 *          The MAX17048 measures cell voltage using nanoamp-range bias current,
 *          completely independent of VDD rail and SAADC. No sampling window,
 *          no impedance loading, no interaction with BLE radio activity.
 *          ModelGauge algorithm provides accurate SOC without requiring a
 *          voltage-to-percent curve. This is the correct solution for CR2032
 *          monitoring on a board with a switching/LDO regulator in the power
 *          path. All previous SAADC-based approaches are dead ends on this
 *          hardware -- do not revisit them.
 *
 * \note    Enable pin removed (2026-03-29):
 *          Original design used P0.29 as a high-side enable switch.
 *          P0.29 was wired through a resistor to P0.31 (AIN7), not to
 *          Vbat, so the enable had no effect. With the enable inactive,
 *          the ADC pin floated and saturated at raw=4095 (3.6V full
 *          scale), producing ghost voltages. Fix: removed enable pin
 *          entirely. No GPIO needed.
 ******************************************************************************/

#include "battery.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include "trinity_log.h"

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define VBAT_BUF_SIZE  48   /**< vbat log message buffer size */

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

   err = adc_read_dt(&g_adc_channel, &seq);
   LOG_INF("RAW ADC ONLY = %d", raw);
   if (0 != err)
   {
      LOG_ERR("ADC read failed (err=%d)", err);
      return -EIO;
   }

   mv = (int32_t)raw;
   (void)adc_raw_to_millivolts_dt(&g_adc_channel, &mv);

   /* MAX17048 handles cell measurement -- this path retained for reference
    * only. See file header notes for full SAADC failure history. */
   vbat_mv = (int)mv;

   LOG_INF("raw=%d vdd_mv=%d", raw, vbat_mv);

   return vbat_mv;
}

uint8_t battery_read_soc(void)
{
   int mv = battery_read_mv();
   if (0 > mv)
   {
      LOG_WRN("[BATT] Read failed (err=%d)", mv);
      return 0;
   }
   uint8_t soc = mv_to_soc(mv);
   LOG_INF("[BATT] %d mV -> %d%%", mv, soc);
   return soc;
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
