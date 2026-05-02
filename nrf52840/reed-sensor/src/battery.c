/******************************************************************************
 * \file battery.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Battery measurement for nRF52840 reed sensor node.
 *
 * \details Reads SOC and voltage from MAX17048 fuel gauge over I2C1
 *          (P1.13 SDA, P1.15 SCL). Uses Zephyr fuel_gauge API.
 *          Returns -1 / 0 on error so callers can store NULL in SQLite.
 *          SOC is clamped to 100 -- ModelGauge reports >100% on fresh
 *          cells until the algorithm settles (~few minutes).
 *
 *          Hardware:
 *          - Board:      Teyleten Robot Pro Micro nRF52840
 *          - Fuel gauge: Adafruit MAX17048 breakout (PID 5580)
 *          - I2C bus:    I2C1, nordic,nrf-twim, 100kHz
 *          - SDA:        P1.13 (header pin SDA1)
 *          - SCL:        P1.15 (header pin SCL1)
 *          - Address:    0x36
 *
 * \warning NEXT DEVELOPER -- READ THIS BEFORE CHANGING BATTERY MEASUREMENT.
 *          Five approaches were tested empirically on this exact hardware
 *          before arriving at the working MAX17048 configuration. Do not
 *          re-attempt any of them. They have all been tried and the failure
 *          modes are fully understood.
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
 * \note    TEST 5 -- MAX17048 fuel gauge IC, first attempt (FAILED) (2026-05-01):
 *          Adafruit MAX17048 breakout (PID 5580) connected over I2C1.
 *          IC powered via VIN pin, SDA/SCL wired to P1.13/P1.15.
 *          Result: Consistent NACK on address 0x36. Logic analyzer confirmed
 *          address byte transmitted correctly but no ACK returned.
 *          Root cause: On this specific Adafruit board revision, VIN and BAT
 *          are NOT the same net. The MAX17048 CELL pin is only powered through
 *          the BAT net (JST connector). Without battery on CELL, the IC
 *          powers up but refuses all I2C transactions.
 *          Note: The Adafruit datasheet and multiple online sources incorrectly
 *          state that VIN and BAT are tied together. They are not on this
 *          board revision. Confirmed by logic analyzer showing correct address
 *          transmission with consistent NACK, and by multimeter showing VIN
 *          powered but CELL floating.
 *          Additional debugging performed:
 *          - Swapped two brand new ICs -- same result, ruling out faulty IC.
 *          - Tried nordic,nrf-twi vs nordic,nrf-twim -- twim required.
 *          - Tried &pinctrl wrapper vs direct node reference -- direct
 *            node reference required to avoid pin conflict with board dtsi.
 *          - Confirmed pins P1.13/P1.15 correct via logic analyzer.
 *          - Confirmed 100kHz clock-frequency required for stable operation.
 *
 * \note    TEST 6 -- MAX17048 with BAT jumper to VIN (CURRENT IMPLEMENTATION)
 *          (2026-05-01):
 *          Same wiring as Test 5 but with a jumper wire from BAT pad to VIN
 *          pin on the Adafruit breakout, connecting the CELL input to the
 *          CR2032 rail. Logic analyzer immediately showed full I2C transaction:
 *          AW:36 ACK, Data write ACK, AR:36 ACK, Data read ACK, STOP.
 *          SOC reads correctly, clamped to 100% (ModelGauge reports >100%
 *          on fresh cells for a few minutes until algorithm settles).
 *
 * \note    MAX17048 wiring (2026-05-01):
 *          Adafruit breakout PID 5580 -- THIS BOARD REVISION REQUIRES A
 *          JUMPER FROM BAT PAD TO VIN PIN. Without this jumper the CELL
 *          input floats and the IC NACKs all I2C transactions.
 *          VIN  → CR2032 B+ (board VCC rail)
 *          GND  → GND
 *          SDA  → P1.13 (header pin SDA1) -- 10K pullup onboard
 *          SCL  → P1.15 (header pin SCL1) -- 10K pullup onboard
 *          BAT  → jumper to VIN (required -- see Test 5/6 notes above)
 *          ALRT → floating
 *
 * \note    Enable pin removed (2026-03-29):
 *          Original design used P0.29 as a high-side enable switch.
 *          P0.29 was wired through a resistor to P0.31 (AIN7), not to
 *          Vbat, so the enable had no effect. With the enable inactive,
 *          the ADC pin floated and saturated at raw=4095 (3.6V full
 *          scale), producing ghost voltages. Fix: removed enable pin
 *          entirely. No GPIO needed.
 *
 * \note    I2C driver fix (2026-05-01):
 *          Board default for i2c1 is nordic,nrf-twi (legacy interrupt-driven
 *          driver). This must be overridden to nordic,nrf-twim (EasyDMA) in
 *          the overlay. The legacy driver causes NACK on address under BLE
 *          radio activity due to timing conflicts. See promicro_nrf52840.overlay
 *          for the compatible override and pinctrl configuration.
 *          Direct node reference (&i2c1_default / &i2c1_sleep) required --
 *          wrapping in &pinctrl {} causes duplicate label conflicts with the
 *          board dtsi identical to the UART pin issue (2026-03-29).
 ******************************************************************************/

#include "battery.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/logging/log.h>
#include "trinity_log.h"

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define VBAT_BUF_SIZE   48    /**< vbat log message buffer size */
#define SOC_MAX         100u  /**< clamp -- ModelGauge reports >100 on fresh cells */

static const struct device *g_fg = NULL;

int battery_init(void)
{
   g_fg = DEVICE_DT_GET_ANY(maxim_max17048);

   if (NULL == g_fg)
   {
      LOG_ERR("MAX17048 device not found in DT");
      return -ENODEV;
   }

   if (!device_is_ready(g_fg))
   {
      LOG_ERR("MAX17048 not ready");
      g_fg = NULL;
      return -ENODEV;
   }

   LOG_INF("MAX17048 ready");
   return 0;
}

int battery_read_mv(void)
{
   union fuel_gauge_prop_val val;
   int                       rc = 0;

   if (NULL == g_fg) { return -EIO; }

   rc = fuel_gauge_get_prop(g_fg, FUEL_GAUGE_VOLTAGE, &val);
   if (0 != rc)
   {
      LOG_ERR("MAX17048 voltage read failed (rc=%d)", rc);
      return -EIO;
   }

   /* FUEL_GAUGE_VOLTAGE returns uV -- convert to mV */
   return val.voltage / 1000;
}

uint8_t battery_read_soc(void)
{
   union fuel_gauge_prop_val val;
   int                       rc  = 0;
   char                      buf[VBAT_BUF_SIZE] = {0};
   uint8_t                   soc = 0;

   if (NULL == g_fg) { return 0; }

   rc = fuel_gauge_get_prop(g_fg, FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE, &val);
   if (0 != rc)
   {
      LOG_ERR("MAX17048 SOC read failed (rc=%d)", rc);
      return 0;
   }

   /* Clamp to 100 -- ModelGauge reports >100% on fresh cells until
    * the algorithm settles over the first few minutes of operation. */
   soc = (val.relative_state_of_charge > SOC_MAX) ?
         SOC_MAX : (uint8_t)val.relative_state_of_charge;

   LOG_INF("[BATT] SOC=%d%%", soc);

   (void)snprintf(buf, sizeof(buf), "EVENT: BATT_SOC %d%%\n", soc);
   trinity_log_event(buf);

   return soc;
}

void battery_print_status(void)
{
   int  mv                 = 0;
   char buf[VBAT_BUF_SIZE] = {0};

   mv = battery_read_mv();
   if (0 > mv) { LOG_ERR("Battery read failed"); return; }

   (void)snprintf(buf, sizeof(buf), "EVENT: BOOT | VBAT: %d mV\n", mv);
   trinity_log_event(buf);
   LOG_INF("VBAT: %d mV", mv);
}
