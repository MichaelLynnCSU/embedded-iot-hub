/******************************************************************************
 * \file    battery.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Battery voltage measurement for nRF52840 smart lock node.
 *
 * \details Reads 4×AA battery voltage via ADC through a switched resistor
 *          divider. P0.29 sinks the bottom of the divider to GND to enable
 *          measurement, eliminating quiescent drain during sleep.
 *
 *          Hardware:
 *          - ADC channel: zephyr_user node index 0, P0.02 (AIN0)
 *          - Divider: Vbat → 100k → 100k → ┬ → P0.02 (AIN0)
 *                                           └ → 100k → P0.29 (driven LOW = GND sink)
 *          - R1 = 200k (two 100k in series), R2 = 100k
 *          - Ratio: R2/(R1+R2) = 100k/300k = 1/3
 *          - Reconstruct: adc_pin_mV × 3 = Vbat_mV
 *          - ADC: 12-bit, ADC_GAIN_1_6, ADC_REF_INTERNAL (0.6V)
 *          - Max ADC input: 3.6V (0.6V / (1/6) gain)
 *          - 4×AA fresh ~6400mV → ADC pin ~2133mV → within 3.6V max ✓
 *          - P0.29 overlay: GPIO_ACTIVE_LOW
 *            logical 1 (active)   = physical LOW = GND sink = divider on
 *            logical 0 (inactive) = physical HIGH = floating = divider off,
 *                                   zero quiescent drain
 *
 *          P0.29 voltage when enabled:
 *          - P0.29 is driven to 0V (GND sink) -- no overvoltage risk
 *          - AIN0 sees Vbat × 100k/300k = Vbat/3 ≈ 2133mV at 6400mV ✓
 *          - Well within nRF52840 IO and ADC limits
 *
 * \note    Why low-side GPIO switch instead of high-side:
 *
 *          Previous design wired P0.29 into the high-voltage side of the
 *          divider (Vbat → P0.29 → resistors → AIN0). With 4×AA fresh at
 *          ~6.4V, this applied 6.4V to an IO pin rated for 3.3V max. The
 *          ESD diodes on P0.29 clamped into the 3.3V rail, damaging the
 *          onboard regulator and permanently disabling USB on two boards.
 *
 *          Fix: P0.29 now sinks the bottom of R2 to GND (low-side switch).
 *          When enabled P0.29 is at 0V -- safe regardless of Vbat.
 *          No FET or additional components required.
 *
 * \note    Why the smart lock uses a switched divider but the reed sensor does not:
 *
 *          Reed sensor (CR2032):
 *          - Divider is always connected: Vbat → 200k+100k → GND
 *          - Always-on quiescent draw: 3V / 300k ≈ 10µA
 *          - CR2032 capacity: ~220mAh → 10µA continuous ≈ 2.5 years
 *          - Acceptable tradeoff: simpler hardware, no GPIO needed
 *
 *          Smart lock (4×AA):
 *          - 4×AA fresh voltage: ~6V → always-on draw: 6V / 300k ≈ 20µA
 *          - Lock draws significantly more current overall (motor, solenoid,
 *            BLE) so minimising every source of idle drain matters
 *          - Switching costs nothing in firmware complexity and is correct
 *            habit regardless of absolute current savings
 *          - Bottom line: CR2032 accepts 10µA always-on as a known budget
 *            item; 4×AA lock switches it because the habit is correct and
 *            the cost is zero
 *
 * \note    k_sleep -> k_busy_wait (2026-03-27):
 *          battery_read_mv() is called from batt_work_handler which runs on
 *          sysworkq. k_sleep() must never be called from a work queue handler
 *          -- it blocks the entire sysworkq thread, corrupting the work
 *          queue's internal scheduler state when the next work item tries to
 *          execute. Confirmed crash: PC=0x5954494E ("NITY"),
 *          SP=0x003BA538 (corrupt), occurring at exactly 300s (first
 *          batt_work fire, BATT_UPDATE_SEC).
 *          Fix: replaced k_sleep(K_MSEC(1)) with k_busy_wait(1000).
 *          k_busy_wait() spins the CPU without yielding the thread, safe
 *          from any context including work queue handlers and ISRs.
 *
 * \note    Settle time 10ms (2026-04-07):
 *          Increased from 1ms to 10ms. The divider source impedance is 300k
 *          (R1+R2 in series from the ADC pin's perspective). At 1ms the ADC
 *          input capacitance has insufficient time to charge through 300k,
 *          producing low readings. 10ms gives a stable result across
 *          temperature. 10ms busy-wait on a 300s read cycle is negligible.
 ******************************************************************************/

#include "battery.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include "trinity_log.h"

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

/* ADC channel config from overlay */
static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

/* Low-side GND sink: P0.29 GPIO_ACTIVE_LOW in overlay.
 * Logical 1 (active)   = physical LOW = GND sink = divider enabled.
 * Logical 0 (inactive) = physical HIGH = floating = divider disabled. */
static const struct gpio_dt_spec div_enable =
    GPIO_DT_SPEC_GET(DT_NODELABEL(vbat_enable), gpios);

/* Voltage divider ratio: R2/(R1+R2) = 100k/300k = 1/3 */
#define DIVIDER_RATIO_NUM    1
#define DIVIDER_RATIO_DEN    3

/* ADC input settle time -- 10ms required for 300k source impedance */
#define ADC_SETTLE_US        10000

/* 4×AA thresholds in mV */
#define VBAT_FULL_MV         6400
#define VBAT_LOW_MV          5200
#define VBAT_CRITICAL_MV     4800
#define VBAT_DEAD_MV         4400

#define VBAT_BUF_SIZE        48

int battery_init(void)
{
    if (!adc_is_ready_dt(&adc_channel))
    {
        LOG_ERR("ADC not ready");
        return -1;
    }
    if (adc_channel_setup_dt(&adc_channel) != 0)
    {
        LOG_ERR("ADC channel setup failed");
        return -1;
    }
    if (!gpio_is_ready_dt(&div_enable))
    {
        LOG_ERR("Divider enable GPIO not ready");
        return -1;
    }

    /* Logical 0 = physical HIGH = divider off, no quiescent drain */
    gpio_pin_configure_dt(&div_enable, GPIO_OUTPUT_INACTIVE);

    LOG_INF("Battery init OK (AIN0/P0.02, low-side GND sink P0.29)");
    return 0;
}

int battery_read_mv(void)
{
    int16_t  raw     = 0;
    int32_t  mv      = 0;
    int      err     = 0;
    int      vbat_mv = 0;

    struct adc_sequence seq =
    {
        .buffer      = &raw,
        .buffer_size = sizeof(raw),
    };

    adc_sequence_init_dt(&adc_channel, &seq);

    /* Logical 1 = physical LOW = GND sink = divider enabled */
    gpio_pin_set_dt(&div_enable, 1);

    /* 10ms settle: allow ADC input cap to charge through 300k source.
     * k_busy_wait() used -- safe from sysworkq context. See note in header. */
    k_busy_wait(ADC_SETTLE_US);

    trinity_wdt_kick();
    err = adc_read_dt(&adc_channel, &seq);
    trinity_wdt_kick();

    /* Logical 0 = physical HIGH = divider disabled, zero drain */
    gpio_pin_configure_dt(&div_enable, GPIO_OUTPUT_INACTIVE);

    if (err != 0)
    {
        LOG_ERR("ADC read failed: %d", err);
        return -1;
    }

    mv = (int32_t)raw;
    adc_raw_to_millivolts_dt(&adc_channel, &mv);

    /* Reconstruct Vbat: ADC pin sees Vbat/3, multiply back by 3 */
    vbat_mv = (int)(mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM);

    LOG_INF("raw=%d adc_pin_mv=%d vbat_mv=%d", raw, (int)mv, vbat_mv);

    return vbat_mv;
}

void battery_print_status(void)
{
    char        vbat_buf[VBAT_BUF_SIZE] = {0};
    const char *status                  = NULL;
    int         vbat_mv                 = 0;

    vbat_mv = battery_read_mv();
    if (0 > vbat_mv)
    {
        LOG_ERR("Battery read failed");
        return;
    }

    (void)snprintf(vbat_buf, sizeof(vbat_buf),
                   "EVENT: BOOT | VBAT: %d mV\n", vbat_mv);
    trinity_log_event(vbat_buf);

    if      (vbat_mv >= VBAT_LOW_MV)      { status = "GOOD";     }
    else if (vbat_mv >= VBAT_CRITICAL_MV) { status = "LOW";      }
    else if (vbat_mv >= VBAT_DEAD_MV)     { status = "CRITICAL"; }
    else                                   { status = "DEAD";     }

    LOG_INF("VBAT: %d mV (%s)", vbat_mv, status);
}
