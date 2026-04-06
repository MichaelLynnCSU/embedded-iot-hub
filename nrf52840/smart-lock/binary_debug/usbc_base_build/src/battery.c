#include "battery.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include "trinity_log.h"

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

/*
 * \note    k_sleep -> k_busy_wait fix (2026-03-27):
 *          battery_read_mv() is called from batt_work_handler which runs on
 *          sysworkq. k_sleep() must never be called from a work queue handler
 *          -- it blocks the entire sysworkq thread, corrupting the work queue's
 *          internal scheduler state when the next work item tries to execute.
 *          Confirmed crash: PC=0x5954494E ("NITY"), SP=0x003BA538 (corrupt),
 *          occurring at exactly 300s (first batt_work fire, BATT_UPDATE_SEC).
 *          Fix: replaced k_sleep(K_MSEC(1)) with k_busy_wait(1000).
 *          k_busy_wait() spins the CPU for 1ms without yielding the thread,
 *          safe from any context including work queue handlers and ISRs.
 *          1ms busy-wait is negligible for a 300s periodic battery read.
 */

/* ADC channel config from overlay */
static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

/* Divider enable pin P0.29 */
static const struct gpio_dt_spec div_enable =
    GPIO_DT_SPEC_GET(DT_NODELABEL(vbat_enable), gpios);

/* Voltage divider ratio: R2/(R1+R2) = 100k/300k = 0.333 */
#define DIVIDER_RATIO_NUM   1
#define DIVIDER_RATIO_DEN   3

/* 4x AA thresholds in mV */
#define VBAT_FULL_MV        6400
#define VBAT_LOW_MV         5200
#define VBAT_CRITICAL_MV    4800
#define VBAT_DEAD_MV        4400

int battery_init(void)
{
    if (!adc_is_ready_dt(&adc_channel)) {
        LOG_ERR("ADC not ready");
        return -1;
    }
    if (adc_channel_setup_dt(&adc_channel) != 0) {
        LOG_ERR("ADC channel setup failed");
        return -1;
    }
    if (!gpio_is_ready_dt(&div_enable)) {
        LOG_ERR("Divider enable GPIO not ready");
        return -1;
    }
    gpio_pin_configure_dt(&div_enable, GPIO_OUTPUT_INACTIVE);
    return 0;
}

int battery_read_mv(void)
{
    int16_t raw = 0;
    struct adc_sequence seq = {
        .buffer      = &raw,
        .buffer_size = sizeof(raw),
    };
    adc_sequence_init_dt(&adc_channel, &seq);

    /* Enable divider */
    gpio_pin_set_dt(&div_enable, 0);   /* LOW = GND = divider active */

    /* 1ms settle -- k_busy_wait() instead of k_sleep():
     * k_sleep() blocks the calling thread and must never be called from
     * a work queue handler. k_busy_wait() spins without yielding and is
     * safe from sysworkq, ISR, and any other non-sleeping context. */
    k_busy_wait(1000);

    trinity_wdt_kick();          /* kick before potentially blocking ADC read */
    int err = adc_read_dt(&adc_channel, &seq);
    trinity_wdt_kick();          /* kick after */

    /* Disable divider */
    gpio_pin_configure_dt(&div_enable, GPIO_INPUT);  /* float = no drain */

    if (err != 0) {
        LOG_ERR("ADC read failed: %d", err);
        return -1;
    }

    /* Convert raw to mV */
    int32_t mv = raw;
    adc_raw_to_millivolts_dt(&adc_channel, &mv);

    /* Scale back up through divider ratio */
    int vbat_mv = (int)(mv * DIVIDER_RATIO_DEN / DIVIDER_RATIO_NUM);
    return vbat_mv;
}

void battery_print_status(void)
{
    printk("Reading battery...\n");
    int vbat_mv = battery_read_mv();
    printk("battery_read_mv returned %d\n", vbat_mv);

    if (vbat_mv < 0) {
        printk("Battery read failed\n");
        return;
    }

    printk("VBAT: %d mV\n", vbat_mv);

    char vbat_buf[48];
    snprintf(vbat_buf, sizeof(vbat_buf), "EVENT: BOOT | VBAT: %d mV\n", vbat_mv);
    trinity_log_event(vbat_buf);

    const char *status;
    if      (vbat_mv >= VBAT_LOW_MV)      { status = "GOOD";     }
    else if (vbat_mv >= VBAT_CRITICAL_MV) { status = "LOW";      }
    else if (vbat_mv >= VBAT_DEAD_MV)     { status = "CRITICAL"; }
    else                                   { status = "DEAD";     }

    LOG_INF("VBAT: %d mV (%s)", vbat_mv, status);
}
