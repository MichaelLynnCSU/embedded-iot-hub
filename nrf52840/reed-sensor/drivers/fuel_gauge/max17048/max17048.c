/* max17048.c - Driver for max17048 battery fuel gauge */
/* Custom version for CR2032 coin cell -- reed sensor node.
 *
 * Changes from upstream Zephyr driver:
 *   1. max17048_percent() replaced with CR2032 voltage-to-SOC lookup table.
 *      Upstream uses ModelGauge SOC register which is tuned for LiPo --
 *      unusable for coin cell. Voltage-based lookup is more accurate for CR2032.
 *   2. Quick-start added to max17048_init() -- forces ModelGauge to restart
 *      SOC estimation from current voltage on boot.
 *
 * Calibration (2026-05-01):
 *   Fresh CR2032 reads ~3600-3700mV from VCELL register (multimeter ~3.5V).
 *   Voltage is flat for most of cell life, drops sharply below 2800mV.
 *   Table thresholds based on empirical readings on this hardware.
 *
 * Copyright (c) 2023 Alvaro Garcia Gomez <maxpowel@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max17048

#include "max17048.h"

#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MAX17048);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "MAX17048 driver enabled without any devices"
#endif

#define RESET_COMMAND   0x5400
#define QUICKSTART_MODE 0x4000

struct max17048_config {
        struct i2c_dt_spec i2c;
};

struct max17048_data {
        uint8_t  charge;
        uint32_t voltage;
        uint16_t time_to_full;
        uint16_t time_to_empty;
        bool     charging;
};

int max17048_read_register(const struct device *dev, uint8_t registerId, uint16_t *response)
{
        uint8_t max17048_buffer[2];
        const struct max17048_config *cfg = dev->config;
        int rc = i2c_write_read_dt(&cfg->i2c, &registerId, sizeof(registerId), max17048_buffer,
                                   sizeof(max17048_buffer));
        if (rc) {
                LOG_ERR("Unable to read register, error %d", rc);
                return rc;
        }

        *response = sys_get_be16(max17048_buffer);
        return 0;
}

int max17048_adc(const struct device *i2c_dev, uint16_t *response)
{
        return max17048_read_register(i2c_dev, REGISTER_VCELL, response);
}

int max17048_voltage(const struct device *i2c_dev, uint32_t *response)
{
        uint16_t raw_voltage;
        int rc = max17048_adc(i2c_dev, &raw_voltage);

        if (rc) {
                return rc;
        }

        /* VCELL register: 16-bit value, 78.125uV per LSB.
         * Returns uV -- callers divide by 1000 for mV.
         * Upstream formula retained -- no shift needed.
         * Empirical calibration: fresh CR2032 reads ~3600-3700mV on this hardware.
         * Reference: MAX17048 datasheet page 10, Table 2. */
        *response = ((uint32_t)raw_voltage * 78125) / 1000;
        return 0;
}

int max17048_percent(const struct device *i2c_dev, uint8_t *response)
{
        uint32_t uv = 0;
        uint32_t mv = 0;
        int rc = max17048_voltage(i2c_dev, &uv);

        if (rc) { return rc; }

        mv = uv / 1000;

        /* CR2032 voltage-to-SOC lookup table.
         * ModelGauge SOC register not used -- tuned for LiPo, not coin cell.
         * Thresholds based on empirical VCELL readings on this hardware (2026-05-01):
         *   Fresh CR2032 = ~3600-3700mV from VCELL register.
         *   Voltage flat 3200-3600mV for most of cell life.
         *   Drops sharply below 2800mV near end of life. */
        if      (mv >= 3600) { *response = 100; }
        else if (mv >= 3500) { *response = 90;  }
        else if (mv >= 3400) { *response = 80;  }
        else if (mv >= 3300) { *response = 70;  }
        else if (mv >= 3200) { *response = 60;  }
        else if (mv >= 3100) { *response = 50;  }
        else if (mv >= 3000) { *response = 40;  }
        else if (mv >= 2900) { *response = 30;  }
        else if (mv >= 2800) { *response = 20;  }
        else if (mv >= 2600) { *response = 10;  }
        else if (mv >= 2000) { *response = 5;   }
        else                 { *response = 0;   }

        return 0;
}

int max17048_crate(const struct device *i2c_dev, int16_t *response)
{
        int rc = max17048_read_register(i2c_dev, REGISTER_CRATE, response);

        if (rc) {
                return rc;
        }

        *response *= 208;
        return 0;
}

static int max17048_init(const struct device *dev)
{
        const struct max17048_config *cfg = dev->config;
        uint16_t version;

        if (!device_is_ready(cfg->i2c.bus)) {
                LOG_ERR("Bus device is not ready");
                return -ENODEV;
        }

        int rc = max17048_read_register(dev, REGISTER_VERSION, &version);

        if (rc) {
                LOG_ERR("Cannot read from I2C");
                return rc;
        }

        version = version & 0xFFF0;
        if (version != 0x10) {
                LOG_ERR("Something found at the provided I2C address, but it is not a MAX17048");
                LOG_ERR("The version registers should be 0x10 but got %x. Maybe your wiring is "
                        "wrong or it is a fake chip\n", version);
                return -ENODEV;
        }

        /* Quick-start -- restart ModelGauge SOC estimation from current voltage.
         * Required for CR2032 on first boot to avoid 0% SOC reading.
         * Writes QUICKSTART_MODE (0x4000) to MODE register (0x06).
         * No fuel_gauge API exposes this -- must write directly over I2C. */
        {
                uint8_t qs_buf[3] = { REGISTER_MODE, 0x40, 0x00 };
                rc = i2c_write_dt(&cfg->i2c, qs_buf, sizeof(qs_buf));
                if (rc) {
                        LOG_WRN("Quick-start write failed (rc=%d)", rc);
                }
                k_sleep(K_MSEC(125));
        }

        return 0;
}

static int max17048_get_single_prop_impl(const struct device *dev, fuel_gauge_prop_t prop,
                                         union fuel_gauge_prop_val *val)
{
        struct max17048_data *data = dev->data;
        int rc = 0;

        switch (prop) {
        case FUEL_GAUGE_RUNTIME_TO_EMPTY:
                val->runtime_to_empty = data->time_to_empty;
                break;
        case FUEL_GAUGE_RUNTIME_TO_FULL:
                val->runtime_to_full = data->time_to_full;
                break;
        case FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE:
                val->relative_state_of_charge = data->charge;
                break;
        case FUEL_GAUGE_VOLTAGE:
                val->voltage = data->voltage;
                break;
        default:
                rc = -ENOTSUP;
        }

        return rc;
}

static int max17048_get_prop(const struct device *dev, fuel_gauge_prop_t prop,
                             union fuel_gauge_prop_val *val)
{
        struct max17048_data *data = dev->data;
        int rc = max17048_percent(dev, &data->charge);
        int16_t crate;
        int ret;

        if (rc) {
                LOG_ERR("Error while reading battery percentage");
                return rc;
        }

        rc = max17048_voltage(dev, &data->voltage);
        if (rc) {
                LOG_ERR("Error while reading battery voltage");
                return rc;
        }

        rc = max17048_crate(dev, &crate);
        if (rc) {
                LOG_ERR("Error while reading battery current rate");
                return rc;
        }

        if (crate != 0) {
                data->charging = crate > 0;

                if (data->charging) {
                        uint8_t percentage_pending = 100 - data->charge;
                        uint32_t hours_pending = percentage_pending * 1000000 / crate;

                        data->time_to_empty = 0;
                        data->time_to_full = hours_pending * 60 / 1000;
                } else {
                        uint32_t hours_pending = data->charge * 1000000 / -crate;

                        data->time_to_empty = hours_pending * 60 / 1000;
                        data->time_to_full = 0;
                }
        } else {
                data->charging = false;
                data->time_to_full = 0;
                data->time_to_empty = 0;
        }

        ret = max17048_get_single_prop_impl(dev, prop, val);

        return ret;
}

static DEVICE_API(fuel_gauge, max17048_driver_api) = {
        .get_property = &max17048_get_prop,
};

#define MAX17048_DEFINE(inst)                                                                      \
        static struct max17048_data max17048_data_##inst;                                          \
                                                                                                   \
        static const struct max17048_config max17048_config_##inst = {                             \
                .i2c = I2C_DT_SPEC_INST_GET(inst)};                                                \
                                                                                                   \
        DEVICE_DT_INST_DEFINE(inst, &max17048_init, NULL, &max17048_data_##inst,                   \
                              &max17048_config_##inst, POST_KERNEL,                                \
                              CONFIG_FUEL_GAUGE_INIT_PRIORITY, &max17048_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX17048_DEFINE)
