#include "max17048.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(max17048, LOG_LEVEL_INF);

#define MAX17048_ADDR       0x36
#define REG_VCELL           0x02  /* Cell voltage */
#define REG_SOC             0x04  /* State of charge */
#define REG_MODE            0x06  /* Mode register */
#define REG_VERSION         0x08  /* IC version */

static const struct i2c_dt_spec dev = I2C_DT_SPEC_GET(DT_NODELABEL(max17048));

static int read_reg16(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    int rc = i2c_write_read_dt(&dev, &reg, 1, buf, 2);
    if (rc != 0) return rc;
    *val = (buf[0] << 8) | buf[1];
    return 0;
}

int max17048_init(void)
{
    if (!i2c_is_ready_dt(&dev)) {
        LOG_ERR("I2C device not ready");
        return -1;
    }

    uint16_t version;
    if (read_reg16(REG_VERSION, &version) != 0) {
        LOG_ERR("MAX17048 not found on I2C bus");
        return -1;
    }

    LOG_INF("MAX17048 found, version: 0x%04x", version);
    return 0;
}

int max17048_read_mv(void)
{
    uint16_t raw;
    if (read_reg16(REG_VCELL, &raw) != 0) return -1;

    /* VCELL: 1 LSB = 78.125uV, result in mV */
    int mv = (raw >> 4) * 1000 / 800;
    return mv;
}

int max17048_read_soc(void)
{
    uint16_t raw;
    if (read_reg16(REG_SOC, &raw) != 0) return -1;

    /* SOC: upper byte = %, lower byte = 1/256% */
    int soc = raw >> 8;
    return soc;
}
