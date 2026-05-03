/******************************************************************************
 * \file trinity_nvs_esp.c
 * \brief NVS persistence -- ESP32-C3 Zephyr.
 *
 * \details Owns the NVS filesystem on storage_partition. Provides
 *          simple read/write API for persistent counters that must
 *          survive deep sleep cycles.
 *
 *          Trinity flash log uses log_partition (raw flash_area API).
 *          This file uses storage_partition (NVS). No overlap.
 *
 *          Partition layout (boards/esp32c3_devkitm.overlay):
 *            storage_partition: 0x3b0000, 176KB -- owned by this file
 *            log_partition:     0x3dc000,  16KB -- owned by trinity_flash_esp.c
 *
 * \note    Why NVS and not RTC_DATA_ATTR (2026-05-03):
 *          RTC_DATA_ATTR was attempted first. The Zephyr ESP32-C3 linker
 *          script has no proper .rtc.data section, so RTC_DATA_ATTR
 *          expands to __attribute__((section(".rtc.data"))) but the
 *          section is never placed in RTC slow memory (0x50000000).
 *          Boot log confirmed: load:0x50000000,len:0xc every wake --
 *          only 12 bytes (the wake stub), motion_count never appeared
 *          there. DRAM (0x3fc...) is reloaded from flash on every deep
 *          sleep wake, so any variable landing there resets to 0 every
 *          boot. Removing `static` from the declaration was also tried
 *          and made no difference -- the section attribute is a no-op
 *          regardless of storage class in this toolchain/Zephyr version.
 *          NVS on storage_partition is the reliable alternative.
 *
 * \note    Flash wear:
 *          nvs_write() is idempotent -- if the value has not changed it
 *          skips the write internally. motion_count only writes on
 *          confirmed PIR wakes, never on timer heartbeats. At 100 PIR
 *          events/day, NVS wear-leveling across 176KB gives decades of
 *          write endurance before concern.
 ******************************************************************************/

#include "trinity_log.h"
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trinity_nvs, LOG_LEVEL_INF);

#define MOTION_COUNT_NVS_ID   1U

static struct nvs_fs s_nvs_fs;
static bool          s_ready = false;

int trinity_nvs_init(void)
{
    struct flash_pages_info info;
    int                     rc = 0;

    s_nvs_fs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
    if (!device_is_ready(s_nvs_fs.flash_device))
    {
        LOG_ERR("NVS flash device not ready");
        return -ENODEV;
    }

    s_nvs_fs.offset = FIXED_PARTITION_OFFSET(storage_partition);

    rc = flash_get_page_info_by_offs(s_nvs_fs.flash_device,
                                     s_nvs_fs.offset, &info);
    if (0 != rc)
    {
        LOG_ERR("flash_get_page_info failed (%d)", rc);
        return rc;
    }

    s_nvs_fs.sector_size  = info.size;
    s_nvs_fs.sector_count = 2U;

    rc = nvs_mount(&s_nvs_fs);
    if (0 != rc)
    {
        LOG_ERR("nvs_mount failed (%d)", rc);
        return rc;
    }

    s_ready = true;
    LOG_INF("NVS mounted (offset=0x%08lX sector_size=%u)",
            (unsigned long)s_nvs_fs.offset, s_nvs_fs.sector_size);
    return 0;
}

uint32_t trinity_nvs_read_motion_count(void)
{
    uint32_t val = 0;
    int      rc  = 0;

    if (!s_ready) { return 0; }

    rc = nvs_read(&s_nvs_fs, MOTION_COUNT_NVS_ID, &val, sizeof(val));
    if (rc == -ENOENT)
    {
        LOG_INF("motion_count not found in NVS -- first boot, starting at 0");
        return 0;
    }
    if (rc < 0)
    {
        LOG_ERR("nvs_read failed (%d)", rc);
        return 0;
    }

    return val;
}

int trinity_nvs_write_motion_count(uint32_t val)
{
    int rc = 0;

    if (!s_ready) { return -ENODEV; }

    rc = nvs_write(&s_nvs_fs, MOTION_COUNT_NVS_ID, &val, sizeof(val));
    if (rc < 0)
    {
        LOG_ERR("nvs_write failed (%d)", rc);
        return rc;
    }

    return 0;
}
