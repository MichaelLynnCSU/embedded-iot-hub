#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>

/* Advertisement burst durations */
#define ADV_BURST_PIR_MS      2000U   /* PIR wake   -- long burst  */
#define ADV_BURST_TIMER_MS     300U   /* Timer wake -- short burst */

/* Deep sleep heartbeat interval -- 30 seconds in microseconds.
 *
 * Hub sliding window configuration (ble_scan.c) this must align with:
 *   PIR_WINDOW_SEC       = 60s  -- window width for occupancy detection
 *   PIR_WINDOW_THRESHOLD = 2    -- events required within window
 *   PIR_HOLD_SEC         = 600s -- hold duration after last trigger
 *
 * At 30s heartbeat, two motion events within the 60s window still
 * satisfies PIR_WINDOW_THRESHOLD correctly. Hub comment states the
 * window cannot meaningfully be smaller than ~20-30s -- 30s heartbeat
 * sits at that boundary and is the maximum safe interval.
 *
 * Hub online/offline threshold (ble_manager.c):
 *   BLE_DEV_PIR age < 60s -- updated from 30s to match 30s heartbeat.
 *   At 30s heartbeat the device age can reach ~30s between packets
 *   during normal operation. A 30s threshold would cause false offline
 *   flickers. 60s gives 2x the heartbeat interval as margin.
 *
 * Power impact of change from 10s to 30s:
 *   10s cycle: ~19.93mA average -- ~6.3 days on 3000mAh LiPo
 *   30s cycle: ~6.6mA projected -- ~18-19 days on 3000mAh LiPo
 *   3x improvement from one constant change. */
#define DEEP_SLEEP_INTERVAL_US  (30ULL * 1000ULL * 1000ULL)

/* WDT timeout -- burst sleep kicks before AND mid-burst if >= this value */
#define WDT_TIMEOUT_MS        3000U

#define MFG_DATA_SIZE           8
#define MFG_COMPANY_ID_0     0xFF
#define MFG_COMPANY_ID_1     0xFF
#define MFG_MOTION_MSB_IDX      2
#define MFG_MOTION_B2_IDX       3
#define MFG_MOTION_B1_IDX       4
#define MFG_MOTION_LSB_IDX      5
#define MFG_BATT_IDX            6
#define MFG_OCCUPIED_IDX        7    /* 0=empty, 1=occupied */

static inline void pack_motion_count(uint8_t *mfg, uint32_t count)
{
    mfg[2] = (count >> 24) & 0xFF;
    mfg[3] = (count >> 16) & 0xFF;
    mfg[4] = (count >>  8) & 0xFF;
    mfg[5] =  count        & 0xFF;
}

static inline uint32_t unpack_motion_count(const uint8_t *mfg)
{
    return ((uint32_t)mfg[2] << 24) |
           ((uint32_t)mfg[3] << 16) |
           ((uint32_t)mfg[4] <<  8) |
            (uint32_t)mfg[5];
}

#endif /* MAIN_H */
