#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define BATT_UPDATE_TICKS     150
#define STATS_INTERVAL_TICKS   30
#define LOOP_SLEEP_MS        2000
#define MFG_DATA_SIZE           7
#define MFG_COMPANY_ID_0     0xFF
#define MFG_COMPANY_ID_1     0xFF
#define MFG_MOTION_MSB_IDX      2
#define MFG_MOTION_B2_IDX       3
#define MFG_MOTION_B1_IDX       4
#define MFG_MOTION_LSB_IDX      5
#define MFG_BATT_IDX            6
#define WDT_TIMEOUT_MS       3000

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
