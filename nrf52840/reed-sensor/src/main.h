#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define BATT_UPDATE_TICKS       150
#define STATS_INTERVAL_TICKS     30
#define MFG_DATA_SIZE             3
#define MFG_COMPANY_ID          0xAB
#define BATT_MV_MAX             3000
#define BATT_MV_MIN             2000
#define BATT_MV_RANGE           1000
#define REED_POLL_MS            2000

static inline uint8_t mv_to_soc(int mv)
{
    if (mv >= BATT_MV_MAX) { return 100u; }
    if (mv <= BATT_MV_MIN) { return 0u;   }
    return (uint8_t)((mv - BATT_MV_MIN) * 100 / BATT_MV_RANGE);
}

#endif /* MAIN_H */
