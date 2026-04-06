#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define STATS_INTERVAL_SEC   60
#define IDLE_HEARTBEAT_SEC   120
#define BATT_UPDATE_SEC      300
#define MFG_COMPANY_ID       0xAC
#define MFG_DATA_SIZE        3
#define MFG_LOCK_STATE_IDX   1
#define MFG_BATT_IDX         2
#define LOCK_WRITE_LEN       1
#define BATT_MV_MAX          6000
#define BATT_MV_MIN          4000
#define BATT_MV_RANGE        2000

static inline uint8_t mv_to_soc(int mv)
{
    if (mv >= BATT_MV_MAX) { return 100u; }
    if (mv <= BATT_MV_MIN) { return 0u;   }
    return (uint8_t)((mv - BATT_MV_MIN) * 100 / BATT_MV_RANGE);
}

#endif /* MAIN_H */
