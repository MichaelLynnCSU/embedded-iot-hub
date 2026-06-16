#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define BATT_UPDATE_TICKS    150   /**< poll ticks between battery reads   */
#define STATS_INTERVAL_TICKS  30   /**< poll ticks between stat dumps      */

/*
 * MFG data layout (6 bytes):
 * [0] = MFG_COMPANY_ID  (0xAE)
 * [1] = temp_decidegc low byte   (int16 little-endian)
 * [2] = temp_decidegc high byte
 * [3] = batt_soc                 (0-100%)
 * [4] = tx_id low byte           (little-endian, per-wake-session counter)
 * [5] = tx_id high byte          (little-endian, per-wake-session counter)
 *
 * tx_id is a per-session counter (RAM only, resets each boot).
 * Correlation key is (MAC + tx_id) within a bounded time window.
 * It is NOT a global unique message ID -- do not use for dedup or audit.
 *
 * Graceful degradation: hub checks mfg_len >= 6 before reading [4..5].
 * Old firmware (4-byte payload) produces tx_id=0 on the hub -- no crash.
 */
#define MFG_DATA_SIZE        6     /**< BLE manufacturer data payload bytes */
#define MFG_COMPANY_ID    0xAE    /**< BLE company ID for temp sensors     */
#define MFG_TEMP_LO_IDX     1     /**< temp low byte index                 */
#define MFG_TEMP_HI_IDX     2     /**< temp high byte index                */
#define MFG_BATT_IDX        3     /**< battery SOC byte index              */
#define MFG_TX_ID_LO_IDX    4     /**< tx_id low byte index                */
#define MFG_TX_ID_HI_IDX    5     /**< tx_id high byte index               */

#define TEMP_POLL_MS        2000   /**< main loop poll interval ms         */

#endif /* MAIN_H */
