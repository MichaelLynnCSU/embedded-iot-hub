/******************************************************************************
 * \file    config.h
 * \brief   Shared compile-time configuration for the smart-lock node.
 *
 * \details Single authoritative source for timing constants and BLE
 *          manufacturer-data layout used by both main.c and ble_gatt.c.
 *          Include this header instead of defining these macros locally.
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

/* ── Timing ────────────────────────────────────────────────────────────────── */
#define STATS_INTERVAL_SEC  60   /**< Statistics report interval (seconds)    */
#define IDLE_HEARTBEAT_SEC  240  /**< Idle heartbeat advertisement (seconds)  */
#define BATT_UPDATE_SEC     300  /**< Battery SOC refresh interval (seconds)  */

/* ── BLE manufacturer data layout ─────────────────────────────────────────── */
#define MFG_DATA_SIZE       4    /**< Total manufacturer data bytes           */
#define MFG_COMPANY_ID      0xAC /**< Registered company ID byte              */
#define MFG_LOCK_STATE_IDX  1    /**< Lock state byte index in mfg data       */
#define MFG_BATT_IDX        2    /**< Battery SOC byte index in mfg data      */
#define MFG_HEARTBEAT_IDX   3    /**< Heartbeat counter byte index in mfg data.
                                   *   Incremented on every heartbeat tick so
                                   *   a passive scanner (hub or app) can see
                                   *   liveness without a GATT connection.   */

/*
 * GATT write payload layout (3 bytes):
 * [0] = state      (0=unlock, 1=lock)
 * [1] = tx_id low byte  (little-endian)
 * [2] = tx_id high byte (little-endian)
 *
 * tx_id is a per-session command counter (RAM only on hub, resets on hub
 * reboot). Correlation key is (MAC + tx_id) within a bounded time window.
 * It is NOT a global unique message ID.
 *
 * Two-phase identity:
 *   tx_id    — generated BEFORE write (command tracking, crosses boundary)
 *   event_id — generated AFTER bus_publish_lock() (hub-internal only)
 *
 * Device logs tx_id only. Hub logs tx_id + event_id mapping.
 * If tx_id exists but no event_id appears in hub log → bus failure.
 * If event_id exists but device did not log tx_id → device failure.
 */
#define LOCK_WRITE_LEN          3
#define LOCK_WRITE_STATE_IDX    0   /**< state byte index in write payload    */
#define LOCK_WRITE_TX_ID_LO_IDX 1   /**< tx_id low byte index                */
#define LOCK_WRITE_TX_ID_HI_IDX 2   /**< tx_id high byte index               */

#endif /* CONFIG_H */
