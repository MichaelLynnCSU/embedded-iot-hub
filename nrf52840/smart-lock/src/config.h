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
#define MFG_DATA_SIZE       3    /**< Total manufacturer data bytes           */
#define MFG_COMPANY_ID      0xAC /**< Registered company ID byte              */
#define MFG_LOCK_STATE_IDX  1    /**< Lock state byte index in mfg data       */
#define MFG_BATT_IDX        2    /**< Battery SOC byte index in mfg data      */

/* ── GATT write payload ────────────────────────────────────────────────────── */
#define LOCK_WRITE_LEN      1    /**< GATT write length for lock control char */

#endif /* CONFIG_H */
