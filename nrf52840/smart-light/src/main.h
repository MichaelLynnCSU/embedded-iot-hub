#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define STATS_INTERVAL_SEC   60
#define IDLE_HEARTBEAT_SEC   240
#define RELAY_PIN            11
#define MFG_COMPANY_ID       0xAD
#define MFG_DATA_SIZE        3
#define MFG_STATE_IDX        1
#define MFG_HEARTBEAT_IDX    2   /**< heartbeat counter, mirrors smart-lock */

/*
 * GATT write payload layout (3 bytes):
 * [0] = state    (0=OFF, 1=ON)
 * [1] = tx_id low byte  (little-endian)
 * [2] = tx_id high byte (little-endian)
 *
 * tx_id is a per-session command counter (RAM only on hub, resets on hub
 * reboot). Correlation key is (MAC + tx_id) within a bounded time window.
 * It is NOT a global unique message ID.
 *
 * Two-phase identity:
 *   tx_id    — generated BEFORE write (command tracking, crosses boundary)
 *   event_id — generated AFTER bus_publish_light() (hub-internal only)
 *
 * Device logs tx_id only. Hub logs tx_id + event_id mapping.
 * If tx_id exists but no event_id appears in hub log → bus failure.
 * If event_id exists but device did not log tx_id → device failure.
 */
#define LIGHT_WRITE_LEN          3
#define LIGHT_WRITE_STATE_IDX    0   /* state byte index                  */
#define LIGHT_WRITE_TX_ID_LO_IDX 1   /* tx_id low byte index              */
#define LIGHT_WRITE_TX_ID_HI_IDX 2   /* tx_id high byte index             */
#define LIGHT_STATE_MAX          1

/**
 * \brief  Drive the relay and status LED to the requested state.
 *         Implemented in main.c, called by ble_gatt.c on GATT write.
 *
 * \param  state  0=OFF, 1=ON.
 */
void relay_set(uint8_t state);

#endif /* MAIN_H */
