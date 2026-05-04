#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define STATS_INTERVAL_SEC   60
#define IDLE_HEARTBEAT_SEC   240
#define RELAY_PIN            11
#define MFG_COMPANY_ID       0xAD
#define MFG_DATA_SIZE        2
#define MFG_STATE_IDX        1

/* write_light_control validation constants */
#define LIGHT_WRITE_LEN      1
#define LIGHT_STATE_MAX      1

/**
 * \brief  Drive the relay and status LED to the requested state.
 *         Implemented in main.c, called by ble_gatt.c on GATT write.
 *
 * \param  state  0=OFF, 1=ON.
 */
void relay_set(uint8_t state);

#endif /* MAIN_H */
