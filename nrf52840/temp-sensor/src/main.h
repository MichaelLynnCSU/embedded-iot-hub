#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define BATT_UPDATE_TICKS    150   /**< poll ticks between battery reads   */
#define STATS_INTERVAL_TICKS  30   /**< poll ticks between stat dumps      */
#define MFG_DATA_SIZE          4   /**< BLE manufacturer data payload bytes:
                                        [0] company ID
                                        [1] temp_decidegc low byte
                                        [2] temp_decidegc high byte
                                        [3] batt_soc                       */
#define MFG_COMPANY_ID      0xAE   /**< BLE company ID for temp sensors    */
#define TEMP_POLL_MS        2000   /**< main loop poll interval ms         */

#endif /* MAIN_H */
