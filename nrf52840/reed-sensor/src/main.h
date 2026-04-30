#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define BATT_UPDATE_TICKS    150   /**< reed poll ticks between batt reads  */
#define STATS_INTERVAL_TICKS  30   /**< reed poll ticks between stat dumps  */
#define MFG_DATA_SIZE          3   /**< BLE manufacturer data payload bytes */
#define MFG_COMPANY_ID      0xAB   /**< BLE company ID for reed sensors     */
#define REED_POLL_MS        2000   /**< main loop poll interval ms          */

#endif /* MAIN_H */
