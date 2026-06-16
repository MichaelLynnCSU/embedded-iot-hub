#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
#define BATT_UPDATE_TICKS    150   /**< reed poll ticks between batt reads  */
#define STATS_INTERVAL_TICKS  30   /**< reed poll ticks between stat dumps  */
#define MFG_DATA_SIZE          5   /**< BLE manufacturer data payload bytes
                                    *   [0] = MFG_COMPANY_ID  (0xAB)
                                    *   [1] = door state      (0=closed, 1=open)
                                    *   [2] = batt_soc        (0-100%)
                                    *   [3] = tx_id low byte
                                    *   [4] = tx_id high byte
                                    *
                                    *   tx_id is a uint16_t sequence counter
                                    *   stamped by the device on every broadcast.
                                    *   The hub reads it out of the raw mfg payload
                                    *   and logs it as tx_id= at BLE ingress so
                                    *   device-side and hub-side logs share the same
                                    *   correlation key.
                                    *
                                    *   Graceful degradation: hubs receiving a 3-byte
                                    *   payload from an older firmware default tx_id=0
                                    *   and continue operating normally.
                                    *
                                    *   Template note: when adding tx_id to a new
                                    *   device type, append tx_id_lo and tx_id_hi at
                                    *   the END of the existing payload so old hub
                                    *   firmware that checks mfg_len < 5 degrades
                                    *   cleanly. Never insert tx_id bytes in the
                                    *   middle of an existing layout.           */
#define MFG_COMPANY_ID      0xAB   /**< BLE company ID for reed sensors     */
#define REED_POLL_MS        2000   /**< main loop poll interval ms          */
#endif /* MAIN_H */
