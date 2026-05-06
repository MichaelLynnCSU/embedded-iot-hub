/******************************************************************************
 * \file wifi_tx_sweep.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief WiFi TX power sweep for ESP32-C3 motor controller node.
 ******************************************************************************/
#ifndef WIFI_TX_SWEEP_H
#define WIFI_TX_SWEEP_H

typedef struct
{
    int tx_power;    /**< Current TX power in quarter-dBm units */
    int start_power; /**< Initial TX power -- reset target       */
    int step;        /**< Power increment per disconnect         */
    int max_power;   /**< Maximum TX power ceiling               */
} WIFI_TX_SWEEP_T;

void wifi_tx_sweep_init(WIFI_TX_SWEEP_T *p_sweep,
                        int start_power,
                        int step,
                        int max_power);

void wifi_tx_sweep_on_disconnect(WIFI_TX_SWEEP_T *p_sweep);

void wifi_tx_sweep_reset(WIFI_TX_SWEEP_T *p_sweep);

#endif /* WIFI_TX_SWEEP_H */
