/******************************************************************************
 * \file wifi_tx_sweep.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief WiFi TX power sweep interface for ESP32-C3 motor controller node.
 *
 * \details Provides stepped TX power increase on WiFi disconnection.
 *          Power units are ESP-IDF quarter-dBm (value * 0.25 = dBm).
 *          See wifi_tx_sweep.c for implementation details.
 ******************************************************************************/

#ifndef INCLUDE_WIFI_TX_SWEEP_H_
#define INCLUDE_WIFI_TX_SWEEP_H_

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief WiFi TX power sweep state. */
typedef struct
{
   int tx_power;  /*!< current TX power in quarter-dBm units */
   int step;      /*!< power increment per disconnect in quarter-dBm units */
   int max_power; /*!< maximum allowed TX power in quarter-dBm units */
} WIFI_TX_SWEEP_T;

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize WiFi TX power sweep state.
 *  \param p_sweep     - Pointer to sweep state struct.
 *  \param start_power - Initial TX power in quarter-dBm units.
 *  \param step        - Power increment per disconnect in quarter-dBm units.
 *  \param max_power   - Maximum TX power in quarter-dBm units.
 *  \return void */
void wifi_tx_sweep_init(WIFI_TX_SWEEP_T *p_sweep,
                        int start_power,
                        int step,
                        int max_power);

/** \brief Increment TX power on WiFi disconnection.
 *  \param p_sweep - Pointer to sweep state struct.
 *  \return void */
void wifi_tx_sweep_on_disconnect(WIFI_TX_SWEEP_T *p_sweep);

#endif /* INCLUDE_WIFI_TX_SWEEP_H_ */
