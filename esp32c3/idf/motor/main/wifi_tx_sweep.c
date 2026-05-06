/******************************************************************************
 * \file wifi_tx_sweep.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief WiFi TX power sweep for ESP32-C3 motor controller node.
 *
 * \details Implements a stepped TX power increase strategy on WiFi
 *          disconnection. Starts at a low TX power and increments by
 *          step on each disconnect until max_power is reached.
 *
 *          Power units are ESP-IDF quarter-dBm units (value * 0.25 = dBm).
 *          Example: 34 = 8.5 dBm, 84 = 21.0 dBm.
 ******************************************************************************/
#include "wifi_tx_sweep.h"
#include "esp_wifi.h"
#include "esp_log.h"

static const char *TAG = "TX_SWEEP";

/******************************************************************************
 * \brief Initialize WiFi TX power sweep state.
 ******************************************************************************/
void wifi_tx_sweep_init(WIFI_TX_SWEEP_T *p_sweep,
                        int start_power,
                        int step,
                        int max_power)
{
    if (NULL == p_sweep) { return; }

    p_sweep->tx_power    = start_power;
    p_sweep->start_power = start_power;
    p_sweep->step        = step;
    p_sweep->max_power   = max_power;

    (void)esp_wifi_set_max_tx_power((int8_t)p_sweep->tx_power);
    ESP_LOGI(TAG, "TX sweep init: %d (%.2f dBm)",
             p_sweep->tx_power, p_sweep->tx_power * 0.25f);
}

/******************************************************************************
 * \brief Increment TX power on WiFi disconnection.
 ******************************************************************************/
void wifi_tx_sweep_on_disconnect(WIFI_TX_SWEEP_T *p_sweep)
{
    if (NULL == p_sweep) { return; }

    ESP_LOGW(TAG, "WiFi disconnected, TX=%d (%.2f dBm)",
             p_sweep->tx_power, p_sweep->tx_power * 0.25f);

    if (p_sweep->tx_power < p_sweep->max_power)
    {
        p_sweep->tx_power += p_sweep->step;
        if (p_sweep->tx_power > p_sweep->max_power)
        {
            p_sweep->tx_power = p_sweep->max_power;
        }
        (void)esp_wifi_set_max_tx_power((int8_t)p_sweep->tx_power);
        ESP_LOGI(TAG, "TX power increased to %d (%.2f dBm)",
                 p_sweep->tx_power, p_sweep->tx_power * 0.25f);
    }
    else
    {
        ESP_LOGW(TAG, "TX power already at max (%d / %.2f dBm)",
                 p_sweep->tx_power, p_sweep->tx_power * 0.25f);
    }
}

/******************************************************************************
 * \brief Reset TX power to start_power on successful reconnect.
 *
 * \param p_sweep - Pointer to sweep state struct.
 *
 * \return void
 *
 * \details Called from the GOT_IP event handler after a successful
 *          reconnect. Resets tx_power to start_power and applies it
 *          via esp_wifi_set_max_tx_power() so the sweep begins fresh
 *          on the next disconnect rather than staying at max.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void wifi_tx_sweep_reset(WIFI_TX_SWEEP_T *p_sweep)
{
    if (NULL == p_sweep) { return; }

    if (p_sweep->tx_power == p_sweep->start_power) { return; }

    p_sweep->tx_power = p_sweep->start_power;
    (void)esp_wifi_set_max_tx_power((int8_t)p_sweep->tx_power);
    ESP_LOGI(TAG, "TX sweep reset to %d (%.2f dBm)",
             p_sweep->tx_power, p_sweep->tx_power * 0.25f);
}
