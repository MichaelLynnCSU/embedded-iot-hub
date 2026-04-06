/******************************************************************************
 * \file ble_internal.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BLE subsystem internal interface for ESP32 hub node.
 *
 * \details Private header shared between ble_gattc.c, ble_scan.c,
 *          ble_lock.c, and ble_light.c. Not for use outside the BLE
 *          subsystem.
 *
 *          Module ownership:
 *          - ble_gattc_if, g_gatt_registered : ble_gattc.c
 *          - connect_queue, g_connection_in_progress : ble_manager.c
 *          - lock_mac, light_mac, found flags  : ble_scan.c
 *
 *          Startup sequence (called by ble_manager.c):
 *          1. ble_gattc_init()  — registers GATT callback and app
 *          2. ble_scan_start()  — sets scan params, starts scanning
 ******************************************************************************/

#ifndef INCLUDE_BLE_INTERNAL_H_
#define INCLUDE_BLE_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/******************************** CONSTANTS ***********************************/

#define CONNECT_QUEUE_DEPTH    5     /**< max pending connection requests */
#define CONNECT_WATCHDOG_MS    8000  /**< connection progress timeout ms */

/******************************* ENUMERATIONS *********************************/

/**
 * \brief BLE device index for per-device age tracking.
 *
 * \details Used with stamp_device() and ble_get_device_age_s().
 */
typedef enum
{
   DEV_IDX_PIR   = 0, /**< PIR motion sensor */
   DEV_IDX_REED1 = 1, /**< reed sensor 1 */
   DEV_IDX_REED2 = 2, /**< reed sensor 2 */
   DEV_IDX_LIGHT = 3, /**< smart light relay */
   DEV_IDX_LOCK  = 4, /**< smart lock */
   DEV_IDX_COUNT = 5  /**< total device count — must be last */
} BLE_DEV_IDX_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief Connection scheduler request — posted to connect_queue. */
typedef struct
{
   void (*connect_fn)(void); /*!< function pointer to device connect fn */
} BLE_CONNECT_REQUEST_T;

/*************************** SHARED GATT INTERFACE ****************************/

/** Shared GATT interface handle — owned by ble_gattc.c */
extern esp_gatt_if_t g_ble_gattc_if;

/** GATT registration flag — owned by ble_gattc.c */
extern bool g_gatt_registered;

/************************* CONNECTION SCHEDULER *******************************/

/**
 * \brief Connection request queue — serialises all open requests.
 *
 * \details Depth CONNECT_QUEUE_DEPTH handles burst adv scenarios.
 *          Owned by ble_manager.c.
 */
extern QueueHandle_t connect_queue;

/**
 * \brief Radio-level connection semaphore flag.
 *
 * \details Set before esp_ble_gattc_open(), cleared on DISCONNECT_EVT
 *          or OPEN_EVT with non-OK status. Owned by ble_manager.c.
 */
extern volatile bool g_connection_in_progress;

/************************* DEVICE DISCOVERY STATE *****************************/

/** Lock device MAC address — owned by ble_scan.c */
extern uint8_t             lock_mac[6];

/** Lock device found flag — owned by ble_scan.c */
extern bool                lock_found;

/** Lock device address type — owned by ble_scan.c */
extern esp_ble_addr_type_t lock_addr_type;

/** Light device MAC address — owned by ble_scan.c */
extern uint8_t             light_mac[6];

/** Light device found flag — owned by ble_scan.c */
extern bool                light_found;

/** Light device address type — owned by ble_scan.c */
extern esp_ble_addr_type_t light_addr_type;

/*************************** FUNCTION PROTOTYPES ******************************/

/** \brief Notify connection scheduler that current connection attempt is done.
 *  \return void */
void ble_scheduler_notify_done(void);

/** \brief Stamp the last-seen time for a device.
 *  \param idx - Device index from BLE_DEV_IDX_E.
 *  \return void */
void stamp_device(BLE_DEV_IDX_E idx);

/** \brief Get seconds since device was last seen.
 *  \param idx - Device index.
 *  \return uint16_t - Age in seconds, 0xFFFF if never seen. */
uint16_t ble_get_device_age_s(int idx);

/** \brief Update lock state from advertisement data.
 *  \param state - Lock state byte from manufacturer data.
 *  \param batt  - Battery SOC percent.
 *  \return void */
void ble_lock_update_adv(uint8_t state, uint8_t batt);

/** \brief Update light state from advertisement data.
 *  \param state - Light state byte from manufacturer data.
 *  \return void */
void ble_light_update_adv(uint8_t state);

/** \brief Get last known lock battery SOC.
 *  \return int - Battery SOC percent. */
int ble_lock_get_batt(void);

/** \brief Get last known lock state.
 *  \return uint8_t - Lock state. */
uint8_t ble_lock_get_state(void);

/** \brief Get last known light state.
 *  \return uint8_t - Light state. */
uint8_t ble_light_get_state(void);

/** \brief Race-safe lock connection entry point from ble_scan.c.
 *  \return void */
void ble_lock_try_connect(void);

/** \brief Route GATT event to lock handler.
 *  \param event    - GATT client event type.
 *  \param gattc_if - GATT client interface handle.
 *  \param p_param  - Pointer to event parameter union.
 *  \return void */
void ble_lock_handle_event(esp_gattc_cb_event_t event,
                            esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *p_param);

/** \brief Race-safe light connection entry point from ble_scan.c.
 *  \return void */
void ble_light_try_connect(void);

/** \brief Route GATT event to light handler.
 *  \param event    - GATT client event type.
 *  \param gattc_if - GATT client interface handle.
 *  \param p_param  - Pointer to event parameter union.
 *  \return void */
void ble_light_handle_event(esp_gattc_cb_event_t event,
                             esp_gatt_if_t gattc_if,
                             esp_ble_gattc_cb_param_t *p_param);

/** \brief Initialize shared BLE GATT client.
 *  \return void */
void ble_gattc_init(void);

/** \brief Start BLE scanning.
 *  \return void */
void ble_scan_start(void);

#endif /* INCLUDE_BLE_INTERNAL_H_ */
