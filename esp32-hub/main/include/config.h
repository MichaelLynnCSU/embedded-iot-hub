/******************************************************************************
 * \file config.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Shared configuration constants for ESP32 hub node.
 *
 * \details Central configuration header included by all hub modules.
 *          Contains event group bits, peripheral configuration, timing
 *          constants, task stack sizes, default values, and shared
 *          room sensor type definition.
 *
 * \note    PI controller defaults (2026-05-04):
 *          DEFAULT_AWS_LOW, DEFAULT_AWS_HIGH, DEFAULT_AWS_MOTOR removed.
 *          Replaced with DEFAULT_AWS_KP/KI/KD/SETPOINT consumed by
 *          device_gateway.c and forwarded to tcp_manager.c via getters.
 *          PWM_DUTY_MAX added — maximum duty count sent to C3 motor node.
 *
 * \note    PIR slot table (2026-05-20):
 *          MAX_PIRS, PIR_NAME_PREFIX, PIR_NAME_PREFIX_LEN, PIR_OFFLINE_S
 *          added to support dynamic multi-PIR slot table in ble_scan.c.
 *          Mirrors the reed sensor pattern.
 *
 * \note    PIR/temp slot timing (2026-06-02):
 *          PIR_OFFLINE_MS, PIR_REMOVE_MS added — mirror reed thresholds.
 *          MAX_TEMPS, TEMP_NAME_PREFIX, TEMP_NAME_PREFIX_LEN,
 *          TEMP_OFFLINE_MS, TEMP_REMOVE_MS added for ble_temp.c slot
 *          expiry and tcp_manager.c offline flag.
 *
 * \warning AWS_LAMBDA_URL and network IPs are defined in network_config.h
 *          which is gitignored. Copy network_config.h.template to
 *          network_config.h and fill in values before building.
 ******************************************************************************/

#ifndef INCLUDE_CONFIG_H_
#define INCLUDE_CONFIG_H_

#include <stdint.h>
#include <inttypes.h>

/******************************** LOGGGING ***********************************/
// Clean formatting wrappers for Phase 4 Traceability
#define PRI_EVENT_ID PRIx64

#define LOG_TCP_TRACE(evt_id, bytes) \
    ESP_LOGI("TCP", "[TCP] event_id=%016" PRI_EVENT_ID " publish bytes=%d", (evt_id), (bytes))

#define LOG_UART_TRACE(evt_id, f_seq) \
    ESP_LOGI("UART", "[UART] event_id=%016" PRI_EVENT_ID " tx frame_seq=%u", (evt_id), (f_seq))

/******************************** CONSTANTS ***********************************/

/** \brief Doorbell camera heartbeat liveness */
#define MAX_DOORBELL_CAMS           4u       /**< max concurrent doorbell cams        */
#define MAX_CAMS                    3u       /**< number of inference cameras         */
#define DOORBELL_OFFLINE_S          90u      /**< offline threshold seconds (3x 30s)  */
#define DOORBELL_HEARTBEAT_MS       90000u   /**< offline threshold ms                */

/** \brief WiFi event group bits */
#define WIFI_CONNECTED_BIT          BIT0  /**< WiFi connected successfully */
#define WIFI_FAIL_BIT               BIT1  /**< WiFi connection failed */

/** \brief System event group bits */
#define ALL_TASKS_CREATED_BIT       BIT0  /**< all FreeRTOS tasks spawned */

/** \brief UART configuration — STM32 blue pill connection */
#define UART_STM32                  UART_NUM_2  /**< UART peripheral number */
#define STM32_TX_PIN                17          /**< ESP32 TX -> STM32 RX */
#define STM32_RX_PIN                16          /**< ESP32 RX <- STM32 TX */
#define UART_BUF_SIZE               256         /**< UART ring buffer size bytes */

/** \brief BLE internal event bits */
#define BIT_CONNECT_LIGHT           (1 << 0)  /**< light connection requested */
#define BIT_RESTART_SCAN            (1 << 1)  /**< scan restart requested */

/** \brief LightNF GATT service and characteristic UUIDs */
#define LIGHT_SERVICE_UUID          0xABCD  /**< light GATT service UUID */
#define LIGHT_CHAR_UUID             0xAB01  /**< light control char UUID */

/** \brief BLE scan parameters */
#define BLE_SCAN_INTERVAL           0x50                    /**< scan interval ~50ms */
#define BLE_SCAN_WINDOW             0x30                    /**< scan window ~30ms */
#define BLE_SCAN_DUPLICATE          BLE_SCAN_DUPLICATE_DISABLE /**< no dedup */

/** \brief BLE connection timing */
#define LIGHT_CONNECT_TIMEOUT_MS    15000  /**< light GATT connect timeout ms */

/** \brief Task startup delay constants */
#define WIFI_INIT_DELAY_MS          1000  /**< delay before WiFi init ms */
#define BLE_INIT_DELAY_MS           4000  /**< delay before BLE init ms */
#define TASK_CREATION_DELAY_MS      50    /**< delay between task spawns ms */
#define ADV_TIMER_DURATION_MS       3000  /**< advertisement stop timer ms */

/** \brief Periodic send intervals */
#define TCP_SEND_INTERVAL_MS        1000    /**< TCP send to BeagleBone ms */
#define AWS_SEND_INTERVAL_MS        300000  /**< AWS Lambda send interval ms */

/** \brief Reconnection timing */
#define RECONNECT_DELAY_MS          2000  /**< delay before reconnect attempt ms */
#define CONNECTION_RETRY_DELAY_MS   2000  /**< GATT connection retry delay ms */

/** \brief WiFi retry backoff table */
#define WIFI_BACKOFF_TABLE_SIZE     5  /**< number of backoff entries */

/** \brief FreeRTOS task stack sizes in bytes */
#define STACK_SIZE_WIFI_INIT        6144  /**< WiFi init task stack */
#define STACK_SIZE_BLE_INIT         6144  /**< BLE init task stack */
#define STACK_SIZE_UART_RX          4096  /**< UART receive task stack */
#define STACK_SIZE_TCP_SEND         4096  /**< TCP send task stack */
#define STACK_SIZE_AWS_SEND         6144  /**< AWS send task stack */

/** \brief Default sensor values on startup */
#define DEFAULT_AVG_TEMP            25     /**< default average temperature C */
#define DEFAULT_MOTION_COUNT        0      /**< default PIR motion count */

/** \brief PI controller defaults — held until first Lambda response arrives */
#define DEFAULT_AWS_KP              1.0f   /**< proportional gain */
#define DEFAULT_AWS_KI              0.05f  /**< integral gain */
#define DEFAULT_AWS_KD              0.0f   /**< derivative gain (PID-ready) */
#define DEFAULT_AWS_SETPOINT        25     /**< target temperature degrees C */

/** \brief PWM duty count ceiling sent to C3 motor node */
#define PWM_OUT_MAX                 100.0f /**< PI output ceiling (percent) */
#define PWM_OUT_MIN                 0.0f   /**< PI output floor (percent) */
#define PWM_DUTY_MAX                1023   /**< 10-bit PWM full scale */

/** \brief HTTP response buffer size */
#define HTTP_RESPONSE_BUFFER_SIZE   512  /**< AWS Lambda response buffer bytes */

/** \brief Room sensor count */
#define ROOM_COUNT                  2  /**< number of room sensors in rooms[] */

/** \brief PIR sliding window occupancy */
#define PIR_WINDOW_SEC        60u   /**< sliding window width seconds */
#define PIR_WINDOW_THRESHOLD  2u    /**< occ=1 events in window to declare occupied */
#define PIR_HOLD_SEC          600u  /**< hold occupied seconds after last trigger */

/** \brief PIR sensor slot table */
#define MAX_PIRS             5u            /**< max concurrent PIR sensors          */
#define PIR_NAME_PREFIX      "PIR_Motion"  /**< PIR device name prefix              */
#define PIR_NAME_PREFIX_LEN  10u           /**< length of PIR_NAME_PREFIX           */
#define PIR_OFFLINE_S        300           /**< PIR offline threshold seconds       */

/** \brief PIR slot state machine timing */
#define PIR_OFFLINE_MS       150000u       /**< ACTIVE->OFFLINE threshold ms        */
#define PIR_REMOVE_MS        3600000u      /**< OFFLINE->EMPTY threshold ms         */

/** \brief Reed sensor slot table configuration */
#define MAX_REEDS               6u            /**< max concurrent Reed sensors          */
#define REED_NAME_PREFIX        "ReedSensor"  /**< Reed device name prefix              */
#define REED_NAME_PREFIX_LEN    10u           /**< length of REED_NAME_PREFIX           */
#define REED_OFFLINE_S          150           /**< reed offline threshold seconds       */

/** \brief Temperature sensor slot table */
#define MAX_TEMPS               4u              /**< max concurrent temp sensors          */
#define TEMP_NAME_PREFIX        "TempSensor"    /**< temp device name prefix              */
#define TEMP_NAME_PREFIX_LEN    10u             /**< length of TEMP_NAME_PREFIX           */
#define TEMP_OFFLINE_MS         150000u         /**< ACTIVE->OFFLINE threshold ms         */
#define TEMP_REMOVE_MS          3600000u        /**< OFFLINE->EMPTY threshold ms          */

/** \brief Global BLE Allocation and Table Sizing */
#define ADV_NAME_BUF_SIZE       32            /**< Buffer size for parsing BLE names    */
#define SLOT_NAME_MAX           32            /**< Max string size for sensor identity  */
#define COOLDOWN_COUNT          8             /**< Allocation cooldown slot pool size   */

/** \brief Allocation State Machine Timing Parameters */
#define BLE_COOLDOWN_MS         5000          /**< Prevention timeframe for bounce loops*/
#define REED_OFFLINE_MS         150000        /**< Time before Reed sets to offline     */
#define REED_REMOVE_MS          3600000       /**< Time before old slots are fully flushed*/

/** \brief Logging and Throttling Limits */
#define REED_AGE_LOG_THRESHOLD  10
#define LOCK_AGE_LOG_THRESHOLD  10

/** \brief Room Mapping Array Index References */
#define ROOM_SENSOR_SLOT0_ID    0
#define ROOM_SENSOR_SLOT1_ID    1

/************************ STRUCTURE/UNION DATA TYPES **************************/

/**
 * \brief Room sensor descriptor — shared across hub modules.
 *
 * \details Populated at startup in hello_uart.c. Read by tcp_manager.c
 *          and device_gateway.c to build JSON payloads.
 */
typedef struct
{
   int        sensor_id; /*!< unique sensor identifier */
   const char *room;     /*!< room name string */
   const char *state;    /*!< current state string e.g. "open", "closed" */
   const char *location; /*!< physical location string */
} ROOM_SENSOR_T;

/********************************** MACROS ************************************/

/**
 * \brief WiFi retry backoff table — seconds between retry attempts.
 *
 * \details Defined as static const to avoid multiple definition errors
 *          when included in multiple translation units. Indexed 0..4.
 */
static const int wifi_backoff_sec[WIFI_BACKOFF_TABLE_SIZE] =
{
   2, 5, 10, 30, 60
};

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Shared room sensor array — defined in hello_uart.c */
extern ROOM_SENSOR_T rooms[];

#endif /* INCLUDE_CONFIG_H_ */
