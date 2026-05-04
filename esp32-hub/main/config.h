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
 * \warning AWS_LAMBDA_URL and network IPs are defined in network_config.h
 *          which is gitignored. Copy network_config.h.template to
 *          network_config.h and fill in values before building.
 ******************************************************************************/

#ifndef INCLUDE_CONFIG_H_
#define INCLUDE_CONFIG_H_

#include <stdint.h>

/******************************** CONSTANTS ***********************************/

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
#define STACK_SIZE_UART_RX          2048  /**< UART receive task stack */
#define STACK_SIZE_TCP_SEND         3072  /**< TCP send task stack */
#define STACK_SIZE_AWS_SEND         6144  /**< AWS send task stack */

/** \brief Default sensor values on startup */
#define DEFAULT_AVG_TEMP            25  /**< default average temperature C */
#define DEFAULT_AWS_LOW             20  /**< default motor low threshold C */
#define DEFAULT_AWS_HIGH            35  /**< default motor high threshold C */
#define DEFAULT_AWS_MOTOR           0   /**< default motor control mode */
#define DEFAULT_MOTION_COUNT        0   /**< default PIR motion count */

/** \brief HTTP response buffer size */
#define HTTP_RESPONSE_BUFFER_SIZE   512  /**< AWS Lambda response buffer bytes */

/** \brief Room sensor count */
#define ROOM_COUNT                  2  /**< number of room sensors in rooms[] */

/** \brief PIR sliding window occupancy */
#define PIR_WINDOW_SEC        60u   /**< sliding window width seconds */
#define PIR_WINDOW_THRESHOLD  2u    /**< occ=1 events in window to declare occupied */
#define PIR_HOLD_SEC          600u  /**< hold occupied seconds after last trigger */
/************************ STRUCTURE/UNION DATA TYPES **************************/

/**
 * \brief Room sensor descriptor — shared across hub modules.
 *
 * \details Populated at startup in hello_uart.c. Read by tcp_manager.c
 *          and aws_manager.c to build JSON payloads.
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
