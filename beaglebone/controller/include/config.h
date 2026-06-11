/******************************************************************************
 * \file config.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Compile-time configuration constants for BeagleBone data controller.
 *
 * \details Centralises all IPC paths, sizing limits, and buffer capacities
 *          used across the controller subsystem. No function prototypes,
 *          no type definitions — constants only.
 *
 * \note    MAX_DOORBELL_CAMS is also defined in shared_data.h for the
 *          LCD display process. Both definitions must remain identical.
 ******************************************************************************/
#ifndef INCLUDE_CONFIG_H_
#define INCLUDE_CONFIG_H_

/** \brief IPC and filesystem paths */
#define SHM_NAME           "/sensor_shm"
#define SENSOR_PIPE        "/tmp/sensor_pipe"
#define COMMAND_PIPE       "/tmp/controller_cmd"
#ifndef DB_PATH
#define DB_PATH            "/home/debian/db/sensors.db"
#endif
#define CONTROLLER_LOG     "/var/log/data_controller.log"
#define CONTROLLER_LOG_OLD "/var/log/data_controller.log.old"
#define UART_DEV           "/dev/ttyS1"

/** \brief Sizing constants */
#define MAX_ROOMS       10
#define MAX_REEDS        6
#define MAX_PIRS         5
#define MAX_TEMPS        4
#define MAX_DOORBELL_CAMS 4   /* must match ESP32 tcp_manager.c and shared_data.h */
#define MAX_CAMS          3   /* must match ESP32 udp_device_ingress.c */
#define TEMP_NAME_LEN   32
#define UART_LINE_LEN   64

/** \brief Ring buffer capacity — must be a power of two for mask wrapping */
#define UART_RING_SIZE  16

/** \brief Room and reed field sizes */
#define ROOM_NAME_SIZE  32
#define ROOM_STATE_SIZE 16
#define ROOM_LOC_SIZE   32
#define REED_NAME_SIZE  16

/** \brief Log rotation threshold */
#define LOG_MAX_BYTES   (5 * 1024 * 1024)

#endif /* INCLUDE_CONFIG_H_ */
