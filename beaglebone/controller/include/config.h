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
#define MAX_CAMS          3   /* must match ESP32 udp_device_ingress.c */
#define TEMP_NAME_LEN   32
#define UART_LINE_LEN   64
/** \brief UART outgoing UartMsg envelope buffer.
 *
 *  Worst-case payload budget (all slots active, uint64_t event_ids):
 *
 *    Fixed lines (STATE/PIR/PIR_COUNT/LGT/LCK/MTR/REED_COUNT/
 *                 TEMP_COUNT/DOORBELL full form):       ~160 bytes
 *    Per-slot lines (PIR×5, OCC×5, DR×6, TEMP×4,
 *                    CAM×3, DB×4):                      ~450 bytes
 *    EID_* lines (PIR×5, REED×6, TEMP×4, LOCK, LIGHT): ~521 bytes
 *                                               Total: ~1131 bytes
 *
 *  1280 covers the worst case with ~13% headroom and stays well under
 *  one UART frame at 115200 baud (~111 ms, inside the 500 ms rate limit).
 *  Previous value of 256 was a ~4× undercount introduced before
 *  per-device EID_* lines were added (2026-06-19).
 */
#define UART_MSG_BUF_SIZE 1280
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
