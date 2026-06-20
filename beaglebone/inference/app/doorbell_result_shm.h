/******************************************************************************
 * \file doorbell_result_shm.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief POSIX shared memory layout for doorbell inference result hand-off.
 *
 * \details Bridges doorbell_daemon (Lane B — TCP JPEG ingest + inference,
 *          port 9091) to uart_controller (STM32 UART push) WITHOUT touching
 *          SharedSensorData (shared_data.h) or SensorData (sensor_types.h).
 *          Those structs are duplicated byte-for-byte across the sensor
 *          server, controller, and ESP32 hub JSON — adding fields there
 *          would require recompiling/redeploying all three. This segment
 *          is new, small, and owned only by the two processes that need it.
 *
 *          Segment: /doorbell_result
 *          Writer:  doorbell_daemon — creates (shm_open O_CREAT), owns
 *                   lifecycle, publishes one result per completed doorbell
 *                   JPEG + inference run.
 *          Reader:  uart_controller — attaches read-write (shm_open, no
 *                   O_CREAT), consumes results and forwards person/conf/
 *                   asset fields to the STM32 over the existing UART
 *                   DOORBELL: line.
 *
 * \note    asset field semantics (2026-06-14):
 *          asset is NOT the saved filename (which is ~50-70 bytes:
 *          "<ts>_<token>_<tag>_<conf>.jpg" from inference_worker_save()).
 *          It is the "%Y%m%dT%H%M%SZ" timestamp string embedded in that
 *          filename (17 chars + nul = 18 bytes, fits in asset[20]),
 *          obtained via inference_worker_save()'s out_ts param — guaranteed
 *          to match the saved file exactly, no independent time(NULL) call.
 *          This keeps the UI layer (STM32 LCD) decoupled from the storage
 *          filename schema: event_id is the identity, asset is a short
 *          human-readable display hint, and the full filename/path is a
 *          storage-layer concern resolved server-side if ever needed.
 *
 * \note    Lock-free publish/consume (single writer, single reader):
 *          event_id is the change-detector. Writer writes all other
 *          fields first, issues a memory barrier, then publishes by
 *          setting event_id to the (non-zero, monotonic-per-device)
 *          value from cam_make_event_id(). Reader compares event_id
 *          against its own locally-held last-seen value; on change,
 *          issues a memory barrier, copies out the other fields, and
 *          updates its local last-seen value.
 *
 *          Nothing in the segment is ever cleared by the reader — this
 *          is what makes the "single reader" assumption safe to relax
 *          later (a second reader can keep its own last-seen event_id
 *          without disturbing the first). However, as written, only
 *          ONE reader is wired up (uart_controller). If a second
 *          consumer is added, give it its own local last-seen state;
 *          do not add any clearing/ready-flag logic to this segment.
 *
 *          event_id == 0 means "no doorbell event published yet" —
 *          cam_make_event_id() never produces 0, so this is a safe
 *          sentinel for the initial zeroed segment.
 *
 * \note    event_id tracing (2026-06-15):
 *          EVENT_ID_FMT / EVENT_ID_ARG below give doorbell_daemon and
 *          uart_controller a single canonical %08lx%08lx representation
 *          of event_id in their respective log files, so one event's
 *          full lifecycle (capture -> inference -> shm publish -> shm
 *          consume -> UART send) can be traced with:
 *              grep <event_id_hex> /var/log/doorbell_daemon.log \
 *                                  /var/log/data_controller.log
 *
 * \note    CAM/doorbell identity (2026-06-19):
 *          event_id here is the camera's own ID, assigned at capture time
 *          by cam_make_event_id() and relayed unchanged through
 *          doorbell_daemon — doorbell_daemon does NOT mint a separate
 *          "doorbell event" ID. There is one ID for one capture.
 *          doorbell_daemon is a consumer/processor of that capture, not
 *          an independent event source. uart_controller names its local
 *          copy db_event_id purely as a variable-naming convenience —
 *          it is the same identity as cam_header_t.event_id throughout.
 *
 *          The non-doorbell monitoring cameras (MAX_CAMS / cam_slots[])
 *          are a completely separate device class with NO event_id today:
 *          CamSlotData carries only age_s and online. They share no
 *          identity with this segment and are not involved in this
 *          publish/consume flow. Adding CAM event tracing is a separate
 *          task starting at cam_trigger.c on the ESP32 side.
 ******************************************************************************/

#ifndef DOORBELL_RESULT_SHM_H
#define DOORBELL_RESULT_SHM_H

#include <stdint.h>

#define DOORBELL_RESULT_SHM_NAME "/doorbell_result"

/**
 * \brief Shared formatting helpers for event_id tracing.
 *
 * \details Usage:
 *              LOG("... event_id=" EVENT_ID_FMT " ...", EVENT_ID_ARG(id), ...);
 *
 *          Renders a 64-bit event_id as 16 lowercase hex digits
 *          (high 32 bits, then low 32 bits), matching the format already
 *          used by the ESP32 cams and Lane A UDP envelope.
 */
#define EVENT_ID_FMT "%08lx%08lx"
#define EVENT_ID_ARG(id) \
   (unsigned long)((uint64_t)(id) >> 32), \
   (unsigned long)((uint64_t)(id) & 0xFFFFFFFFUL)

/**
 * \brief Doorbell inference result — published by doorbell_daemon,
 *        consumed by uart_controller.
 */
struct DoorbellResult
{
   uint64_t event_id;     /*!< 0 = none yet; else the camera-assigned ID from
                            *   cam_header_t.event_id in the Lane B frame that
                            *   triggered this result.
                            *
                            *   doorbell_daemon relays this ID unchanged — it
                            *   does not mint a new one. One capture = one ID,
                            *   continuous from camera through inference through
                            *   UART send. uart_controller's local variable
                            *   db_event_id holds this same value.
                            *
                            *   Unrelated to MAX_CAMS/cam_slots[] monitoring
                            *   cameras, which have no event_id today.        */
   uint8_t  device_id;    /*!< doorbell cam ID (0-3)                          */
   uint8_t  person;       /*!< 1 = person detected, 0 = not                   */
   uint8_t  conf_pct;     /*!< confidence, 0-100, represents 0.00-1.00        */
   char     asset[20];    /*!< "%Y%m%dT%H%M%SZ" timestamp from
                            *   inference_worker_save() out_ts — NOT a
                            *   filename. See note above.               */
};

#endif /* DOORBELL_RESULT_SHM_H */
