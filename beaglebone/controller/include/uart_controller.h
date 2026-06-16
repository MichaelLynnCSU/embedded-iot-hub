/******************************************************************************
 * \file uart_controller.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief Public UART API for BeagleBone data controller.
 *
 * \details Unchanged public surface after the 2026-06-15 module split.
 *          Implementations now live in:
 *            - uart_transport.c — uart_reader_thread, uart_ring_push,
 *                                  uart_ring_pop
 *            - uart_protocol.c  — uart_push_thread, uart_update_frame
 *            - uart_lock.c      — uart_sync_lock_state
 *                                  (uart_lock.h also declares
 *                                  uart_process_lock, internal-only)
 *            - doorbell_pending.c — doorbell correlation, internal-only
 *                                    (doorbell_pending.h)
 *
 *          g_uart_frame_sem and g_uart_ring remain defined in
 *          data_controller.c (or wherever they were previously defined)
 *          and are used by both uart_transport.c (producer) and
 *          uart_protocol.c (consumer).
 ******************************************************************************/
#ifndef INCLUDE_UART_CONTROLLER_H_
#define INCLUDE_UART_CONTROLLER_H_

#include <semaphore.h>
#include "sensor_types.h"

extern sem_t       g_uart_frame_sem;
extern uart_ring_t g_uart_ring;

void *uart_reader_thread   (void *p_arg);
void *uart_push_thread     (void *p_arg);
void  uart_sync_lock_state (int state);
void  uart_update_frame    (const struct LatestData *p_snapshot,
                             const struct SensorData *p_data);
void  uart_ring_push       (const UartFrame *p_frame);
int   uart_ring_pop        (UartFrame *p_frame);

#endif
