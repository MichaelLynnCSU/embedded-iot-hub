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
