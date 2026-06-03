#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

/* Reduce buffers to bare minimum to protect display RAM allocations */
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (1)
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           (1)
#define SEGGER_RTT_BUFFER_SIZE_UP                 (256)  /* Shrunk from 1024 */
#define SEGGER_RTT_BUFFER_SIZE_DOWN               (16)
#define SEGGER_RTT_PRINTF_BUFFER_SIZE             (64)
#define SEGGER_RTT_MODE_DEFAULT                   SEGGER_RTT_MODE_NO_BLOCK_SKIP

#define SEGGER_RTT_LOCK()
#define SEGGER_RTT_UNLOCK()

#endif
