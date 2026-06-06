#ifndef STUB_DRIVER_UART_H
#define STUB_DRIVER_UART_H

#include <stdint.h>
#include <stddef.h>

#define UART_NUM_2          2
#define UART_PIN_NO_CHANGE  -1

typedef int uart_port_t;
typedef int uart_parity_t;
typedef int uart_stop_bits_t;
typedef int uart_word_length_t;
typedef int uart_hw_flowcontrol_t;

typedef struct {
    int                  baud_rate;
    uart_word_length_t   data_bits;
    uart_parity_t        parity;
    uart_stop_bits_t     stop_bits;
    uart_hw_flowcontrol_t flow_ctrl;
} uart_config_t;

#define UART_DATA_8_BITS   0
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1   0
#define UART_HW_FLOWCTRL_DISABLE 0

static inline int uart_param_config(uart_port_t p, const uart_config_t *c)
{ (void)p;(void)c; return 0; }
static inline int uart_set_pin(uart_port_t p, int tx, int rx, int rts, int cts)
{ (void)p;(void)tx;(void)rx;(void)rts;(void)cts; return 0; }
static inline int uart_driver_install(uart_port_t p, int rb, int tb, int qz, void *q, int f)
{ (void)p;(void)rb;(void)tb;(void)qz;(void)q;(void)f; return 0; }
static inline int uart_read_bytes(uart_port_t p, void *buf, size_t len, uint32_t ticks)
{ (void)p;(void)buf;(void)len;(void)ticks; return 0; }

#endif /* STUB_DRIVER_UART_H */
