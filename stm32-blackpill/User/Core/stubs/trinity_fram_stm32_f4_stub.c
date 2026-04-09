#include "trinity_log.h"
#include <string.h>
/* Stub: UART only, no FRAM writes */
void trinity_uart_log(const char *p_msg)     { (void)p_msg; }
void trinity_rtc_store(TRINITY_ERROR_E err)  { (void)err; }
void trinity_log_init(void)                  {}
void trinity_log_dump_previous(void)         {}
void trinity_log_event(const char *p_msg)    { trinity_uart_log(p_msg); }
void trinity_log_erase(void)                 {}
