#include "trinity_log.h"
#include <string.h>
uint32_t g_boot_count = 0ul;
void trinity_uart_log(const char *p)               { (void)p; }
void trinity_rtc_store(TRINITY_ERROR_E e, uint8_t b) { (void)e; (void)b; }
void trinity_log_init_ex(uint32_t r)               { (void)r; }
void trinity_log_init(void)                        {}
void trinity_log_dump_previous(void)               {}
void trinity_log_event(const char *p)              { trinity_uart_log(p); }
void trinity_log_erase(void)                       {}
