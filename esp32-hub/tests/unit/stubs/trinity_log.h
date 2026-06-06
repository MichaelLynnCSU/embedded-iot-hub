#ifndef STUB_TRINITY_LOG_H
#define STUB_TRINITY_LOG_H
#include <stdint.h>
void trinity_log_event(const char *p);
void trinity_wdt_kick(void);
void trinity_wdt_add(void);
#endif
