#ifndef STUB_TRINITY_LOG_H
#define STUB_TRINITY_LOG_H
/* Stub: trinity_log.h -- no ESP-IDF dependency for unit test build */
#include <stdint.h>
#define TRINITY_CANARY_ALIVE   0xCA11AB1EU
#define TRINITY_CANARY_BOOTED  0xB007ED00U
static inline void trinity_log_init(void)            {}
static inline void trinity_log_dump_previous(void)   {}
static inline void trinity_log_erase(void)           {}
static inline void trinity_log_event(const char *p)  { (void)p; }
static inline void trinity_wdt_init(void)            {}
static inline void trinity_wdt_add(void)             {}
static inline void trinity_wdt_kick(void)            {}
static inline void trinity_log_heap_stats(void)      {}
static inline void trinity_log_task_stats(void)      {}
static inline void trinity_build_info_print(void)    {}
static inline void trinity_canary_set_booted(void)   {}
#endif /* STUB_TRINITY_LOG_H */
