#include "trinity_log.h"
#include "trinity_private.h"
int  trinity_log_init(void)                       { return 0; }
void trinity_log_dump_previous(void)              {}
void trinity_log_dump_previous_deferred(void)     {}
int  trinity_log_erase(void)                      { return 0; }
void trinity_log_event(const char *p) { printk("[TRINITY] %s\n", p); }
void trinity_flash_lock(void)                     {}
void trinity_flash_unlock(void)                   {}
void write_panic(const char *p, uint16_t len)     { (void)p; (void)len; }
int  trinity_log_stats_init(void)                 { return 0; }
