#include "trinity_log.h"
#include "trinity_private_esp.h"
int  trinity_log_init(void)                   { return 0; }
void trinity_log_dump_previous(void)          {}
int  trinity_log_erase(void)                  { return 0; }
void trinity_log_event(const char *p)         { printk("[TRINITY] %s\n", p); }
void write_panic(const char *p, uint16_t len) { (void)p; (void)len; }
int  trinity_log_stats_init(void)             { return 0; }
