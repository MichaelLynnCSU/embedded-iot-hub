#include <stdint.h>
#include <stddef.h>

/* trinity stubs */
void trinity_log_event(const char *p)  { (void)p; }
void trinity_wdt_add(void)             {}
void trinity_wdt_kick(void)            {}
void trinity_log_heap_stats(void)      {}
void trinity_log_task_stats(void)      {}

/* wifi stub */
void *g_wifi_eg = NULL;

/* battery stub */
int battery_read_mv(void) { return 9500; }

/* tcp_client stub */
uint32_t tcp_client_exchange(void) { return 0; }
