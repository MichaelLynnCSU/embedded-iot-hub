#include "trinity_log.h"
#include "trinity_private_idf.h"
#include "esp_log.h"
void trinity_log_init(void)              { trinity_canary_set_booted(); }
void trinity_log_dump_previous(void)     {}
void trinity_log_erase(void)             {}
void trinity_log_event(const char *p)    { ESP_LOGI("TRINITY", "%s", p); }
