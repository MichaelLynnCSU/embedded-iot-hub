#include "trinity_log.h"
#include "esp_system.h"
#include "esp_private/panic_internal.h"
void esp_panic_handler_user(panic_info_t *p)
{ (void)p; esp_restart(); }
