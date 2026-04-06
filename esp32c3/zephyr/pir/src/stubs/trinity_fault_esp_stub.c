#include "trinity_log.h"
#include <zephyr/sys/reboot.h>
void k_sys_fatal_error_handler(unsigned int r, const struct arch_esf *e)
{ (void)r; (void)e; sys_reboot(SYS_REBOOT_COLD); }
