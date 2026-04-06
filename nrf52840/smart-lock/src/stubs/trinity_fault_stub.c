#include "trinity_log.h"
#include <nrf.h>
void k_sys_fatal_error_handler(unsigned int r, const struct arch_esf *e)
{ (void)r; (void)e; NVIC_SystemReset(); }
