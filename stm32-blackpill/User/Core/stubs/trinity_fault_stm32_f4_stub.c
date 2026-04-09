#include "trinity_log.h"
void panic_handler(const char *p, TRINITY_ERROR_E e)
{ (void)p; (void)e; NVIC_SystemReset(); }
void trinity_hard_fault_handler(void) { NVIC_SystemReset(); }
