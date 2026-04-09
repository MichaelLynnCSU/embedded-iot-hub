#include "trinity_log.h"
#include "FreeRTOS.h"
#include "task.h"
extern uint32_t g_boot_count;
void panic_handler(const char *p, TRINITY_ERROR_E e) { (void)p; (void)e; NVIC_SystemReset(); }
void trinity_hard_fault_handler(void)                { NVIC_SystemReset(); }
void crash_fault_handler_c(uint32_t *s, uint32_t t)  { (void)s; (void)t; NVIC_SystemReset(); }
void HardFault_Handler_C(uint32_t *s)                { crash_fault_handler_c(s, 1u); }
void vApplicationStackOverflowHook(TaskHandle_t t, char *n) { (void)t; (void)n; NVIC_SystemReset(); }
void vApplicationMallocFailedHook(void)              { NVIC_SystemReset(); }
void vApplicationIdleHook(void)                      { trinity_wdt_kick(); }
