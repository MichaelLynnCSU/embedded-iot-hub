/******************************************************************************
 * \file trinity_wdt_stm32_f1.c
 * \brief Trinity watchdog -- STM32F103 FreeRTOS.
 *
 * \details Fully standalone. Arms IWDG via HAL.
 *          STM32F103 LSI ~40kHz, prescaler 64 → ~1.6ms/tick, reload 1875 → ~3s.
 *          WDT is kicked from vApplicationIdleHook() in trinity_fault_stm32_f1.c.
 *          If any FreeRTOS task starves the idle task for >3s the IWDG fires.
 ******************************************************************************/

#include "trinity_log.h"
#include "stm32f1xx_hal.h"

#define IWDG_PRESCALER_VAL  IWDG_PRESCALER_64
#define IWDG_RELOAD_VAL     1875u

static IWDG_HandleTypeDef g_hiwdg = {0};

void trinity_wdt_init(void)
{
   g_hiwdg.Instance       = IWDG;
   g_hiwdg.Init.Prescaler = IWDG_PRESCALER_VAL;
   g_hiwdg.Init.Reload    = IWDG_RELOAD_VAL;

   if (HAL_OK != HAL_IWDG_Init(&g_hiwdg))
   {
      trinity_uart_log("[TRINITY] WARN: IWDG init failed\r\n");
      return;
   }

   trinity_uart_log("[TRINITY] IWDG armed (~3s timeout)\r\n");
}

void trinity_wdt_kick(void)
{
   (void)HAL_IWDG_Refresh(&g_hiwdg);
}
