/******************************************************************************
 * \file trinity_wdt_stm32_f4.c
 * \brief Trinity watchdog -- STM32F411 bare metal.
 *
 * \details Fully standalone -- no dependencies on other Trinity files.
 *          Arms IWDG via HAL. LSI ~32kHz, prescaler 32, reload 3000 (~3s).
 *          trinity_uart_log() called for status -- depends on trinity_fram
 *          being compiled. If stubbing wdt, stub fram too or the uart_log
 *          call will be the stub's no-op version.
 ******************************************************************************/

#include "trinity_log.h"
#include "stm32f4xx_hal.h"

#define IWDG_PRESCALER_VAL  IWDG_PRESCALER_32
#define IWDG_RELOAD_VAL     3000u

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
