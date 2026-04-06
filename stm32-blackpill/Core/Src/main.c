/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    main.c
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   Smart Home Dashboard entry point — STM32F411 BlackPill.
 *
 * \details Initialises all peripherals (SPI, UART, USB, LVGL), starts the
 *          ILI9341 display and XPT2046 touch controller, then enters the
 *          main loop which:
 *            1. Drives the LVGL timer handler.
 *            2. Drains the USB CDC log queue.
 *            3. Processes queued UART telemetry lines from the ESP32.
 *            4. Refreshes the dashboard UI on every heartbeat tick.
 *
 *          UART telemetry is received one byte at a time via interrupt and
 *          assembled into lines stored in a ring buffer. The main loop
 *          dequeues and parses each line without blocking.
 ******************************************************************************/

#include "crash_log.h"
#include "main.h"
#include "usb_device.h"
#include "ili9341.h"
#include "xpt2046.h"
#include "lvgl.h"
#include "ui.h"
#include "parser.h"
#include "log.h"
#include "trinity_log.h"
#include "fram_driver.h"
#include <string.h>

/******************************** CONSTANTS ***********************************/

#define HB_CHECK_MS      1000ul  /**< Heartbeat check interval in ms          */
#define UART_QUEUE_DEPTH    8u   /**< UART line ring-buffer depth              */
#define MAIN_LOOP_DELAY_MS  5u   /**< Main loop sleep between iterations (ms) */
#define PING_BUF_LEN       80u   /**< Heartbeat log string buffer length       */

/************************** STATIC (PRIVATE) DATA *****************************/

static SPI_HandleTypeDef  g_hspi1; /**< SPI1 — ILI9341 display               */
static SPI_HandleTypeDef  g_hspi2; /**< SPI2 — XPT2046 touch controller       */
UART_HandleTypeDef        g_huart1; /**< USART1 — ESP32 telemetry (extern in board_compat.h) */ /**< USART1 — ESP32 telemetry              */
I2C_HandleTypeDef         g_hi2c1;  /**< I2C1 — FRAM chip PB6/PB7             */

static uint8_t  g_uart_rx_byte              = 0u;  /**< Single-byte DMA target  */
static char     g_uart_line[UART_LINE_LEN]  = {0}; /**< Line assembly buffer    */
static uint8_t  g_uart_line_idx             = 0u;  /**< Current write position  */
static char     g_uart_queue[UART_QUEUE_DEPTH][UART_LINE_LEN]; /**< Line queue  */
static uint8_t  g_uart_q_head              = 0u;   /**< Queue write index       */
static uint8_t  g_uart_q_tail             = 0u;   /**< Queue read index        */
static volatile uint8_t g_uart_q_count    = 0u;   /**< Number of pending lines */

static uint32_t g_last_hb = 0ul; /**< Tick of last heartbeat check            */

/************************** STATIC (PRIVATE) PROTOTYPES ***********************/

static void mx_gpio_init(void);
static void mx_i2c1_init(void);
static void mx_spi1_init(void);
static void mx_spi2_init(void);
static void mx_usart1_uart_init(void);

/************************** STATIC (PRIVATE) FUNCTIONS ************************/

/**
 * \brief  Process all pending UART lines and refresh the UI if any arrived.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void drain_uart_queue(void)
{
   char    tmp[UART_LINE_LEN] = {0}; /**< Local copy of queued line           */
   uint8_t ui_dirty           = 0u;  /**< Non-zero when UI needs refreshing   */

   while (g_uart_q_count > 0u)
   {
      (void)memcpy(tmp, g_uart_queue[g_uart_q_tail], UART_LINE_LEN);
      g_uart_q_tail = (uint8_t)((g_uart_q_tail + 1u) % UART_QUEUE_DEPTH);

      __disable_irq();
      g_uart_q_count--;
      __enable_irq();

      parser_process_line(tmp);
      ui_dirty = 1u;
   }

   if (0u != ui_dirty)
   {
      ui_update();
   }
}

/**
 * \brief  Emit a periodic heartbeat log line and refresh the UI.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void heartbeat_tick(void)
{
   static char g_ping[PING_BUF_LEN]; /**< Static: avoids stack allocation each call */
   uint32_t    now = 0ul;            /**< Current HAL tick                          */

   now = HAL_GetTick();

   if ((now - g_last_hb) < HB_CHECK_MS)
   {
      return;
   }

   g_last_hb = now;
   trinity_check_stack();
   ui_update();

   (void)snprintf(g_ping, sizeof(g_ping),
                  "[HB] t=%lu reeds=%u pir=%u lgt=%u lck=%u\r\n",
                  (unsigned long)now,
                  (unsigned int)ui_get_reed_count(),
                  (unsigned int)ui_get_dev_online(eDEV_PIR),
                  (unsigned int)ui_get_dev_online(eDEV_LIGHT),
                  (unsigned int)ui_get_dev_online(eDEV_LOCK));
   log_enqueue(g_ping);
}

/**
 * \brief  Initialise SPI1 (ILI9341 display, max speed).
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void mx_spi1_init(void)
{
   g_hspi1.Instance               = SPI1;
   g_hspi1.Init.Mode              = SPI_MODE_MASTER;
   g_hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
   g_hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
   g_hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
   g_hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
   g_hspi1.Init.NSS               = SPI_NSS_SOFT;
   g_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
   g_hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
   g_hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
   g_hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
   g_hspi1.Init.CRCPolynomial     = 10u;

   if (HAL_OK != HAL_SPI_Init(&g_hspi1))
   {
      Error_Handler();
   }
}

/**
 * \brief  Initialise SPI2 (XPT2046 touch, reduced speed).
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void mx_spi2_init(void)
{
   g_hspi2.Instance               = SPI2;
   g_hspi2.Init.Mode              = SPI_MODE_MASTER;
   g_hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
   g_hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
   g_hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
   g_hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
   g_hspi2.Init.NSS               = SPI_NSS_SOFT;
   g_hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
   g_hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
   g_hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
   g_hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
   g_hspi2.Init.CRCPolynomial     = 10u;

   if (HAL_OK != HAL_SPI_Init(&g_hspi2))
   {
      Error_Handler();
   }
}

/**
 * \brief  Initialise USART1 at 115200 8N1 for ESP32 telemetry.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void mx_usart1_uart_init(void)
{
   g_huart1.Instance          = USART1;
   g_huart1.Init.BaudRate     = 115200u;
   g_huart1.Init.WordLength   = UART_WORDLENGTH_8B;
   g_huart1.Init.StopBits     = UART_STOPBITS_1;
   g_huart1.Init.Parity       = UART_PARITY_NONE;
   g_huart1.Init.Mode         = UART_MODE_TX_RX;
   g_huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
   g_huart1.Init.OverSampling = UART_OVERSAMPLING_16;

   if (HAL_OK != HAL_UART_Init(&g_huart1))
   {
      Error_Handler();
   }
}

/**
 * \brief  Initialise I2C1 for FRAM chip on PB6 (SCL) / PB7 (SDA).
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void mx_i2c1_init(void)
{
   g_hi2c1.Instance              = I2C1;
   g_hi2c1.Init.ClockSpeed       = 400000u;
   g_hi2c1.Init.DutyCycle        = I2C_DUTYCYCLE_2;
   g_hi2c1.Init.OwnAddress1      = 0u;
   g_hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
   g_hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLED;
   g_hi2c1.Init.OwnAddress2      = 0u;
   g_hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLED;
   g_hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLED;

   if (HAL_OK != HAL_I2C_Init(&g_hi2c1))
   {
      Error_Handler();
   }
}

/**
 * \brief  Initialise all GPIO pins for display, touch, and UART.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
static void mx_gpio_init(void)
{
   GPIO_InitTypeDef g = {0}; /**< GPIO init structure */

   __HAL_RCC_GPIOH_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();

   HAL_GPIO_WritePin(GPIOA, DC_RS_Pin | RESET_Pin | CS_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(T_CS_GPIO_Port, T_CS_Pin, GPIO_PIN_RESET);

   g.Pin   = DC_RS_Pin | RESET_Pin | CS_Pin;
   g.Mode  = GPIO_MODE_OUTPUT_PP;
   g.Pull  = GPIO_NOPULL;
   g.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &g);

   g.Pin = T_CS_Pin;
   HAL_GPIO_Init(T_CS_GPIO_Port, &g);

   g.Pin  = T_IRQ_Pin;
   g.Mode = GPIO_MODE_INPUT;
   g.Pull = GPIO_PULLUP;
   HAL_GPIO_Init(T_IRQ_GPIO_Port, &g);
}

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  UART receive-complete ISR callback — assembles bytes into lines.
 *
 * \param  p_huart - HAL UART handle that triggered the callback.
 *
 * \return void
 *
 * \details Lines are delimited by CR or LF. Completed lines are copied into
 *          the ring buffer for main-loop processing. Overflow bytes are
 *          discarded and the assembly index is reset.
 *
 * \author MichaelLynnCSU
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *p_huart)
{
   char c = '\0'; /**< Received character */

   if (NULL == p_huart)
   {
      return;
   }

   if (USART1 != p_huart->Instance)
   {
      return;
   }

   c = (char)g_uart_rx_byte;

   if (('\n' == c) || ('\r' == c))
   {
      if ((g_uart_line_idx > 0u) && (g_uart_q_count < UART_QUEUE_DEPTH))
      {
         g_uart_line[g_uart_line_idx] = '\0';
         (void)memcpy(g_uart_queue[g_uart_q_head],
                      g_uart_line,
                      g_uart_line_idx + 1u);
         g_uart_q_head = (uint8_t)((g_uart_q_head + 1u) % UART_QUEUE_DEPTH);
         g_uart_q_count++;
      }

      g_uart_line_idx = 0u;
   }
   else if (g_uart_line_idx < (uint8_t)(UART_LINE_LEN - 1u))
   {
      g_uart_line[g_uart_line_idx] = c;
      g_uart_line_idx++;
   }
   else
   {
      /* Line overflow — discard and reset */
      g_uart_line_idx = 0u;
   }

   (void)HAL_UART_Receive_IT(&g_huart1, &g_uart_rx_byte, 1u);
}

/**
 * \brief  System Clock configuration — 96 MHz from 23 MHz HSE via PLL.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void SystemClock_Config(void)
{
   RCC_OscInitTypeDef osc_init = {0}; /**< Oscillator init parameters */
   RCC_ClkInitTypeDef clk_init = {0}; /**< Clock tree init parameters */

   __HAL_RCC_PWR_CLK_ENABLE();
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

   osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   osc_init.HSEState       = RCC_HSE_ON;
   osc_init.PLL.PLLState   = RCC_PLL_ON;
   osc_init.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
   osc_init.PLL.PLLM       = 23u;
   osc_init.PLL.PLLN       = 354u;
   osc_init.PLL.PLLP       = RCC_PLLP_DIV4;
   osc_init.PLL.PLLQ       = 8u;

   if (HAL_OK != HAL_RCC_OscConfig(&osc_init))
   {
      Error_Handler();
   }

   clk_init.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK |
                             RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
   clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
   clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
   clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
   clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

   if (HAL_OK != HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_3))
   {
      Error_Handler();
   }
}

/**
 * \brief  Application entry point.
 *
 * \param  void
 *
 * \return int - Never returns under normal operation.
 *
 * \author MichaelLynnCSU
 */
int main(void)
{
   HAL_Init();
   SystemClock_Config();

   HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
   HAL_NVIC_SetPriority(OTG_FS_IRQn,  5u, 0u);
   HAL_NVIC_SetPriority(USART1_IRQn,  6u, 0u);
   HAL_NVIC_SetPriority(SysTick_IRQn, 0u, 0u);

   mx_gpio_init();

   HAL_GPIO_WritePin(GPIOA, RESET_Pin, GPIO_PIN_RESET);
   HAL_Delay(100u);
   HAL_GPIO_WritePin(GPIOA, RESET_Pin, GPIO_PIN_SET);
   HAL_Delay(100u);

   MX_USB_DEVICE_Init();
   crash_log_init();
   trinity_log_init();
   mx_i2c1_init();
   fram_init(&g_hi2c1);
   mx_spi1_init();
   mx_spi2_init();
   mx_usart1_uart_init();

   ILI9341_Init(&g_hspi1);
   XPT2046_Init(&g_hspi2);

   lv_init();
   ui_create();
   ui_update();

   (void)HAL_UART_Receive_IT(&g_huart1, &g_uart_rx_byte, 1u);
   log_enqueue("[BOOT] Dashboard ready\r\n");

   while (1)
   {
      /* lv_timer_handler() drives LVGL animations and flushes the display.
       * Cap its execution time to prevent SPI blocking the main loop. */
      (void)lv_timer_handler();
      log_drain();
      drain_uart_queue();
      heartbeat_tick();
      HAL_Delay(MAIN_LOOP_DELAY_MS);
   }

   return 0;
}

/**
 * \brief  Fatal error handler — disables interrupts and halts.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void Error_Handler(void)
{
   __disable_irq();
   while (1)
   {
      /* Halt */
   }
}

#ifdef USE_FULL_ASSERT
/**
 * \brief  Assert failure handler invoked by HAL when USE_FULL_ASSERT is defined.
 *
 * \param  p_file - Source file name string.
 * \param  line   - Line number of the assertion.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void assert_failed(uint8_t *p_file, uint32_t line)
{
   /* Parameter checking not carried out for performance reasons */
   (void)p_file;
   (void)line;
}
#endif /* USE_FULL_ASSERT */
