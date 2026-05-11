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
 *          assembled into lines by ring_buffer.c. The main loop dequeues and
 *          parses each line without blocking.
 *
 * \note    Pipeline architecture:
 *          Firmware is split into two independent producer/consumer
 *          pipelines. This separation is intentional and must be
 *          preserved -- mixing them reintroduces the instability
 *          this design was built to fix.
 *
 *          TELEMETRY  UART ISR -> ring buffer -> drain_uart_queue()
 *                     -> parser_process_line() -> ui_update()
 *
 *          DEBUG LOG  log_enqueue() -> RAM queue -> log_drain()
 *                     -> USB CDC (non-blocking)
 *
 *          ISR does byte collection only -- no parsing, no output,
 *          no state mutation. All processing happens in main loop.
 *          Debug traffic has zero influence on telemetry timing.
 *
 * \note    Freeze / stall fix:
 *          System previously froze under UART load and stalled during
 *          debug prints. Root cause: blocking work in timing-critical
 *          paths and no decoupling between data production and
 *          consumption.
 *
 *          Fix: UART RX reduced to byte-only ISR buffering. Debug
 *          output deferred to a RAM queue drained here in the main
 *          loop. All transmission is non-blocking. ISR execution time
 *          is now bounded and deterministic; freezes are eliminated.
 *
 *          Main loop is the cooperative scheduler for all non-ISR work.
 *          Every function called here must be non-blocking and complete
 *          in < 1ms. Violating this degrades UI frame timing and risks
 *          dropping UART bytes.
 *
 *
 * \note    Render trigger:
 *          ui_update() is called conditionally, not on every loop tick.
 *          drain_uart_queue() triggers it only when new telemetry arrived.
 *          heartbeat_tick() triggers it once per second as a fallback.
 *          This is intentional -- unconditional rendering adds SPI
 *          traffic and LVGL work with no visual benefit on a
 *          sporadically-updated dashboard. The pipeline is still
 *          deterministic; the render stage is just gated on dirty state.
 *
 *          Event-driven render on new telemetry, 1Hz periodic render as a
 *          liveness guarantee when the ESP32 is silent heartbeat and ui_update()
 *          never fires unconditionally.
 *
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
#include "ring_buffer.h"
#include "trinity_log.h"
#include "fram_driver.h"
#include <string.h>

/******************************** CONSTANTS ***********************************/

#define HB_CHECK_MS      1000ul  /**< Heartbeat check interval in ms          */
#define MAIN_LOOP_DELAY_MS  5u   /**< Main loop sleep between iterations (ms) */
#define PING_BUF_LEN       80u   /**< Heartbeat log string buffer length       */

/************************** STATIC (PRIVATE) DATA *****************************/

static SPI_HandleTypeDef  g_hspi1;  /**< SPI1 — ILI9341 display              */
static SPI_HandleTypeDef  g_hspi2;  /**< SPI2 — XPT2046 touch controller      */
UART_HandleTypeDef        g_huart1; /**< USART1 — ESP32 telemetry             */
I2C_HandleTypeDef         g_hi2c1;  /**< I2C1 — FRAM chip PB6/PB7            */

static uint8_t  g_uart_rx_byte = 0u; /**< Single-byte DMA target              */
static uint32_t g_last_hb      = 0ul; /**< Tick of last heartbeat check       */

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
   char    tmp[UART_LINE_LEN] = {0}; /**< Local copy of dequeued line         */
   uint8_t ui_dirty           = 0u;  /**< Non-zero when UI needs refreshing   */

   while (rb_dequeue(tmp))
   {
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
   static char g_ping[PING_BUF_LEN]; /**< Static: avoids stack alloc each call */
   uint32_t    now = 0ul;

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

   if (HAL_OK != HAL_SPI_Init(&g_hspi1)) { Error_Handler(); }
}

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

   if (HAL_OK != HAL_SPI_Init(&g_hspi2)) { Error_Handler(); }
}

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

   if (HAL_OK != HAL_UART_Init(&g_huart1)) { Error_Handler(); }
}

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

   if (HAL_OK != HAL_I2C_Init(&g_hi2c1)) { Error_Handler(); }
}

static void mx_gpio_init(void)
{
   GPIO_InitTypeDef g = {0};

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
 * \brief  UART receive-complete ISR callback — feeds bytes into ring buffer.
 *
 * \param  p_huart - HAL UART handle that triggered the callback.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *p_huart)
{
   if ((NULL == p_huart) || (USART1 != p_huart->Instance))
   {
      return;
   }

   rb_push_byte(g_uart_rx_byte);

   (void)HAL_UART_Receive_IT(&g_huart1, &g_uart_rx_byte, 1u);
}

void SystemClock_Config(void)
{
   RCC_OscInitTypeDef osc_init = {0};
   RCC_ClkInitTypeDef clk_init = {0};

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

   if (HAL_OK != HAL_RCC_OscConfig(&osc_init)) { Error_Handler(); }

   clk_init.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK |
                             RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
   clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
   clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
   clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
   clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

   if (HAL_OK != HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_3)) { Error_Handler(); }
}

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
   mx_i2c1_init();

   /* I2C bus scan -- remove after FRAM confirmed */
   {
      char scan_msg[48];
      uint8_t found = 0u;
      for (uint16_t a = 0x08u; a < 0x78u; a++)
      {
         if (HAL_OK == HAL_I2C_IsDeviceReady(&g_hi2c1, (uint16_t)(a << 1u), 2u, 10u))
         {
            snprintf(scan_msg, sizeof(scan_msg), "[I2C] device at 0x%02X\r\n", a);
            log_enqueue(scan_msg);
            found = 1u;
         }
      }
      if (!found) { log_enqueue("[I2C] no devices found\r\n"); }
   }

   fram_init(&g_hi2c1);
   trinity_log_init();
   mx_spi1_init();
   mx_spi2_init();
   mx_usart1_uart_init();

   rb_init();                   /* Initialise UART ring buffer                */

   ILI9341_Init(&g_hspi1);
   XPT2046_Init(&g_hspi2);

   lv_init();
   ui_create();
   ui_update();

   (void)HAL_UART_Receive_IT(&g_huart1, &g_uart_rx_byte, 1u);
   log_enqueue("[BOOT] Dashboard ready\r\n");

   while (1)
   {
      (void)lv_timer_handler();
      log_drain();
      drain_uart_queue();
      heartbeat_tick();
      HAL_Delay(MAIN_LOOP_DELAY_MS);
   }

   return 0;
}

void Error_Handler(void)
{
   __disable_irq();
   while (1) { /* Halt */ }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *p_file, uint32_t line)
{
   (void)p_file;
   (void)line;
}
#endif /* USE_FULL_ASSERT */
