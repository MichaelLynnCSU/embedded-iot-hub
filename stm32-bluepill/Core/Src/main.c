
/* USER CODE BEGIN Header */
/******************************************************************************
 * \file main.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief STM32F103 Blue Pill — DHT11 temperature sensor hub with FRAM
 *        crash logging and ESP32 JSON output.
 *
 * \details Four FreeRTOS tasks run concurrently:
 *          - StartDefaultTask: aggregates sensor readings, sends JSON
 *            to ESP32 hub via UART2 every 5 seconds.
 *          - StartTask02/03/04: read DHT11 sensors on PA4/PA5/PA6,
 *            log to FRAM, update shared sensor1_data.
 *
 *          UART1: debug output to PC
 *          UART2: JSON output to ESP32 hub
 *          I2C1:  FRAM MB85RC256V
 ******************************************************************************/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "fram_driver.h"
#include "dht11_driver.h"
#include "trinity_log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/** \brief Shared sensor data buffer — written by sensor tasks, read by default task */
typedef struct
{
   uint8_t temp[4];  /*!< temperature readings from sensors 1-3 */
   uint8_t hum[4];   /*!< humidity readings from sensors 1-3 */
   uint8_t valid[4]; /*!< 1 if reading is valid, 0 if not yet read */
} SENSOR1_DATA_T;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR1_ID          1     /**< FRAM log ID for sensor 1 */
#define SENSOR2_ID          2     /**< FRAM log ID for sensor 2 */
#define SENSOR3_ID          3     /**< FRAM log ID for sensor 3 */
#define SENSOR4_ID          4
#define SENSOR_COUNT        4     /**< number of DHT11 sensors */
#define JSON_BUF_SIZE       64    /**< JSON message buffer size bytes */
#define MSG_BUF_SIZE        80    /**< UART message buffer size bytes */
#define I2C_SCAN_START      0x50  /**< I2C scan start address */
#define I2C_SCAN_END        0x58  /**< I2C scan end address */
#define I2C_SCAN_BUF_SIZE   50    /**< I2C scan message buffer size */
#define UART_TIMEOUT_MS     1000  /**< UART transmit timeout ms */
#define MUTEX_TIMEOUT_MS    100   /**< mutex wait timeout ms */
#define DEFAULT_TASK_DELAY  5000  /**< default task send interval ms */
#define SENSOR_TASK_DELAY   2000  /**< sensor task read interval ms */
#define SENSOR_WARMUP_MS    2000  /**< sensor warmup delay ms */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;

/* USER CODE BEGIN PV */
osMutexId g_dht11_mutex;   /**< mutex protecting DHT11 bus access */
osMutexId g_fram_mutex;    /**< mutex protecting FRAM I2C access */
osMutexId g_sensor1_mutex; /**< mutex protecting sensor1_data */

static SENSOR1_DATA_T g_sensor1_data; /**< shared sensor reading buffer */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

/* USER CODE BEGIN PFP */
void uart_print(const char *p_msg);
void uart_print_esp32(const char *p_msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******************************************************************************
 * \brief Transmit a string over UART1 (debug output).
 *
 * \param p_msg - Null-terminated string to transmit.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_print(const char *p_msg)
{
   (void)HAL_UART_Transmit(&huart1,
                             (uint8_t *)p_msg,
                             (uint16_t)strlen(p_msg),
                             UART_TIMEOUT_MS);
}

/******************************************************************************
 * \brief Transmit a string over UART2 (ESP32 hub output).
 *
 * \param p_msg - Null-terminated string to transmit.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_print_esp32(const char *p_msg)
{
   (void)HAL_UART_Transmit(&huart2,
                             (uint8_t *)p_msg,
                             (uint16_t)strlen(p_msg),
                             UART_TIMEOUT_MS);
}

/******************************************************************************
 * \brief Transmit a string via ITM SWO debug output.
 *
 * \param p_msg - Null-terminated string to transmit.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void itm_print(const char *p_msg)
{
   while ('\0' != *p_msg)
   {
      (void)ITM_SendChar(*p_msg);
      p_msg++;
   }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   DWT->CYCCNT  = 0;
   DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
   {
      uint32_t reset_reason              = 0;   /**< RCC reset reason flags */
      char     buffer[I2C_SCAN_BUF_SIZE] = {0}; /**< I2C scan message buffer */
      uint8_t  addr                      = 0;   /**< I2C scan address */

      uart_print("\r\n=== System Starting ===\r\n");

      reset_reason  = RCC->CSR;
      RCC->CSR     |= RCC_CSR_RMVF;

      FRAM_Init(&hi2c1);
      trinity_log_init();
      crash_log_dump_previous();
      trinity_log_init();
      DHT11_Init();

      for (addr = I2C_SCAN_START; addr < I2C_SCAN_END; addr++)
      {
         if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1,
                                               (uint16_t)(addr << 1),
                                               1, MUTEX_TIMEOUT_MS))
         {
            (void)snprintf(buffer, sizeof(buffer),
                           "Found I2C at 0x%02X\r\n", addr);
            uart_print(buffer);
         }
      }

      uart_print("Initialization complete!\r\n");
   }
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
   osMutexDef(dht11Mutex);
   g_dht11_mutex = osMutexCreate(osMutex(dht11Mutex));

   osMutexDef(framMutex);
   g_fram_mutex = osMutexCreate(osMutex(framMutex));

   osMutexDef(sensor1Mutex);
   g_sensor1_mutex = osMutexCreate(osMutex(sensor1Mutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 384);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 384);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 384);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTask04, osPriorityIdle, 0, 384);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  osThreadDef(myTask05, StartTask05, osPriorityIdle, 0, 384);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
   {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * \brief Default task — aggregates sensor readings and sends JSON to ESP32.
 *
 * \details Reads g_sensor1_data under mutex, computes average temperature
 *          across valid sensors, and sends JSON every DEFAULT_TASK_DELAY ms.
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
   char json_msg[JSON_BUF_SIZE] = {0}; /**< JSON output buffer */
   int  avg_temp                = 0;   /**< computed average temperature */
   int  valid_count             = 0;   /**< number of valid sensor readings */
   int  i                       = 0;   /**< loop index */

   (void)argument;

   uart_print("\r\n=== DefaultTask - JSON to ESP32 ===\r\n");
   crash_log_event("EVENT: TASKS_STARTED\n");

   for (;;)
   {
      avg_temp    = 0;
      valid_count = 0;

      if (osOK == osMutexWait(g_sensor1_mutex, MUTEX_TIMEOUT_MS))
      {
         for (i = 0; i < SENSOR_COUNT; i++)
         {
            if (g_sensor1_data.valid[i])
            {
               avg_temp += g_sensor1_data.temp[i];
               valid_count++;
            }
         }

         (void)osMutexRelease(g_sensor1_mutex);
      }

      if (0 < valid_count)
      {
         avg_temp = avg_temp / valid_count;
      }

      (void)snprintf(json_msg, sizeof(json_msg),
                     "{\"avg_temp\":%d}\n", avg_temp);
      uart_print_esp32(json_msg);
      uart_print("Sent to VROOM: ");
      uart_print(json_msg);

      osDelay(DEFAULT_TASK_DELAY);
   }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * \brief Sensor 1 task — reads DHT11 on PA4, logs to FRAM.
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
   uint8_t temp = 0;      /**< DHT11 temperature reading */
   uint8_t hum  = 0;      /**< DHT11 humidity reading */
   uint8_t ok   = 0;      /**< DHT11 read success flag */
   char    msg[MSG_BUF_SIZE] = {0}; /**< UART message buffer */

   (void)argument;

   uart_print("\r\n=== SENSOR 1 TASK STARTED (PA4) ===\r\n");
   osDelay(MUTEX_TIMEOUT_MS);

   for (;;)
   {
      if (osOK == osMutexWait(g_dht11_mutex, osWaitForever))
      {
         ok = DHT11_ReadData(GPIO_PIN_4, &temp, &hum);
         osMutexRelease(g_dht11_mutex);
      }

      if (ok)
      {
         (void)snprintf(msg, sizeof(msg),
                        "[S1] Temp=%dC Hum=%d%%\r\n", temp, hum);
         uart_print(msg);

         if (osOK == osMutexWait(g_sensor1_mutex, osWaitForever))
         {
            g_sensor1_data.temp[0]  = temp;
            g_sensor1_data.hum[0]   = hum;
            g_sensor1_data.valid[0] = 1;
            (void)osMutexRelease(g_sensor1_mutex);
         }

         (void)osMutexWait(g_fram_mutex, osWaitForever);
         FRAM_LogTemp(SENSOR1_ID, temp, hum);
         (void)osMutexRelease(g_fram_mutex);
      }
      else
      {
         uart_print("[S1] FAILED\r\n");
         crash_log_event("EVENT: DHT11_FAIL | SENSOR=1\n");
      }

      osDelay(SENSOR_TASK_DELAY);
   }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * \brief Sensor 2 task — reads DHT11 on PA5, logs to FRAM.
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
   uint8_t temp = 0;      /**< DHT11 temperature reading */
   uint8_t hum  = 0;      /**< DHT11 humidity reading */
   uint8_t ok   = 0;      /**< DHT11 read success flag */
   char    msg[MSG_BUF_SIZE] = {0}; /**< UART message buffer */

   (void)argument;

   uart_print("\r\n=== SENSOR 2 TASK STARTED (PA5) ===\r\n");

   osDelay(SENSOR_WARMUP_MS);
   if (osOK == osMutexWait(g_dht11_mutex, osWaitForever))
   {
      (void)DHT11_ReadData(GPIO_PIN_5, &temp, &hum);
      osMutexRelease(g_dht11_mutex);
   }
   osDelay(SENSOR_WARMUP_MS);

   for (;;)
   {
      if (osOK == osMutexWait(g_dht11_mutex, osWaitForever))
      {
         ok = DHT11_ReadData(GPIO_PIN_5, &temp, &hum);
         osMutexRelease(g_dht11_mutex);
      }

      if (ok)
      {
         (void)snprintf(msg, sizeof(msg),
                        "[S2] Temp=%dC Hum=%d%%\r\n", temp, hum);
         uart_print(msg);

         if (osOK == osMutexWait(g_sensor1_mutex, osWaitForever))
         {
            g_sensor1_data.temp[1]  = temp;
            g_sensor1_data.hum[1]   = hum;
            g_sensor1_data.valid[1] = 1;
            (void)osMutexRelease(g_sensor1_mutex);
         }

         (void)osMutexWait(g_fram_mutex, osWaitForever);
         FRAM_LogTemp(SENSOR2_ID, temp, hum);
         (void)osMutexRelease(g_fram_mutex);
      }
      else
      {
         uart_print("[S2] FAILED\r\n");
         crash_log_event("EVENT: DHT11_FAIL | SENSOR=2\n");
      }

      osDelay(SENSOR_TASK_DELAY);
   }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * \brief Sensor 3 task — reads DHT11 on PA6, logs to FRAM.
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
   uint8_t temp = 0;      /**< DHT11 temperature reading */
   uint8_t hum  = 0;      /**< DHT11 humidity reading */
   uint8_t ok   = 0;      /**< DHT11 read success flag */
   char    msg[MSG_BUF_SIZE] = {0}; /**< UART message buffer */

   (void)argument;

   uart_print("\r\n=== SENSOR 3 TASK STARTED (PA6) ===\r\n");

   osDelay(SENSOR_WARMUP_MS);
   if (osOK == osMutexWait(g_dht11_mutex, osWaitForever))
   {
      (void)DHT11_ReadData(GPIO_PIN_6, &temp, &hum);
      osMutexRelease(g_dht11_mutex);
   }
   osDelay(SENSOR_WARMUP_MS);

   for (;;)
   {
      if (osOK == osMutexWait(g_dht11_mutex, osWaitForever))
      {
         ok = DHT11_ReadData(GPIO_PIN_6, &temp, &hum);
         osMutexRelease(g_dht11_mutex);
      }

      if (ok)
      {
         (void)snprintf(msg, sizeof(msg),
                        "[S3] Temp=%dC Hum=%d%%\r\n", temp, hum);
         uart_print(msg);

         if (osOK == osMutexWait(g_sensor1_mutex, osWaitForever))
         {
            g_sensor1_data.temp[2]  = temp;
            g_sensor1_data.hum[2]   = hum;
            g_sensor1_data.valid[2] = 1;
            (void)osMutexRelease(g_sensor1_mutex);
         }

         (void)osMutexWait(g_fram_mutex, osWaitForever);
         FRAM_LogTemp(SENSOR3_ID, temp, hum);
         (void)osMutexRelease(g_fram_mutex);
      }
      else
      {
         uart_print("[S3] FAILED\r\n");
         crash_log_event("EVENT: DHT11_FAIL | SENSOR=3\n");
      }

      osDelay(SENSOR_TASK_DELAY);
   }
  /* USER CODE END StartTask04 */
}

void StartTask05(void const * argument)
{
    uint8_t temp = 0;
    uint8_t hum  = 0;
    uint8_t ok   = 0;
    char    msg[MSG_BUF_SIZE] = {0};

    (void)argument;

    uart_print("\r\n=== SENSOR 4 TASK STARTED (PA7 - ATtiny85) ===\r\n");
    osDelay(SENSOR_WARMUP_MS);
    if (osOK == osMutexWait(g_dht11_mutex, osWaitForever))
    {
       ok = DHT11_ReadData(GPIO_PIN_7, &temp, &hum);
       osMutexRelease(g_dht11_mutex);
    }
    osDelay(SENSOR_WARMUP_MS);

    for (;;)
    {
       if (osOK == osMutexWait(g_dht11_mutex, osWaitForever))
       {
          ok = DHT11_ReadData(GPIO_PIN_7, &temp, &hum);
          osMutexRelease(g_dht11_mutex);
       }

        if (ok)
        {
            (void)snprintf(msg, sizeof(msg),
                           "[S4-ATtiny] Temp=%dC Hum=%d%%\r\n", temp, hum);
            uart_print(msg);

            if (osOK == osMutexWait(g_sensor1_mutex, osWaitForever))
            {
                g_sensor1_data.temp[3]  = temp;
                g_sensor1_data.hum[3]   = hum;
                g_sensor1_data.valid[3] = 1;
                (void)osMutexRelease(g_sensor1_mutex);
            }

            (void)osMutexWait(g_fram_mutex, osWaitForever);
            FRAM_LogTemp(SENSOR4_ID, temp, hum);
            (void)osMutexRelease(g_fram_mutex);
        }
        else
        {
            uart_print("[S4-ATtiny] FAILED\r\n");
            crash_log_event("EVENT: DHT11_FAIL | SENSOR=4\n");
        }

        osDelay(SENSOR_TASK_DELAY);
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
   crash_log_event("EVENT: ERROR_HANDLER\n");
   __disable_irq();
   while (1)
   {
      /* halt */
   }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
