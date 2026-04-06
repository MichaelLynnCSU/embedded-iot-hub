#ifndef FRAM_DRIVER_H
#define FRAM_DRIVER_H
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* FRAM I2C address and size */
#define FRAM_ADDR           0x50
#define FRAM_SIZE           32768

/* Partition layout */
#define FRAM_META_ADDR      0x0000u  /**< Metadata block (16 bytes)          */
#define FRAM_TEMPLOG_ADDR   0x0010u  /**< Temperature log ring buffer        */
#define FRAM_CRASHLOG_ADDR  0x4010u  /**< Crash log ring buffer              */
#define FRAM_TEMPLOG_SIZE   0x4000u  /**< 16KB for temp log                  */
#define FRAM_CRASHLOG_SIZE  0x3FF0u  /**< ~16KB for crash log                */

/**
 * \brief One temperature log entry.
 */
typedef struct __attribute__((packed))
{
   uint32_t timestamp;    /*!< HAL_GetTick() at time of logging    */
   uint8_t  pin;          /*!< Sensor ID / slot index              */
   uint8_t  temperature;  /*!< Temperature in degrees C            */
   uint8_t  humidity;     /*!< Relative humidity percent           */
   uint8_t  crc;          /*!< CRC-8 over preceding 7 bytes        */
} TEMP_LOG_ENTRY_X;

/**
 * \brief One crash log entry stored in FRAM crash partition.
 */
typedef struct __attribute__((packed))
{
   uint32_t timestamp;    /*!< HAL_GetTick() at time of crash      */
   uint8_t  error_code;   /*!< TRINITY_ERROR_E value               */
   uint8_t  boot_count;   /*!< Boot number when crash occurred     */
   uint8_t  reserved[2];  /*!< Padding for alignment               */
   uint8_t  crc;          /*!< CRC-8 over preceding 8 bytes        */
} CRASH_LOG_ENTRY_X;

/* Existing temp log API */
void FRAM_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef FRAM_Write(uint16_t addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef FRAM_Read(uint16_t addr, uint8_t *data, uint16_t len);
void FRAM_SaveMeta(void);
void FRAM_LoadMeta(void);
void FRAM_LogTemp(uint16_t sensor_id, uint8_t temp, uint8_t hum);
uint16_t FRAM_GetWritePtr(void);
uint32_t FRAM_GetTotalEntries(void);

/* Crash log API — used by trinity_log */
void     FRAM_LogCrash(uint8_t error_code, uint8_t boot_count);
uint32_t FRAM_GetTotalCrashes(void);

#endif /* FRAM_DRIVER_H */
