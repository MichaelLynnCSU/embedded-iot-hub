#ifndef FRAM_DRIVER_H
#define FRAM_DRIVER_H

/******************************************************************************
 * Copyright (c) 2025 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    fram_driver.h
 * \author  MichaelLynnCSU
 * \date    01-01-2025
 *
 * \brief   FM24CL16B FRAM driver — BluePill (STM32F103) public interface.
 *
 * \details Crash-log state (g_crash_write_ptr, g_total_crash_entries) has
 *          moved to ring_buffer.c.  FRAM_SaveMeta / FRAM_LoadMeta now accept
 *          those values as parameters so the driver stays stateless w.r.t.
 *          the crash partition.
 *
 *          CRASH_LOG_ENTRY_X and the FRAM_CRASHLOG_* constants remain here
 *          because both layers (fram_driver and ring_buffer) need them.
 ******************************************************************************/

#include "stm32f1xx_hal.h"
#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* FRAM I2C address and total size                                            */
/*---------------------------------------------------------------------------*/
#define FRAM_ADDR           0x50u
#define FRAM_SIZE           32768u

/*---------------------------------------------------------------------------*/
/* Partition layout                                                            */
/*---------------------------------------------------------------------------*/
#define FRAM_META_ADDR      0x0000u  /**< Metadata block (16 bytes)          */
#define FRAM_TEMPLOG_ADDR   0x0010u  /**< Temperature log ring buffer        */
#define FRAM_CRASHLOG_ADDR  0x4010u  /**< Crash log ring buffer              */
#define FRAM_TEMPLOG_SIZE   0x4000u  /**< 16 KB for temp log                 */
#define FRAM_CRASHLOG_SIZE  0x3FF0u  /**< ~16 KB for crash log               */

/*---------------------------------------------------------------------------*/
/* Entry types — needed by both fram_driver and ring_buffer                  */
/*---------------------------------------------------------------------------*/

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

/*---------------------------------------------------------------------------*/
/* Core FRAM API                                                              */
/*---------------------------------------------------------------------------*/

void              FRAM_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef FRAM_Write(uint16_t addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef FRAM_Read(uint16_t addr, uint8_t *data, uint16_t len);

/*---------------------------------------------------------------------------*/
/* Metadata API — crash fields supplied by caller (owned by ring_buffer.c)   */
/*---------------------------------------------------------------------------*/

/**
 * \brief  Persist metadata to FRAM.
 *
 * \param  temp_write_ptr    - Current temp-log write offset.
 * \param  total_temp        - Cumulative temp entry count.
 * \param  crash_write_ptr   - Current crash-log write offset (from rb layer).
 * \param  total_crash       - Cumulative crash entry count   (from rb layer).
 */
void FRAM_SaveMeta(uint16_t temp_write_ptr,
                   uint32_t total_temp,
                   uint16_t crash_write_ptr,
                   uint32_t total_crash);

/**
 * \brief  Load metadata from FRAM.
 *
 * \param  p_temp_write_ptr  - Out: temp-log write offset  (may be NULL).
 * \param  p_total_temp      - Out: cumulative temp count  (may be NULL).
 * \param  p_crash_write_ptr - Out: crash-log write offset (may be NULL).
 * \param  p_total_crash     - Out: cumulative crash count (may be NULL).
 */
void FRAM_LoadMeta(uint16_t *p_temp_write_ptr,
                   uint32_t *p_total_temp,
                   uint16_t *p_crash_write_ptr,
                   uint32_t *p_total_crash);

/*---------------------------------------------------------------------------*/
/* Temperature log API                                                        */
/*---------------------------------------------------------------------------*/

void     FRAM_LogTemp(uint16_t sensor_id, uint8_t temp, uint8_t hum);
uint16_t FRAM_GetWritePtr(void);
uint32_t FRAM_GetTotalEntries(void);

#endif /* FRAM_DRIVER_H */
