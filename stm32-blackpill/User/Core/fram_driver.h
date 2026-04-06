/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    fram_driver.h
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   FM24CL16B FRAM driver — BlackPill port of BluePill fram_driver.
 *
 * \details Partition layout (32KB FM24CL16B):
 *
 *   0x0000 -  0x000F  Meta         (16 bytes) — write ptr, entry count
 *   0x0010 -  0x400F  Temp log     (16KB)     — ring buffer of TempLog entries
 *   0x4010 -  0x7FFF  Trinity log  (16KB)     — ring buffer of crash entries
 *
 * \warning FRAM_Init() must be called before any other FRAM function.
 *          I2C address is set by A0/A1/A2 pins — default 0x50 (all low).
 ******************************************************************************/

#ifndef INCLUDE_FRAM_DRIVER_H_
#define INCLUDE_FRAM_DRIVER_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/******************************** CONSTANTS ***********************************/

#define FRAM_ADDR            0x50u      /**< I2C address (A0/A1/A2 all low)   */
#define FRAM_SIZE            0x8000u    /**< Total FRAM size: 32768 bytes      */

/** Partition base addresses */
#define FRAM_META_ADDR       0x0000u    /**< Metadata block (16 bytes)         */
#define FRAM_CRASHLOG_ADDR   0x0010u    /**< Trinity crash log ring buffer     */

/** Partition sizes */
#define FRAM_CRASHLOG_SIZE   0x7FF0u    /**< ~32KB for crash log               */

/************************** STRUCTURE DATA TYPES ******************************/

/**
 * \brief One trinity crash log entry stored in FRAM crash partition.
 */
typedef struct __attribute__((packed))
{
   uint32_t timestamp;    /*!< HAL_GetTick() at time of crash      */
   uint8_t  error_code;   /*!< TRINITY_ERROR_E value               */
   uint8_t  boot_count;   /*!< Boot number when crash occurred     */
   uint8_t  reserved[2];  /*!< Padding for alignment               */
   uint8_t  crc;          /*!< CRC-8 over preceding 8 bytes        */
} CRASH_LOG_ENTRY_X;

/**
 * \brief FRAM metadata block stored at FRAM_META_ADDR.
 */
typedef struct
{
   uint16_t crash_write_ptr;    /*!< Next write offset in crash log  */
   uint32_t total_crash_entries;/*!< Cumulative crash entries logged */
} FRAM_META_X;

/*************************** FUNCTION PROTOTYPES ******************************/

/* I2C handle defined in main.c — initialised by mx_i2c1_init() */
extern I2C_HandleTypeDef g_hi2c1;

/**
 * \brief  Initialise FRAM driver with the given I2C handle.
 *
 * \param  p_hi2c - Pointer to initialised HAL I2C handle (I2C1, PB6/PB7).
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void fram_init(I2C_HandleTypeDef *p_hi2c);

/**
 * \brief  Write bytes to FRAM at the given address.
 *
 * \param  addr    - 16-bit FRAM address.
 * \param  p_data  - Pointer to data buffer to write.
 * \param  len     - Number of bytes to write.
 *
 * \return HAL_StatusTypeDef - HAL_OK on success.
 *
 * \author MichaelLynnCSU
 */
HAL_StatusTypeDef fram_write(uint16_t addr, uint8_t *p_data, uint16_t len);

/**
 * \brief  Read bytes from FRAM at the given address.
 *
 * \param  addr    - 16-bit FRAM address.
 * \param  p_data  - Pointer to receive buffer.
 * \param  len     - Number of bytes to read.
 *
 * \return HAL_StatusTypeDef - HAL_OK on success.
 *
 * \author MichaelLynnCSU
 */
HAL_StatusTypeDef fram_read(uint16_t addr, uint8_t *p_data, uint16_t len);

/**
 * \brief  Save metadata (write pointers, entry counts) to FRAM.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void fram_save_meta(void);

/**
 * \brief  Load metadata from FRAM into RAM cache.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void fram_load_meta(void);

/**
 * \brief  Log a trinity crash event to the FRAM crash partition.
 *
 * \param  error_code  - TRINITY_ERROR_E value cast to uint8_t.
 * \param  boot_count  - Current boot counter value.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void fram_log_crash(uint8_t error_code, uint8_t boot_count);

/**
 * \brief  Return total number of crash entries ever logged.
 *
 * \param  void
 *
 * \return uint32_t - Cumulative crash entry count.
 *
 * \author MichaelLynnCSU
 */
uint32_t fram_get_total_crashes(void);

#endif /* INCLUDE_FRAM_DRIVER_H_ */
