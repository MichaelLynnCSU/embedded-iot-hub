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
 *   0x0010 -  0x7FFF  Trinity log  (~32KB)    — ring buffer of crash entries
 *
 * \note    Crash log enqueue logic (rb_crashlog_push, rb_crashlog_init, etc.)
 *          lives in ring_buffer.c. This header exposes only raw I2C transport
 *          and meta block persistence. The CRASH_LOG_ENTRY_X and FRAM_META_X
 *          structs are defined here because both layers share them.
 *
 * \warning fram_init() must be called before any other FRAM function.
 *          Call rb_crashlog_init() afterwards to restore crash log state.
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
   uint16_t crash_write_ptr;     /*!< Next write offset in crash log  */
   uint32_t total_crash_entries; /*!< Cumulative crash entries logged */
} FRAM_META_X;

/*************************** FUNCTION PROTOTYPES ******************************/

/* I2C handle defined in main.c — initialised by mx_i2c1_init() */
extern I2C_HandleTypeDef g_hi2c1;

/**
 * \brief  Initialise FRAM driver with the given I2C handle.
 *
 * \param  p_hi2c - Pointer to initialised HAL I2C handle (I2C1, PB6/PB7).
 *
 * \note   Does NOT load metadata. Call rb_crashlog_init() after this to
 *         restore crash log write pointer and entry count from FRAM.
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
 * \brief  Persist crash log state (write pointer, entry count) to FRAM meta block.
 *
 * \param  write_ptr     - Current crash log write offset (owned by ring_buffer.c).
 * \param  total_entries - Cumulative crash entry count (owned by ring_buffer.c).
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void fram_save_meta(uint16_t write_ptr, uint32_t total_entries);

/**
 * \brief  Load metadata from FRAM into RAM cache and return crash log state.
 *
 * \param  p_write_ptr     - Out: crash log write offset. Set to 0 on read failure.
 * \param  p_total_entries - Out: cumulative crash entry count. Set to 0 on failure.
 *
 * \note   Either out-pointer may be NULL if the caller does not need that field.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void fram_load_meta(uint16_t *p_write_ptr, uint32_t *p_total_entries);

#endif /* INCLUDE_FRAM_DRIVER_H_ */
