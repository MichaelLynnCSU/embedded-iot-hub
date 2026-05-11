/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    fram_driver.c
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   FM24CL16B FRAM driver — BlackPill port of BluePill fram_driver.
 *
 * \details Stays as close as possible to the BluePill implementation.
 *          Key differences from BluePill version:
 *            - HAL header: stm32f4xx_hal.h (was stm32f1xx_hal.h)
 *            - Naming: snake_case per EchoStar §6.2.2
 *            - All globals prefixed with g_ per EchoStar §6.5.4
 *
 * \note    Crash log state (write ptr, entry count, enqueue logic) lives in
 *          ring_buffer.c (RING BUFFER 2 — FRAM section). This file owns only
 *          raw I2C transport and the meta block persistence layer.
 ******************************************************************************/

#include "fram_driver.h"
#include <string.h>

/************************** STATIC (PRIVATE) DATA *****************************/

static I2C_HandleTypeDef *g_p_fram_i2c = NULL; /**< I2C handle           */
static FRAM_META_X         g_fram_meta  = {0};  /**< RAM cache of metadata*/

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Initialise FRAM driver with the given I2C handle.
 *
 * \param  p_hi2c - Pointer to initialised HAL I2C handle.
 *
 * \return void
 *
 * \note   Does NOT load metadata — call rb_crashlog_init() after fram_init()
 *         to restore crash log write pointer and entry count from FRAM.
 *
 * \author MichaelLynnCSU
 */
void fram_init(I2C_HandleTypeDef *p_hi2c)
{
   if (NULL == p_hi2c)
   {
      return;
   }

   g_p_fram_i2c = p_hi2c;
}

/**
 * \brief  Write bytes to FRAM at the given address.
 *
 * \param  addr   - 16-bit FRAM address.
 * \param  p_data - Pointer to data to write.
 * \param  len    - Number of bytes.
 *
 * \return HAL_StatusTypeDef - HAL_OK on success.
 *
 * \author MichaelLynnCSU
 */
HAL_StatusTypeDef fram_write(uint16_t addr, uint8_t *p_data, uint16_t len)
{
   if ((NULL == g_p_fram_i2c) || (NULL == p_data))
   {
      return HAL_ERROR;
   }

   return HAL_I2C_Mem_Write(g_p_fram_i2c,
                             (uint16_t)(FRAM_ADDR << 1u),
                             addr,
                             I2C_MEMADD_SIZE_16BIT,
                             p_data,
                             len,
                             1000u);
}

/**
 * \brief  Read bytes from FRAM at the given address.
 *
 * \param  addr   - 16-bit FRAM address.
 * \param  p_data - Pointer to receive buffer.
 * \param  len    - Number of bytes.
 *
 * \return HAL_StatusTypeDef - HAL_OK on success.
 *
 * \author MichaelLynnCSU
 */
HAL_StatusTypeDef fram_read(uint16_t addr, uint8_t *p_data, uint16_t len)
{
   if ((NULL == g_p_fram_i2c) || (NULL == p_data))
   {
      return HAL_ERROR;
   }

   return HAL_I2C_Mem_Read(g_p_fram_i2c,
                            (uint16_t)(FRAM_ADDR << 1u),
                            addr,
                            I2C_MEMADD_SIZE_16BIT,
                            p_data,
                            len,
                            1000u);
}

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
void fram_save_meta(uint16_t write_ptr, uint32_t total_entries)
{
   g_fram_meta.crash_write_ptr     = write_ptr;
   g_fram_meta.total_crash_entries = total_entries;

   (void)fram_write(FRAM_META_ADDR,
                    (uint8_t *)&g_fram_meta,
                    (uint16_t)sizeof(g_fram_meta));
}

/**
 * \brief  Load metadata from FRAM into RAM cache and return crash log state.
 *
 * \param  p_write_ptr     - Out: crash log write offset. Set to 0 on read failure.
 * \param  p_total_entries - Out: cumulative crash entry count. Set to 0 on failure.
 *
 * \return void
 *
 * \note   Either out-pointer may be NULL if the caller does not need that field.
 *
 * \author MichaelLynnCSU
 */
void fram_load_meta(uint16_t *p_write_ptr, uint32_t *p_total_entries)
{
   HAL_StatusTypeDef status = HAL_ERROR; /**< I2C read result */

   status = fram_read(FRAM_META_ADDR,
                      (uint8_t *)&g_fram_meta,
                      (uint16_t)sizeof(g_fram_meta));

   if (HAL_OK == status)
   {
      if (NULL != p_write_ptr)     { *p_write_ptr     = g_fram_meta.crash_write_ptr;     }
      if (NULL != p_total_entries) { *p_total_entries  = g_fram_meta.total_crash_entries; }
   }
   else
   {
      if (NULL != p_write_ptr)     { *p_write_ptr     = 0u;  }
      if (NULL != p_total_entries) { *p_total_entries  = 0ul; }
   }
}

