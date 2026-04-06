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
 *            - Added crash log partition (fram_log_crash)
 *            - Meta block extended with crash write ptr + crash entry count
 *            - All globals prefixed with g_ per EchoStar §6.5.4
 ******************************************************************************/

#include "fram_driver.h"
#include <string.h>

/************************** STATIC (PRIVATE) DATA *****************************/

static I2C_HandleTypeDef *g_p_fram_i2c        = NULL; /**< I2C handle           */
static FRAM_META_X         g_fram_meta         = {0};  /**< RAM cache of metadata*/
static uint16_t            g_crash_write_ptr     = 0u;  /**< Crash log write offset  */
static uint32_t            g_total_crash_entries = 0ul; /**< Cumulative crash count  */

/************************** STATIC (PRIVATE) FUNCTIONS ************************/

/**
 * \brief  Calculate CRC-8 over a data buffer.
 *
 * \param  p_data - Pointer to data bytes.
 * \param  len    - Number of bytes to process.
 *
 * \return uint8_t - Computed CRC-8 value.
 *
 * \author MichaelLynnCSU
 */
static uint8_t calc_crc8(uint8_t *p_data, uint16_t len)
{
   uint8_t  crc = 0xFFu; /**< CRC accumulator, initialised to 0xFF */
   uint16_t i   = 0u;    /**< Byte index                           */
   uint8_t  j   = 0u;    /**< Bit index                            */

   for (i = 0u; i < len; i++)
   {
      crc ^= p_data[i];
      for (j = 0u; j < 8u; j++)
      {
         if (0u != (crc & 0x80u))
         {
            crc = (uint8_t)((crc << 1u) ^ 0x07u);
         }
         else
         {
            crc <<= 1u;
         }
      }
   }

   return crc;
}

/************************** PUBLIC FUNCTIONS ***********************************/

/**
 * \brief  Initialise FRAM driver with the given I2C handle.
 *
 * \param  p_hi2c - Pointer to initialised HAL I2C handle.
 *
 * \return void
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
   fram_load_meta();
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
 * \brief  Save metadata (write pointers, entry counts) to FRAM.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void fram_save_meta(void)
{
   g_fram_meta.crash_write_ptr     = g_crash_write_ptr;
   g_fram_meta.total_crash_entries = g_total_crash_entries;

   (void)fram_write(FRAM_META_ADDR,
                    (uint8_t *)&g_fram_meta,
                    (uint16_t)sizeof(g_fram_meta));
}

/**
 * \brief  Load metadata from FRAM into RAM cache.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void fram_load_meta(void)
{
   HAL_StatusTypeDef status = HAL_ERROR; /**< I2C read result */

   status = fram_read(FRAM_META_ADDR,
                      (uint8_t *)&g_fram_meta,
                      (uint16_t)sizeof(g_fram_meta));

   if (HAL_OK == status)
   {
      g_crash_write_ptr     = g_fram_meta.crash_write_ptr;
      g_total_crash_entries = g_fram_meta.total_crash_entries;
   }
   else
   {
      g_crash_write_ptr     = 0u;
      g_total_crash_entries = 0ul;
   }
}

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
void fram_log_crash(uint8_t error_code, uint8_t boot_count)
{
   CRASH_LOG_ENTRY_X entry = {0}; /**< Crash entry to write */
   uint16_t          addr  = 0u;  /**< Computed write addr  */

   entry.timestamp  = HAL_GetTick();
   entry.error_code = error_code;
   entry.boot_count = boot_count;
   entry.crc        = calc_crc8((uint8_t *)&entry,
                                 (uint16_t)(sizeof(entry) - 1u));

   addr = (uint16_t)(FRAM_CRASHLOG_ADDR + g_crash_write_ptr);

   if (HAL_OK == fram_write(addr, (uint8_t *)&entry, (uint16_t)sizeof(entry)))
   {
      g_crash_write_ptr += (uint16_t)sizeof(entry);

      if ((g_crash_write_ptr + (uint16_t)sizeof(entry)) >= FRAM_CRASHLOG_SIZE)
      {
         g_crash_write_ptr = 0u;
      }
      else
      {
         /* Still space in crash partition */
      }

      g_total_crash_entries++;
      fram_save_meta();
   }
   else
   {
      /* Write failed */
   }
}

/**
 * \brief  Return total number of crash entries ever logged.
 *
 * \param  void
 *
 * \return uint32_t - Cumulative crash entry count.
 *
 * \author MichaelLynnCSU
 */
uint32_t fram_get_total_crashes(void)
{
   return g_total_crash_entries;
}
