/******************************************************************************
 * Copyright (c) 2025 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    fram_driver.c
 * \author  MichaelLynnCSU
 * \date    01-01-2025
 *
 * \brief   FM24CL16B FRAM driver — BluePill (STM32F103).
 *
 * \details Manages two partitions:
 *            Temp log  (0x0010 - 0x4010) — DHT11 sensor readings
 *            Crash log (0x4010 - 0x7FFF) — trinity crash entries
 ******************************************************************************/

#include "fram_driver.h"
#include "main.h"
#include <string.h>

/************************** STATIC (PRIVATE) DATA *****************************/

static I2C_HandleTypeDef *g_p_fram_i2c        = NULL; /**< I2C handle           */
static uint16_t            g_temp_write_ptr    = 0u;   /**< Temp log write offset*/
static uint32_t            g_total_temp_entries  = 0ul; /**< Cumulative temp count*/
static uint16_t            g_crash_write_ptr   = 0u;   /**< Crash log write offs */
static uint32_t            g_total_crash_entries = 0ul; /**< Cumulative crash cnt */

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
   uint8_t  crc = 0xFFu; /**< CRC accumulator */
   uint16_t i   = 0u;    /**< Byte index      */
   uint8_t  j   = 0u;    /**< Bit index       */

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
 * \param  hi2c - Pointer to initialised HAL I2C handle.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void FRAM_Init(I2C_HandleTypeDef *hi2c)
{
   if (NULL == hi2c) { return; }
   g_p_fram_i2c = hi2c;
   FRAM_LoadMeta();
}

/**
 * \brief  Write bytes to FRAM at the given address.
 *
 * \param  addr - 16-bit FRAM address.
 * \param  data - Pointer to data to write.
 * \param  len  - Number of bytes.
 *
 * \return HAL_StatusTypeDef - HAL_OK on success.
 *
 * \author MichaelLynnCSU
 */
HAL_StatusTypeDef FRAM_Write(uint16_t addr, uint8_t *data, uint16_t len)
{
   if ((NULL == g_p_fram_i2c) || (NULL == data)) { return HAL_ERROR; }

   return HAL_I2C_Mem_Write(g_p_fram_i2c,
                             (uint16_t)(FRAM_ADDR << 1u),
                             addr,
                             I2C_MEMADD_SIZE_16BIT,
                             data,
                             len,
                             1000u);
}

/**
 * \brief  Read bytes from FRAM at the given address.
 *
 * \param  addr - 16-bit FRAM address.
 * \param  data - Pointer to receive buffer.
 * \param  len  - Number of bytes.
 *
 * \return HAL_StatusTypeDef - HAL_OK on success.
 *
 * \author MichaelLynnCSU
 */
HAL_StatusTypeDef FRAM_Read(uint16_t addr, uint8_t *data, uint16_t len)
{
   if ((NULL == g_p_fram_i2c) || (NULL == data)) { return HAL_ERROR; }

   return HAL_I2C_Mem_Read(g_p_fram_i2c,
                            (uint16_t)(FRAM_ADDR << 1u),
                            addr,
                            I2C_MEMADD_SIZE_16BIT,
                            data,
                            len,
                            1000u);
}

/**
 * \brief  Save metadata to FRAM.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void FRAM_SaveMeta(void)
{
   uint8_t meta[8] = {0}; /**< Packed metadata buffer */

   meta[0] = (uint8_t)(g_temp_write_ptr & 0xFFu);
   meta[1] = (uint8_t)(g_temp_write_ptr >> 8u);
   meta[2] = (uint8_t)(g_total_temp_entries & 0xFFu);
   meta[3] = (uint8_t)((g_total_temp_entries >> 8u) & 0xFFu);
   meta[4] = (uint8_t)(g_crash_write_ptr & 0xFFu);
   meta[5] = (uint8_t)(g_crash_write_ptr >> 8u);
   meta[6] = (uint8_t)(g_total_crash_entries & 0xFFu);
   meta[7] = (uint8_t)((g_total_crash_entries >> 8u) & 0xFFu);

   (void)FRAM_Write(FRAM_META_ADDR, meta, (uint16_t)sizeof(meta));
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
void FRAM_LoadMeta(void)
{
   uint8_t           meta[8] = {0}; /**< Packed metadata buffer */
   HAL_StatusTypeDef status  = HAL_ERROR;

   status = FRAM_Read(FRAM_META_ADDR, meta, (uint16_t)sizeof(meta));

   if (HAL_OK == status)
   {
      g_temp_write_ptr      = (uint16_t)(meta[0] | ((uint16_t)meta[1] << 8u));
      g_total_temp_entries  = (uint32_t)(meta[2] | ((uint32_t)meta[3] << 8u));
      g_crash_write_ptr     = (uint16_t)(meta[4] | ((uint16_t)meta[5] << 8u));
      g_total_crash_entries = (uint32_t)(meta[6] | ((uint32_t)meta[7] << 8u));
   }
   else
   {
      g_temp_write_ptr      = 0u;
      g_total_temp_entries  = 0ul;
      g_crash_write_ptr     = 0u;
      g_total_crash_entries = 0ul;
   }
}

/**
 * \brief  Log a temperature/humidity reading to the FRAM temp partition.
 *
 * \param  sensor_id - Sensor slot index.
 * \param  temp      - Temperature in degrees C.
 * \param  hum       - Humidity percent.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void FRAM_LogTemp(uint16_t sensor_id, uint8_t temp, uint8_t hum)
{
   TEMP_LOG_ENTRY_X entry = {0}; /**< Log entry to write  */
   uint16_t         addr  = 0u;  /**< Computed write addr */

   entry.timestamp   = HAL_GetTick();
   entry.pin         = (uint8_t)sensor_id;
   entry.temperature = temp;
   entry.humidity    = hum;
   entry.crc         = calc_crc8((uint8_t *)&entry,
                                  (uint16_t)(sizeof(entry) - 1u));

   addr = (uint16_t)(FRAM_TEMPLOG_ADDR + g_temp_write_ptr);

   if (HAL_OK == FRAM_Write(addr, (uint8_t *)&entry, (uint16_t)sizeof(entry)))
   {
      g_temp_write_ptr += (uint16_t)sizeof(entry);

      if ((g_temp_write_ptr + (uint16_t)sizeof(entry)) >= FRAM_TEMPLOG_SIZE)
      {
         g_temp_write_ptr = 0u;
      }

      g_total_temp_entries++;
      FRAM_SaveMeta();
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
void FRAM_LogCrash(uint8_t error_code, uint8_t boot_count)
{
   CRASH_LOG_ENTRY_X entry = {0}; /**< Crash entry to write */
   uint16_t          addr  = 0u;  /**< Computed write addr  */

   entry.timestamp  = HAL_GetTick();
   entry.error_code = error_code;
   entry.boot_count = boot_count;
   entry.crc        = calc_crc8((uint8_t *)&entry,
                                 (uint16_t)(sizeof(entry) - 1u));

   addr = (uint16_t)(FRAM_CRASHLOG_ADDR + g_crash_write_ptr);

   if (HAL_OK == FRAM_Write(addr, (uint8_t *)&entry, (uint16_t)sizeof(entry)))
   {
      g_crash_write_ptr += (uint16_t)sizeof(entry);

      if ((g_crash_write_ptr + (uint16_t)sizeof(entry)) >= FRAM_CRASHLOG_SIZE)
      {
         g_crash_write_ptr = 0u;
      }

      g_total_crash_entries++;
      FRAM_SaveMeta();
   }
}

/**
 * \brief  Return current temp log write pointer.
 *
 * \param  void
 *
 * \return uint16_t - Byte offset of next write in temp partition.
 *
 * \author MichaelLynnCSU
 */
uint16_t FRAM_GetWritePtr(void)
{
   return g_temp_write_ptr;
}

/**
 * \brief  Return total number of temp entries logged.
 *
 * \param  void
 *
 * \return uint32_t - Cumulative entry count.
 *
 * \author MichaelLynnCSU
 */
uint32_t FRAM_GetTotalEntries(void)
{
   return g_total_temp_entries;
}

/**
 * \brief  Return total number of crash entries logged.
 *
 * \param  void
 *
 * \return uint32_t - Cumulative crash entry count.
 *
 * \author MichaelLynnCSU
 */
uint32_t FRAM_GetTotalCrashes(void)
{
   return g_total_crash_entries;
}
