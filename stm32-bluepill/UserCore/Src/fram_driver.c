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
 *            Temp log   (0x0010 - 0x4010) — DHT11 sensor readings
 *            Crash log  (0x4010 - 0x7FFF) — trinity crash entries
 *
 *          Crash-log state (write pointer, total count) has moved to
 *          ring_buffer.c.  FRAM_SaveMeta / FRAM_LoadMeta now accept those
 *          values as parameters; the driver no longer owns them.
 *
 *          FRAM_Init() no longer calls FRAM_LoadMeta().  The ring-buffer
 *          layer calls rb_crashlog_init() after FRAM is ready, which pulls
 *          both crash fields and the temp fields in one read.
 ******************************************************************************/

#include "fram_driver.h"
#include "main.h"
#include <string.h>

/************************** STATIC (PRIVATE) DATA *****************************/

static I2C_HandleTypeDef *g_p_fram_i2c       = NULL; /**< I2C handle            */
static uint16_t            g_temp_write_ptr   = 0u;   /**< Temp log write offset */
static uint32_t            g_total_temp_entries = 0ul; /**< Cumulative temp count */

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
 * \details Stores the handle only.  Meta-data loading is deferred — the
 *          ring-buffer layer calls rb_crashlog_init() (which internally calls
 *          FRAM_LoadMeta) once FRAM availability has been confirmed.
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
   /* NOTE: FRAM_LoadMeta() deliberately NOT called here.
    *       rb_crashlog_init() owns that responsibility.            */
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
 * \details Caller supplies crash-log fields; this layer owns only the temp
 *          fields internally.  All four values are packed into one 8-byte
 *          write so the meta block stays backward compatible.
 *
 * \param  temp_write_ptr  - Current temp-log write offset (internal copy
 *                           updated from g_temp_write_ptr by FRAM_LogTemp).
 * \param  total_temp      - Cumulative temp entry count.
 * \param  crash_write_ptr - Crash-log write offset owned by ring_buffer.c.
 * \param  total_crash     - Cumulative crash count owned by ring_buffer.c.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void FRAM_SaveMeta(uint16_t temp_write_ptr,
                   uint32_t total_temp,
                   uint16_t crash_write_ptr,
                   uint32_t total_crash)
{
   uint8_t meta[8] = {0}; /**< Packed metadata buffer */

   meta[0] = (uint8_t)(temp_write_ptr & 0xFFu);
   meta[1] = (uint8_t)(temp_write_ptr >> 8u);
   meta[2] = (uint8_t)(total_temp & 0xFFu);
   meta[3] = (uint8_t)((total_temp >> 8u) & 0xFFu);
   meta[4] = (uint8_t)(crash_write_ptr & 0xFFu);
   meta[5] = (uint8_t)(crash_write_ptr >> 8u);
   meta[6] = (uint8_t)(total_crash & 0xFFu);
   meta[7] = (uint8_t)((total_crash >> 8u) & 0xFFu);

   (void)FRAM_Write(FRAM_META_ADDR, meta, (uint16_t)sizeof(meta));
}

/**
 * \brief  Load metadata from FRAM into caller-supplied out-parameters.
 *
 * \details Any out-pointer may be NULL — the corresponding field is simply
 *          not written.  On read failure all non-NULL out-params are zeroed.
 *
 * \param  p_temp_write_ptr  - Out: temp-log write offset  (may be NULL).
 * \param  p_total_temp      - Out: cumulative temp count  (may be NULL).
 * \param  p_crash_write_ptr - Out: crash-log write offset (may be NULL).
 * \param  p_total_crash     - Out: cumulative crash count (may be NULL).
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void FRAM_LoadMeta(uint16_t *p_temp_write_ptr,
                   uint32_t *p_total_temp,
                   uint16_t *p_crash_write_ptr,
                   uint32_t *p_total_crash)
{
   uint8_t           meta[8] = {0}; /**< Packed metadata buffer */
   HAL_StatusTypeDef status  = HAL_ERROR;

   status = FRAM_Read(FRAM_META_ADDR, meta, (uint16_t)sizeof(meta));

   if (HAL_OK == status)
   {
      if (NULL != p_temp_write_ptr)
      {
         *p_temp_write_ptr  = (uint16_t)(meta[0] | ((uint16_t)meta[1] << 8u));
      }
      if (NULL != p_total_temp)
      {
         *p_total_temp      = (uint32_t)(meta[2] | ((uint32_t)meta[3] << 8u));
      }
      if (NULL != p_crash_write_ptr)
      {
         *p_crash_write_ptr = (uint16_t)(meta[4] | ((uint16_t)meta[5] << 8u));
      }
      if (NULL != p_total_crash)
      {
         *p_total_crash     = (uint32_t)(meta[6] | ((uint32_t)meta[7] << 8u));
      }
   }
   else
   {
      if (NULL != p_temp_write_ptr)  { *p_temp_write_ptr  = 0u;  }
      if (NULL != p_total_temp)      { *p_total_temp      = 0ul; }
      if (NULL != p_crash_write_ptr) { *p_crash_write_ptr = 0u;  }
      if (NULL != p_total_crash)     { *p_total_crash     = 0ul; }
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

      /* Pass crash fields as 0/0 — ring_buffer.c will call FRAM_SaveMeta
       * with the correct crash values when it logs a crash entry.
       * Here we only need to persist the updated temp fields; the crash
       * fields in the meta block will be overwritten correctly next time
       * rb_crashlog_push() runs.  To avoid stale crash data, ring_buffer
       * calls FRAM_SaveMeta after every crash push anyway.               */
      FRAM_SaveMeta(g_temp_write_ptr,
                    g_total_temp_entries,
                    0u,
                    0ul);
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
