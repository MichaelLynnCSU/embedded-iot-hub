/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    ui.h
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   LVGL UI layer public interface for the Smart Home Dashboard.
 *
 * \details Exposes tile creation, layout reflow, sensor-state setters, and
 *          device-online stamping. The parser and main modules use these
 *          accessor functions instead of touching HomeState directly.
 ******************************************************************************/

#ifndef INCLUDE_UI_H_
#define INCLUDE_UI_H_

#include "main.h"
#include "lvgl.h"
#include <stdint.h>

/******************************** CONSTANTS ***********************************/

#define MAX_REEDS          6u    /**< Maximum number of reed sensors supported */
#define HB_TIMEOUT_MS  30000ul  /**< ms before a device is considered offline  */
#define TILE_GAP           4u   /**< Pixel gap between adjacent tiles          */
#define TILE_LEFT_MARGIN   5u   /**< Left edge x-coordinate for left column    */
#define TILE_RIGHT_COL_X 125u   /**< Left edge x-coordinate for right column   */
#define TILE_WIDTH       110u   /**< Standard tile width in pixels              */
#define FULL_TILE_WIDTH  230u   /**< Full-width tile width (motor tile)         */

/******************************* ENUMERATIONS *********************************/

/**
 * \brief Device ID enumeration used to index online-tracking arrays.
 */
typedef enum
{
   eDEV_TEMP  = 0, /*!< Temperature / humidity sensor */
   eDEV_PIR,       /*!< PIR motion sensor             */
   eDEV_LIGHT,     /*!< Smart light                   */
   eDEV_LOCK,      /*!< Smart lock                    */
   eDEV_MOTOR,     /*!< Cooling/heating motor          */
   eDEV_COUNT      /*!< Must be last — array size      */
} DEVICE_ID_E;

/*************************** FUNCTION PROTOTYPES ******************************/

/**
 * \brief  Initialise LVGL display driver and create all dashboard tiles.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_create(void);

/**
 * \brief  Refresh all tile labels and status dots from current sensor state.
 *
 * \param  void
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_update(void);

/**
 * \brief  Reposition all tiles for a given active reed count.
 *
 * \param  n - Number of active reed sensors (1..MAX_REEDS).
 *
 * \return void
 *
 * \details Hides tiles for unused reed slots; shows and repositions the rest.
 *          Safe to call at any time from the main loop.
 *
 * \author MichaelLynnCSU
 */
void ui_reflow(int n);

/**
 * \brief  Return the current active reed count.
 *
 * \param  void
 *
 * \return uint8_t - Number of active reed sensors.
 *
 * \author MichaelLynnCSU
 */
uint8_t ui_get_reed_count(void);

/**
 * \brief  Set the active reed sensor count.
 *
 * \param  count - New reed count (will be clamped to MAX_REEDS).
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_set_reed_count(uint8_t count);

/**
 * \brief  Record the current tick as the last-seen time for a device.
 *
 * \param  dev_id - Device to stamp (must be < eDEV_COUNT).
 * \param  tick   - HAL_GetTick() value to record.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_stamp_dev_online(DEVICE_ID_E dev_id, uint32_t tick);

/**
 * \brief  Record the current tick as the last-seen time for a reed sensor.
 *
 * \param  slot - Zero-based reed index (must be < MAX_REEDS).
 * \param  tick - HAL_GetTick() value to record.
 *
 * \return void
 *
 * \author MichaelLynnCSU
 */
void ui_stamp_reed_online(uint8_t slot, uint32_t tick);

/* ---- Sensor state setters ---- */

/** \brief Set temperature value (degrees C). \param val - New value. */
void ui_set_temp(uint8_t val);

/** \brief Set humidity value (percent). \param val - New value. */
void ui_set_hum(uint8_t val);

/** \brief Set cumulative PIR event count. \param val - New count. */
void ui_set_pir_count(uint32_t val);

/** \brief Set PIR battery percent. \param val - 0-100. */
void ui_set_pir_batt(uint8_t val);

/** \brief Set reed door state. \param slot - 0-based index. \param state - 0=closed,1=open. */
void ui_set_reed_state(uint8_t slot, uint8_t state);

/** \brief Set reed battery percent. \param slot - 0-based index. \param batt - percent or -1. */
void ui_set_reed_batt(uint8_t slot, int8_t batt);

/** \brief Set reed BLE age in seconds. \param slot - 0-based index. \param age - seconds. */
void ui_set_reed_age(uint8_t slot, uint16_t age);

/** \brief Set smart light state. \param val - 0=off, 1=on. */
void ui_set_light(uint8_t val);

/** \brief Set smart lock state. \param val - 0=unlocked, 1=locked. */
void ui_set_lock(uint8_t val);

/** \brief Set lock battery percent. \param val - 0-100 or -1. */
void ui_set_lock_batt(int8_t val);

/** \brief Set motor state. \param val - 0=off,1=cooling,2=heating. */
void ui_set_motor(uint8_t val);

/**
 * \brief  Return the current online flag for a named device.
 *
 * \param  dev_id - Device to query (must be < eDEV_COUNT).
 *
 * \return uint8_t - 1 if online, 0 if offline or unknown.
 *
 * \author MichaelLynnCSU
 */
uint8_t ui_get_dev_online(DEVICE_ID_E dev_id);

#endif /* INCLUDE_UI_H_ */
