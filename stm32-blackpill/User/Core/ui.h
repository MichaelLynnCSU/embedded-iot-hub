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
#define MAX_PIRS           4u    /**< Maximum number of per-slot PIR sensors   */
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
   eDEV_PIR,       /*!< PIR motion sensor (aggregate) */
   eDEV_LIGHT,     /*!< Smart light                   */
   eDEV_LOCK,      /*!< Smart lock                    */
   eDEV_MOTOR,     /*!< Cooling/heating motor          */
   eDEV_COUNT      /*!< Must be last — array size      */
} DEVICE_ID_E;

/*************************** FUNCTION PROTOTYPES ******************************/

void ui_create(void);
void ui_update(void);
void ui_reflow(int n);
void ui_reflow_pir(int n);

uint8_t ui_get_reed_count(void);
void    ui_set_reed_count(uint8_t count);

uint8_t ui_get_pir_count_slots(void);
void    ui_set_pir_count_slots(uint8_t count);

void ui_stamp_dev_online(DEVICE_ID_E dev_id, uint32_t tick);
void ui_stamp_reed_online(uint8_t slot, uint32_t tick);
void ui_stamp_pir_online(uint8_t slot, uint32_t tick);

/* ---- Sensor state setters ---- */

void ui_set_temp(uint8_t val);
void ui_set_hum(uint8_t val);
void ui_set_pir_count(uint32_t val);
void ui_set_pir_batt(uint8_t val);
void ui_set_pir_occupied(uint8_t val);

/** \brief Set per-slot PIR motion count. \param slot 0-based. \param val count. */
void ui_set_pir_slot_count(uint8_t slot, uint32_t val);

/** \brief Set per-slot PIR battery percent. \param slot 0-based. \param batt 0-100 or -1. */
void ui_set_pir_slot_batt(uint8_t slot, int8_t batt);

/** \brief Set per-slot PIR BLE age in seconds. \param slot 0-based. \param age seconds. */
void ui_set_pir_slot_age(uint8_t slot, uint16_t age);

/** \brief Set per-slot PIR occupancy flag. \param slot 0-based. \param val 0 or 1. */
void ui_set_pir_slot_occupied(uint8_t slot, uint8_t val);

void ui_set_reed_state(uint8_t slot, uint8_t state);
void ui_set_reed_batt(uint8_t slot, int8_t batt);
void ui_set_reed_age(uint8_t slot, uint16_t age);
void ui_set_light(uint8_t val);
void ui_set_lock(uint8_t val);
void ui_set_lock_batt(int8_t val);
void ui_set_motor(uint8_t val);
void ui_set_motor_batt(int val);

uint8_t ui_get_dev_online(DEVICE_ID_E dev_id);

#endif /* INCLUDE_UI_H_ */
