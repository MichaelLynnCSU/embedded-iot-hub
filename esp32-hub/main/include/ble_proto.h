/******************************************************************************
 * \file ble_proto.h
 * \brief BLE advertisement and manufacturer data protocol definitions
 *
 * \details Defines BLE packet layout, manufacturer data indexes,
 *          and advertisement type constants used by ble_scan.c.
 *
 *          This file MUST only contain protocol-level definitions.
 *          No system configuration or timing parameters belong here.
 ******************************************************************************/

#ifndef BLE_PROTO_H_
#define BLE_PROTO_H_

/************************ BLE ADVERTISEMENT TYPES ****************************/

/** \brief BLE advertising data types (AD structures) */
#define ADV_TYPE_SHORT_NAME         0x08  /**< shortened local name */
#define ADV_TYPE_FULL_NAME          0x09  /**< complete local name   */

/************************ MANUFACTURER DATA (PIR) ****************************/

/** \brief PIR manufacturer payload format */
#define MFG_PIR_MIN_LEN             6     /**< minimum PIR payload length */
#define MFG_PIR_BATT_IDX            6     /**< battery level byte index */
#define MFG_PIR_OCCUPIED_IDX        7     /**< occupancy flag byte index */

/************************ MANUFACTURER DATA (REED) ****************************/

/** \brief Reed sensor manufacturer payload format */
#define MFG_REED_STATE_IDX          1     /**< reed open/close state */
#define MFG_REED_BATT_IDX           2     /**< reed battery level */

/************************ MANUFACTURER DATA (LIGHT) ***************************/

/** \brief Light sensor manufacturer payload format */
#define MFG_LIGHT_STATE_IDX         2     /**< light state byte */
#define MFG_LIGHT_MIN_LEN           2     /**< minimum payload length */

/************************ MANUFACTURER DATA (LOCK) ****************************/

/** \brief Lock sensor manufacturer payload format */
#define MFG_LOCK_STATE_IDX          1     /**< lock state byte */
#define MFG_LOCK_BATT_IDX           2     /**< lock battery level */
#define MFG_LOCK_MIN_LEN            3     /**< minimum payload length */

/************************ PIR ENCODING FORMAT *********************************/

/** \brief PIR count encoding inside advertisement payload */
#define PIR_COUNT_BYTE0             2     /**< MSB */
#define PIR_COUNT_BYTE1             3
#define PIR_COUNT_BYTE2             4
#define PIR_COUNT_BYTE3             5     /**< LSB */

/************************ GENERAL PROTOCOL CONSTANTS **************************/

/** \brief Maximum representable age value in device protocol */
#define AGE_MAX_VALUE               0xFFFE /**< max reportable age value */

#endif /* BLE_PROTO_H_ */
