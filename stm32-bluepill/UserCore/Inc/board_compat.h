/******************************************************************************
 * Copyright (c) 2025 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    board_compat.h
 * \author  MichaelLynnCSU
 * \date    01-01-2025
 *
 * \brief   Compatibility shim between CubeMX-generated ISR stubs and the
 *          refactored trinity logging module — BluePill (STM32F103).
 *
 * \details Include this file in stm32f1xx_it.c inside the USER CODE
 *          BEGIN Includes block:
 *            #include "board_compat.h"
 *
 * \warning Re-add this include after every CubeMX code regeneration.
 ******************************************************************************/

#ifndef INCLUDE_BOARD_COMPAT_H_
#define INCLUDE_BOARD_COMPAT_H_

#include "trinity_log.h"
#include "main.h"

/* Route ARM HardFault to trinity handler */

#endif /* INCLUDE_BOARD_COMPAT_H_ */
