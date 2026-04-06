/******************************************************************************
 * Copyright (c) 2024 MichaelLynnCSU
 * All Rights Reserved
 *
 * \file    parser.h
 * \author  MichaelLynnCSU
 * \date    01-01-2024
 *
 * \brief   UART telemetry parser public interface.
 *
 * \details Declares the entry point for parsing line-oriented ASCII messages
 *          received from the ESP32 over USART1. Each message is of the form
 *          "ID:value[,value...]" terminated by CR or LF.
 ******************************************************************************/

#ifndef INCLUDE_PARSER_H_
#define INCLUDE_PARSER_H_

/******************************** CONSTANTS ***********************************/

#define UART_LINE_LEN  128u  /**< Max UART line length including NUL         */

/*************************** FUNCTION PROTOTYPES ******************************/

/**
 * \brief  Parse one null-terminated ASCII telemetry line from the ESP32.
 *
 * \param  p_line - Null-terminated message string, e.g. "TEMP:24,55".
 *
 * \return void
 *
 * \details Updates the shared HomeState and triggers UI_Reflow() when the
 *          reed count changes. Must be called from main-loop context only.
 *
 * \author MichaelLynnCSU
 */
void parser_process_line(const char *p_line);

#endif /* INCLUDE_PARSER_H_ */
