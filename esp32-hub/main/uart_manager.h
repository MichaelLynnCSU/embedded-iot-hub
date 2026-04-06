/******************************************************************************
 * \file uart_manager.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief UART manager public interface for ESP32 hub node.
 *
 * \details Receives JSON temperature data from STM32 blue pill over
 *          UART2. See uart_manager.c for implementation details.
 ******************************************************************************/

#ifndef INCLUDE_UART_MANAGER_H_
#define INCLUDE_UART_MANAGER_H_

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize the UART manager.
 *  \return void */
void uart_manager_init(void);

/** \brief UART manager FreeRTOS task — receives and parses STM32 JSON.
 *  \return void */
void uart_manager_task(void);

/** \brief Get last received average temperature from STM32.
 *  \return int - Average temperature in Celsius. */
int uart_get_avg_temp(void);

#endif /* INCLUDE_UART_MANAGER_H_ */
