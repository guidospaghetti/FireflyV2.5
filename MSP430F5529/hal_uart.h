/*
 * hal_uart.h
 *
 *  Created on: Feb 23, 2018
 *      Author: Aaron
 */

#ifndef HAL_UART_H_
#define HAL_UART_H_

#include <stdint.h>

uint8_t lastByte0, lastByte1;

void hal_UART_Init(void);
void hal_UART_TxByte(uint8_t byte, uint8_t channel);
uint8_t hal_UART_RxByte(uint8_t channel);
uint8_t hal_UART_DataAvailable(uint8_t channel);
uint8_t hal_UART_SpaceAvailable(uint8_t channel);

#endif /* HAL_UART_H_ */
