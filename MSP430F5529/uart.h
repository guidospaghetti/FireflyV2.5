/*
 * uart.h
 *
 *  Created on: Mar 2, 2018
 *      Author: Aaron
 */

#ifndef UART_H_
#define UART_H_

#include "hal_uart.h"

void sendUARTA0(char* bytes, uint32_t length);
void sendUARTA1(char* bytes, uint32_t length);
void addRxHandler(void (*handler)(uint8_t), uint8_t channel);
void UARTA0_handler(uint8_t byte);
void UARTA1_handler(uint8_t byte);

#endif /* UART_H_ */
