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


#endif /* UART_H_ */
