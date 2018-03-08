/*
 * uart.c
 *
 *  Created on: Mar 2, 2018
 *      Author: Aaron
 */

#include <stdint.h>
#include "hal_uart.h"

void sendUARTA0(char* bytes, uint32_t length) {
	volatile int i;
	for (i = 0; i < length; i++) {
		while(hal_UART_SpaceAvailable(0) == 0);
		hal_UART_TxByte((uint8_t)*bytes, 0);
		bytes++;
	}
}

void sendUARTA1(char* bytes, uint32_t length) {
	volatile int i;
	for (i = 0; i < length; i++) {
		while(hal_UART_SpaceAvailable(1) == 0);
		hal_UART_TxByte((uint8_t)*bytes, 1);
		bytes++;
	}
}
