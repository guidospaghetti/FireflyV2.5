/*
 * uart.c
 *
 *  Created on: Mar 2, 2018
 *      Author: Aaron
 */

#include <stdint.h>
#include "hal_uart.h"

#define HANDLER_ARRAY_SIZE	5

void (*handler_A0[HANDLER_ARRAY_SIZE])(uint8_t) = {0};
void (*handler_A1[HANDLER_ARRAY_SIZE])(uint8_t) = {0};
uint8_t A0pos = 0;
uint8_t A1pos = 0;

void sendUARTA0(char* bytes, uint32_t length) {
	__disable_interrupt();
	volatile int i;
	for (i = 0; i < length; i++) {
		while(hal_UART_SpaceAvailable(0) == 0);
		hal_UART_TxByte((uint8_t)*bytes, 0);
		bytes++;
	}
	__enable_interrupt();
}

void sendUARTA1(char* bytes, uint32_t length) {
	__disable_interrupt();
	volatile int i;
	for (i = 0; i < length; i++) {
		while(hal_UART_SpaceAvailable(1) == 0);
		hal_UART_TxByte((uint8_t)*bytes, 1);
		bytes++;
	}
	__enable_interrupt();
}

void addRxHandler(void (*handler)(uint8_t), uint8_t channel) {
	switch (channel) {
	case 0:
		handler_A0[A0pos] = handler;
		A0pos++;
		if (A0pos == HANDLER_ARRAY_SIZE) {
			A0pos = 0;
		}
		break;
	case 1:
		handler_A1[A1pos] = handler;
		A1pos++;
		if (A1pos == HANDLER_ARRAY_SIZE) {
			A1pos = 0;
		}
		break;
	default:
		break;
	}
}

void UARTA0_handler(uint8_t byte) {
	volatile uint8_t i;
	for (i = 0; i < HANDLER_ARRAY_SIZE; i++) {
		if (handler_A0[i] == 0) {
			continue;
		}
		(*handler_A0[i])(byte);
	}
}

void UARTA1_handler(uint8_t byte) {
	volatile uint8_t i;
	for (i = 0; i < HANDLER_ARRAY_SIZE; i++) {
		if (handler_A1[i] == 0) {
			continue;
		}
		(*handler_A1[i])(byte);
	}
}
