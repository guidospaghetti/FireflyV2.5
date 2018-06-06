/*
 * receiver_uart.h
 *
 *  Created on: Jun 6, 2018
 *      Author: Aaron
 */

#ifndef RECEIVER_UART_H_
#define RECEIVER_UART_H_

#include "collection.h"
#include <stdint.h>

void sendUART(char* bytes, uint8_t len);
void sendString(char* bytes, ...);
void sendCollection(collection_t* collection);

#endif /* RECEIVER_UART_H_ */
