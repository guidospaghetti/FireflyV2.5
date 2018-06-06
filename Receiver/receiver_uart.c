/*
 * receiver_uart.c
 *
 *  Created on: Jun 6, 2018
 *      Author: Aaron
 */

#include <msp430f5529.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include "receiver_uart.h"
#include "collection.h"

void sendUART(char* bytes, uint8_t len) {
	__disable_interrupt();
	volatile uint8_t i;
	for (i = 0; i < len; i++) {
		while(UCA1IFG & UCTXIFG == 0);
		UCA1TXBUF = *bytes;
		bytes++;
	}
	__enable_interrupt();
}

void sendString(char* bytes, ...) {
	va_list va;
	va_start(va, bytes);
	char buffer[256];
	uint8_t len = vsprintf(buffer, bytes, va);

	sendUART(buffer, len);
}

void sendCollection(collection_t* collection) {
	sendString("%d,%d,%d,%f,%f,%c,%f,%c,%f,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
				collection->data.gps.time.day,
				collection->data.gps.time.month,
				collection->data.gps.time.year,
				collection->data.gps.time.utcTime,
				collection->data.gps.location.latitude,
				collection->data.gps.location.NS,
				collection->data.gps.location.longitude,
				collection->data.gps.location.EW,
				collection->data.gps.location.altitude,
				collection->data.gps.fix,
				collection->data.gps.status,
				collection->data.accel.x,
				collection->data.accel.y,
				collection->data.accel.z,
				collection->data.gyro.x,
				collection->data.gyro.y,
				collection->data.gyro.z,
				collection->data.altitude,
				collection->data.temp);
}
