/*
 * storage.c
 *
 *  Created on: Jun 2, 2018
 *      Author: Aaron
 */

#include <msp430f5529.h>
#include <stdint.h>
#include "storage.h"


#define FLASH3_ORIGIN	0x5400
#define FLASH3_LENGTH	0xAB80
#define FLASH4_ORIGIN	0x1A200
#define FLASH4_LENGTH	0xA200

typedef enum flashSection_t {
	FLASH3,
	FLASH4,
	FULL
} flashSection_t;

uint8_t* flash3PtrR = (uint8_t*)FLASH3_ORIGIN;
uint8_t* flash3PtrW = (uint8_t*)FLASH3_ORIGIN;
uint8_t* flash4PtrR = (uint8_t*)FLASH4_ORIGIN;
uint8_t* flash4PtrW = (uint8_t*)FLASH4_ORIGIN;

flashSection_t section = FLASH3;
uint16_t writeCounter = 0;
uint16_t readCounter = 0;

void save(collection_t* data) {

	if (section == FLASH3 && (writeCounter + COLLECTION_SIZE) >= FLASH3_LENGTH) {
		writeCounter = 0;
		section = FLASH4;
	}
	else if (section == FLASH4 && (writeCounter + COLLECTION_SIZE) >= FLASH4_LENGTH) {
		section = FULL;
	}

	switch (section) {
	case FLASH3:
	{
		FCTL3 = FWKEY;
		FCTL1 = FWKEY + WRT;
		volatile uint8_t i;
		for (i = 0; i < COLLECTION_SIZE; i++) {
			*flash3PtrW++ = data->bytes[i];
			writeCounter++;
		}
		FCTL1 = FWKEY;
		FCTL3 = FWKEY + LOCK;
	}
		break;
	case FLASH4:
	{
		FCTL3 = FWKEY;
		FCTL1 = FWKEY + WRT;
		volatile uint8_t i;
		for (i = 0; i < COLLECTION_SIZE; i++) {
			*flash4PtrW++ = data->bytes[i];
		}
		FCTL1 = FWKEY;
		FCTL3 = FWKEY + LOCK;
	}
		break;
	case FULL:
		break;
	default:
		break;
	}
}

void readNext(collection_t* data) {

	if (section == FLASH3 && (readCounter + COLLECTION_SIZE) >= FLASH3_LENGTH) {
		readCounter = 0;
		section = FLASH4;
	}
	else if (section == FLASH4 && (readCounter + COLLECTION_SIZE) >= FLASH4_LENGTH) {
		section = FULL;
	}

	switch (section) {
	case FLASH3:
	{
		volatile uint8_t i;
		for (i = 0; i < COLLECTION_SIZE; i++) {
			data->bytes[i] = *flash3PtrR++;
			readCounter++;
		}

	}
		break;
	case FLASH4:
	{
		volatile uint8_t i;
		for (i = 0; i < COLLECTION_SIZE; i++) {
			data->bytes[i] = *flash4PtrR++;
			readCounter++;
		}
	}
		break;
	case FULL:
		break;
	default:
		break;
	}
}
