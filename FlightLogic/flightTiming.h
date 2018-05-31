#include <stdint.h>
#include "collection.h"

#ifndef FLIGHTTIMING_H
#define FLIGHTTIMING_H

typedef enum flightState_t {
	ON_PAD,
	UPWARDS,
	DOWNWARDS,
	LANDED
} flightState_t;

void initDataStorage(void);
flightState_t update(collection_t* data);

#endif
