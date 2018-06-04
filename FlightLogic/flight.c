#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include "settings.h"
#include "MpuUtil.h"
#include "MplUtil.h"
#include "MTK3339.h"
#include "uart.h"
#include "collection.h"
#include "transmission.h"
#include "LED.h"
#include "flightTiming.h"


void wait_for_launch(void);
void upwards(void);
void downwards(void);
void landed(void);

void run_flight(void) {
	initDataStorage();
	wait_for_launch();
	upwards();
	downwards();
	landed();
}

void wait_for_launch() {
	// Get full data collection but use LPM of accelerometer
	collectionConfig_t config;
	// Set the LPM of the MPU6050 to 40 Hz
	config.lpm = 4;
	// Get data at 40 Hz
	config.sampleRate = WAIT_FOR_LAUNCH_SAMPLE_RATE;
#ifndef NO_SAVING
	// Store data every 10 seconds, 40 ms * 250 = 10 s
	config.storeRate = WAIT_FOR_LAUNCH_STORAGE_RATE;
#else
	config.storeRate = 0;
#endif
#ifndef NO_TRANSMITTING
	config.transmitRate = WAIT_FOR_LAUNCH_TRANSMIT_RATE;
#else
	config.transmitRate = 0;
#endif
	setup_collection(&config);
	initDataStorage();
	collection_t data;
	while (1) {
		RED_OFF();
		collect(&data, 0);
		RED_ON();
		flightState_t flightState = update(&data);
		if (flightState == UPWARDS) {
			RED_OFF();
			stop_collection();
			break;
		}
	}
}

void upwards(void) {
	collectionConfig_t config;
	config.lpm = 0;
	config.sampleRate = UPWARDS_SAMPLE_RATE;
#ifndef NO_SAVING
	config.storeRate = UPWARDS_STORAGE_RATE;
#else
	config.storeRate = 0;
#endif
#ifndef NO_TRANSMITTING
	config.transmitRate = UPWARDS_TRANSMIT_RATE;
#else
	config.transmitRate = 0;
#endif
	setup_collection(&config);
	collection_t data;
	while (1) {
		RED_OFF();
		collect(&data, 0);
		RED_ON();
		flightState_t flightState = update(&data);
		if (flightState == DOWNWARDS) {
			RED_OFF();
			stop_collection();
			break;
		}
	}
}

void downwards(void) {
	collectionConfig_t config;
	config.lpm = 0;
	config.sampleRate = DOWNWARDS_SAMPLE_RATE;
#ifndef NO_SAVING
	config.storeRate = DOWNWARDS_STORAGE_RATE;
#else
	config.storeRate = 0;
#endif
#ifndef NO_TRANSMITTING
	config.transmitRate = DOWNWARDS_TRANSMIT_RATE;
#else
	config.transmitRate = 0;
#endif
	setup_collection(&config);
	collection_t data;
	while (1) {
		RED_OFF();
		collect(&data, 1);
		RED_ON();
		flightState_t flightState = update(&data);
		if (flightState == LANDED) {
			RED_OFF();
			stop_collection();
			break;
		}
	}
}

void landed(void) {
	collectionConfig_t config;
	config.lpm = 4;
	config.sampleRate = LANDED_SAMPLE_RATE;
#ifndef NO_SAVING
	config.storeRate = LANDED_STORAGE_RATE;
#else
	config.storeRate = 0;
#endif
#ifndef NO_TRANSMITTING
	config.transmitRate = LANDED_TRANSMIT_RATE;
#else
	config.transmitRate = 0;
#endif
	setup_collection(&config);
	collection_t data;
	while (1) {
		RED_OFF();
		collect(&data, 0);
		RED_ON();
		flightState_t flightState = update(&data);
	}
}
