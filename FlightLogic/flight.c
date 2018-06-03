#include <msp430.h>
#include "MpuUtil.h"
#include "MplUtil.h"
#include "MTK3339.h"
#include "uart.h"
#include "collection.h"
#include "transmission.h"
#include "LED.h"
#include "flightTiming.h"
#include <stdio.h>
#include <string.h>

#define NO_SAVING

void wait_for_launch(void);
void upwards(void);
void downwards(void);
void landed(void);
uint8_t detectLaunch(void);
uint8_t detectApogee(void);
uint8_t detectLanding(void);

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
	config.sampleRate = 0.025;
#ifndef NO_SAVING
	// Store data every 10 seconds, 40 ms * 250 = 10 s
	config.storeRate = 250;
#else
	config.storeRate = 0;
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
	config.sampleRate = 0.025;
#ifndef NO_SAVING
	config.storeRate = 1;
#else
	config.storeRate = 0;
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
	config.sampleRate = 2;
#ifndef NO_SAVING
	config.storeRate = 2;
#else
	config.storeRate = 0;
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
	config.sampleRate = 5;
#ifndef NO_SAVING
	config.storeRate = 0;
#else
	config.storeRate = 0;
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
