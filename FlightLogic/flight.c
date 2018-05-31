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
	config.rate = 40;
	setup_collection(&config);
	initDataStorage();
	collection_t data;
	while (1) {
		collect(&data);
		SWITCH_RED();

		//sendString(1, "%.3f\t%.3f\t%.3f\r\n", data.data.altitude, data.data.temp, data.data.accel.z);
		// Assume the payload will be placed flat, GPS down
		// Z axis will be facing upwards, towards the sky
		flightState_t flightState = update(&data);
		if (flightState == UPWARDS) {
			break;
		}
	}
}

void upwards(void) {
	collectionConfig_t config;
	config.lpm = 0;
	config.rate = 40;
	setup_collection(&config);
	collection_t data;
	while (1) {
		collect(&data);
		SWITCH_GREEN();
		flightState_t flightState = update(&data);
		if (flightState == DOWNWARDS) {
			break;
		}
	}
}

void downwards(void) {
	collectionConfig_t config;
	config.lpm = 4;
	config.rate = 2000;
	setup_collection(&config);
	collection_t data;
	while (1) {
		collect(&data);
		SWITCH_GREEN();
		flightState_t flightState = update(&data);
		if (flightState == DOWNWARDS) {
			break;
		}
	}
}

void landed(void) {
	collectionConfig_t config;
	config.lpm = 1;
	config.rate = 5000;
	setup_collection(&config);
	collection_t data;
	while (1) {
		collect(&data);
		SWITCH_GREEN();
		flightState_t flightState = update(&data);
		if (flightState == DOWNWARDS) {
			break;
		}
	}
}
