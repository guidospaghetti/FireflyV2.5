#include <msp430.h>
#include "MpuUtil.h"
#include "MplUtil.h"
#include "MTK3339.h"
#include "uart.h"
#include "collection.h"
#include "transmission.h"
#include "LED.h"
#include <stdio.h>
#include <string.h>

#define ACCEL_HIST_SIZE	15
#define ALT_HIST_SIZE	15

void wait_for_launch(void);
void upwards(void);
void downwards(void);
void landed(void);
uint8_t detectLaunch(void);
uint8_t detectApogee(void);
uint8_t detectLanding(void);

float accelHist[15] = {0};
float altHist[15] = {0};
float* accelPos = accelHist;
float* altPos = altHist;
float* accelEnd = accelHist + ACCEL_HIST_SIZE;
float* altEnd = accelHist + ALT_HIST_SIZE;

void run_flight(void) {
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
	config.rate = 40;
	setup_collection(&config);
	collection_t data;
	while (1) {
		collect(&data);
		SWITCH_RED();
		sendString(1, "%.3f\t%.3f\t%.3f\r\n", data.data.altitude, data.data.temp, data.data.accel.z);
		// Assume the payload will be placed flat, GPS down
		// Z axis will be facing upwards, towards the sky
//		*accelPos = data.data.accel.z;
//		*altPos = data.data.altitude;
//		accelPos++;
//		altPos++;
//		if (accelPos == accelEnd) {
//			accelPos = accelHist;
//		}
//		if (altPos == altEnd) {
//			altPos = altHist;
//		}
	}
}

void upwards(void) {

}

void downwards(void) {

}

void landed(void) {

}
