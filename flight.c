#include <msp430.h>
#include "MpuUtil.h"
#include "MplUtil.h"
#include "MTK3339.h"
#include "hal_uart.h"
#include "collection.h"
#include "transmission.h"

void wait_for_launch(void);
void upwards(void);
void downwards(void);
void landed(void);

float accelHist[15] = {0};
float altHist[10] = {0};

void run_flight(void) {
	wait_for_launch();
	upwards();
	downwards();
	landed();
}

void wait_for_launch() {
	// Get full data collection but use LPM of accelerometer
}

void upwards(void) {

}

void downwards(void) {

}

void landed(void) {

}
