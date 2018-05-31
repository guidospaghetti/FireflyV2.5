#include "flightTiming.h"

#define ACCEL_HIST_SIZE	15
#define ALT_HIST_SIZE	15

float accelHist[ACCEL_HIST_SIZE] = {0};
float altHist[ALT_HIST_SIZE] = {0};
float* accelPos = accelHist;
float* altPos = altHist;
float* accelEnd = accelHist;
float* altEnd = altHist;

void initDataStorage(void) {
	accelEnd += ACCEL_HIST_SIZE;
	altEnd += ALT_HIST_SIZE;
}

uint8_t update(collection_t* data) {

}
