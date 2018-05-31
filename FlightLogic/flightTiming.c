#include "flightTiming.h"

#define ACCEL_HIST_SIZE		15
#define ALT_HIST_SIZE		15
#define AVERAGE_ACCEL_SIZE	ACCEL_HIST_SIZE - 2
#define AVERAGE_ALT_SIZE	ALT_HIST_SIZE - 2
#define ROLLING_AVERAGE_LEN	3
#define ACCEL_THRESHOLD		8.0f

float accelHist[ACCEL_HIST_SIZE];
float altHist[ALT_HIST_SIZE];
float averageAccel[AVERAGE_ACCEL_SIZE];
float averageAlt[AVERAGE_ALT_SIZE];
float tempAvgAccel[ROLLING_AVERAGE_LEN];
float tempAvgAlt[ROLLING_AVERAGE_LEN];

float* accelPos = accelHist;
float* altPos = altHist;
float* accelEnd = accelHist;
float* altEnd = altHist;

float* avgAccelPos = averageAccel;
float* curAvgAccel;
float* avgAltPos = averageAlt;
float* curAvgAlt;
float* avgAccelEnd = averageAccel;
float* avgAltEnd = averageAlt;

uint64_t measCount = 1;
uint8_t accelFull = 0;
uint8_t altFull = 0;
flightState_t state = ON_PAD;

void updateStorage(collection_t* data);
void incPosRaw(void);
void incPosAvg(void);

void initDataStorage(void) {
	accelEnd += ACCEL_HIST_SIZE;
	altEnd += ALT_HIST_SIZE;
	avgAccelEnd += AVERAGE_ACCEL_SIZE;
	avgAltEnd += AVERAGE_ALT_SIZE;
}

flightState_t update(collection_t* data) {
	updateStorage(data);
	flightState_t ret = ON_PAD;
	switch (state) {
	case ON_PAD:
		if (*curAvgAccel > ACCEL_THRESHOLD) {
			ret = UPWARDS;
		}
		else {
			ret = ON_PAD;
		}
		break;
	case UPWARDS:

		break;
	case DOWNWARDS:
		break;
	case LANDED:
		break;
	default:
		break;
	}

	return ret;
}

void updateStorage(collection_t* data) {
	*accelPos = data->data.accel.z;
	*altPos = data->data.altitude;

	if (measCount % 3 == 0) {
		float sumAccel = 0, sumAlt = 0;
		volatile uint8_t i;
		for (i = 0; i < ROLLING_AVERAGE_LEN; i++) {
			sumAccel += tempAvgAccel[i];
			sumAlt += tempAvgAlt[i];
		}
		*avgAccelPos = sumAccel / (float)ROLLING_AVERAGE_LEN;
		*avgAltPos = sumAlt / (float)ROLLING_AVERAGE_LEN;
		incPosAvg();
	}
	else {
		tempAvgAccel[measCount % 3] = *accelPos;
		tempAvgAlt[measCount % 3] = *altPos;
	}

	incPosRaw();
}

void incPosRaw(void) {
	accelPos++;
	altPos++;
	if (accelPos == accelEnd) {
		accelFull = 1;
		accelPos = accelHist;
	}
	if (altPos == altEnd) {
		altFull = 1;
		altPos = altHist;
	}
}

void incPosAvg(void) {
	curAvgAccel = avgAccelPos;
	curAvgAlt = avgAltPos;
	avgAccelPos++;
	avgAltPos++;
	if (avgAccelPos == avgAccelEnd) {
		avgAccelPos = averageAccel;
	}
	if (avgAltPos == avgAltEnd) {
		avgAltPos = averageAlt;
	}
}
