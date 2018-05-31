#include "flightTiming.h"

#define ACCEL_HIST_SIZE		15
#define ALT_HIST_SIZE		15
#define AVERAGE_ACCEL_SIZE	ACCEL_HIST_SIZE - 2
#define AVERAGE_ALT_SIZE	ALT_HIST_SIZE - 2
#define ROLLING_AVERAGE_LEN	3
#define ACCEL_THRESHOLD		8.0f
#define ALT_DIFF_THRESHOLD	25

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
float* curAvgAccel = averageAccel;
float* avgAltPos = averageAlt;
float* curAvgAlt = averageAlt;
float* avgAccelEnd = averageAccel;
float* avgAltEnd = averageAlt;

uint64_t measCount = 1;
float maxAltitude = -1000.0f;
float prevAlt = 0.0f;

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
	static flightState_t state = ON_PAD;
	static uint8_t zeroCount = 0;

	switch (state) {
	case ON_PAD:
		if (*curAvgAccel > ACCEL_THRESHOLD) {
			state = UPWARDS;
		}
		else {
			state = ON_PAD;
		}
		break;
	case UPWARDS:
		if (maxAltitude - ALT_DIFF_THRESHOLD > *curAvgAccel) {
			state = DOWNWARDS;
		}
		else {
			state = UPWARDS;
		}
		break;
	case DOWNWARDS:
		if (*curAvgAlt - prevAlt < 1 || *curAvgAlt - prevAlt > -1) {
			zeroCount++;
		}
		if (zeroCount > 100) {
			state = LANDED;
		}
		else {
			state = DOWNWARDS;
		}
		break;
	case LANDED:
		state = LANDED;
		break;
	default:
		break;
	}

	return state;
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

		if (*avgAltPos > maxAltitude) {
			maxAltitude = *avgAltPos;
		}

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
		accelPos = accelHist;
	}
	if (altPos == altEnd) {
		altPos = altHist;
	}
}

void incPosAvg(void) {
	curAvgAccel = avgAccelPos;
	prevAlt = *curAvgAlt;
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
