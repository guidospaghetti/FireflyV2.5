#include <msp430f5529.h>
#include "collection.h"
#include "MTK3339.h"
#include "MpuUtil.h"
#include "MplUtil.h"
#include "stdint.h"

#define ACLK	32768
uint32_t timer_ticks = 0;
gpsData_t lastGPS;

void setup_collection(collectionConfig_t* _config) {
    gpsParams_t params;
    params.outputFrames = PMTK_RMC | PMTK_GGA;
	if (_config->lpm > 0) {
		params.updateRate = 1000;
	}
	else {
		params.updateRate = (_config->rate > 100) ? _config->rate : 100;
	}

    // Initialize GPS
    initGPS(&params);

    // Initialize MPU6050, first in low power mode
    mpuConfig_t mpuConfig;
    mpuConfig.LPM = _config->lpm;
    mpuConfig.accel = 3;
    mpuConfig.gyro = 2;
    mpuInit(&mpuConfig);

    // Initialize MPL3115A2
    mplInit(ALTITUDE_MODE);

    // Setup timer
    TA1CCR0 = ACLK/_config->rate;
    TA1CTL = TASSEL__ACLK + MC__UP + TACLR + ID__1 + TAIE;
}

void stop_collection(void) {
	TA1CTL = 0;
	TA1CCR0 = 0;
	TA1CCTL0 = 0;
	timer_ticks = 0;
}

void collect(collection_t* data) {

	allMPUData_t dataMPU;
	allMPLData_t dataMPL;
	gpsData_t gps;
	// Enter LPM
	__bis_SR_register(CPUOFF + GIE);
	readMeasurementMPU(ALL_MPU, (void*)&dataMPU);
	readMeasurementMPL(ALL_MPL, (void*)&dataMPL);
	uint8_t update = checkForUpdate(&gps);
	if (update) {
		lastGPS = gps;
	}
	data->accel.x = dataMPU.accelX;
	data->accel.y = dataMPU.accelY;
	data->accel.z = dataMPU.accelZ;
	data->gyro.x = dataMPU.gyroX;
	data->gyro.y = dataMPU.gyroY;
	data->gyro.z = dataMPU.gyroZ;
	data->altitude = dataMPL.pressure;
	data->temp = dataMPL.temp;
	data->gps.location = lastGPS.location;
	data->gps.fix = lastGPS.fix;
	data->gps.status = lastGPS.status;
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void TimerA1_ISR(void) {
	switch(__even_in_range(TA1IV, 4)) {
	case TA1IV_NONE:
		break;
	case TA1IV_TACCR1:
		timer_ticks++;
		_BIC_SR_IRQ(LPM3_bits);
		break;
	case TA1IV_TACCR2:
		break;
	default:
		break;
	}
}
