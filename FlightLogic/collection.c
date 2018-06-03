#include <msp430f5529.h>
#include <stdint.h>
#include "storage.h"
#include "collection.h"
#include "transmission.h"
#include "MTK3339.h"
#include "MpuUtil.h"
#include "MplUtil.h"
#include "uart.h"


#define ACLK	32768
#define SMCLK	16000000

uint32_t timer_ticks = 0;
gpsData_t lastGPS;
collectionConfig_t config;

void updateSensors(collection_t* data, uint8_t verbose);

void setup_collection(collectionConfig_t* _config) {
    static uint8_t setup = 0;
    uint8_t mpuStartup = 1;

    if (setup == 1) {
    	if (config.lpm == _config->lpm) {
    		mpuStartup = 0;
    	}
    }

	config = *_config;
	gpsParams_t params;
    params.outputFrames = PMTK_RMC | PMTK_GGA;
	if (config.lpm > 0) {
		params.updateRate = 1000;
	}
	else {
		params.updateRate = (_config->sampleRate > 0.100f) ? (uint16_t)(_config->sampleRate * 1000) : 100;
	}

    // Initialize GPS
    initGPS(&params);

    if (mpuStartup == 1) {
		// Initialize MPU6050, first in low power mode
		mpuConfig_t mpuConfig;
		mpuConfig.LPM = config.lpm;
		mpuConfig.accel = 3;
		mpuConfig.gyro = 2;
		mpuInit(&mpuConfig);
    }

    if (setup == 0) {
		// Initialize MPL3115A2
		mplInit(ALTITUDE_MODE);
    }
    // Setup timer
    uint16_t value = (uint16_t)((float)(ACLK >> 2) * _config->sampleRate);
    TA1CCR0 = value;
    TA1CTL = TASSEL__ACLK + MC__UP + TACLR + ID__4 + TAIE;

    setup = 1;
}

void stop_collection(void) {
	TA1CTL = 0;
	TA1CCR0 = 0;
	TA1CCTL1 = 0;
	timer_ticks = 0;
}

void collect(collection_t* data, uint8_t verbose) {

	TA1CCTL1 = CCIE;
	if (verbose == 1) {
		sendString(1, "Entering LPM\r\n");
	}
	// Enter LPM
	__bis_SR_register(CPUOFF + GIE);

	if (verbose == 1) {
		sendString(1, "Exited LPM\r\n");
	}

	updateSensors(data, verbose);

	if (config.storeRate > 0) {
		if (timer_ticks % config.storeRate == 0) {
			save(data);
		}
	}

	if (config.transmitRate > 0) {
		if (timer_ticks % config.transmitRate == 0) {
			transmit(data);
		}
	}

}

void updateSensors(collection_t* data, uint8_t verbose) {
	static uint8_t started = 0;
	allMPUData_t dataMPU;
	allMPLData_t dataMPL;
	gpsData_t gps;

	readMeasurementMPU(ALL_MPU, (void*)&dataMPU);
	if (started == 0 || verbose == 1) {
		sendString(1, "MPU Success\r\n");
	}
	readMeasurementMPL(ALL_MPL, (void*)&dataMPL);
	if (started == 0 || verbose == 1) {
		sendString(1, "MPL Success\r\n");
	}
	uint8_t update = checkForUpdate(&gps);
	if (update) {
		lastGPS = gps;
		if (started == 0 || verbose == 1) {
			sendString(1, "GPS Updated\r\n");
		}
	}
	else {
		if (started == 0 || verbose == 1) {
			sendString(1, "GPS Not Updated\r\n");
		}
	}

	data->data.accel.x = dataMPU.accelX;
	data->data.accel.y = dataMPU.accelY;
	data->data.accel.z = dataMPU.accelZ;
	if (config.lpm == 0) {
		data->data.gyro.x = dataMPU.gyroX;
		data->data.gyro.y = dataMPU.gyroY;
		data->data.gyro.z = dataMPU.gyroZ;
		data->data.temp = dataMPL.temp;
	}
	else {
		data->data.temp = dataMPU.temp;
	}
	data->data.altitude = dataMPL.pressure;
	data->data.gps.time = lastGPS.time;
	data->data.gps.location = lastGPS.location;
	data->data.gps.fix = lastGPS.fix;
	data->data.gps.status = lastGPS.status;

	started = 1;
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
