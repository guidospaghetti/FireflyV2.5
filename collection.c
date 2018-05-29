#include "collection.h"
#include "MTK3339.h"
#include "MpuUtil.h"
#include "MplUtil.h"
#include "stdint.h"

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
}
