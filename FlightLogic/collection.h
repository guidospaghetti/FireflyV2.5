#include "MTK3339.h"
#include "stdint.h"

#ifndef COLLECTION_H
#define COLLECTION_H

#define COLLECTION_SIZE	55+1

typedef union collection_t {
	struct data {
		struct gps {
			time_t time;
			location_t location;
			uint8_t fix;
			uint8_t status;
		} gps;
		struct accel {
			float x;
			float y;
			float z;
		} accel;
		struct gyro {
			float x;
			float y;
			float z;
		} gyro;
		float altitude;
		float temp;
	} data;
	uint8_t bytes[COLLECTION_SIZE];
} collection_t;

typedef struct collectionConfig_t {
	uint8_t lpm;			/**< Low power mode configuration, see MpuUtil.h*/
	float sampleRate;		/**< Sampling rate in seconds for each measurement, if lpm > 0 then GPS will be polled at 1 Hz*/
	uint16_t storeRate;		/**< How often data will be stored, increments of sample rate. i.e. 1 - save every sample, 2 - save every other, etc. 0 - don't store data*/
	uint16_t transmitRate;	/**< How often data will be transmitted, increments of sample rate*/
	uint8_t doneStoring;	/**< No more data will need to be stored after this setup. Saves bytes to flash signaling the end of the data.*/
} collectionConfig_t;

void setup_collection(collectionConfig_t* _config);
void stop_collection(void);
void collect(collection_t* data, uint8_t verbose);

#endif
