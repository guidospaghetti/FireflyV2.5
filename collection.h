#include "MTK3339.h"
#include "stdint.h"
typedef union collection_t {
	struct gps {
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
	uint8_t bytes[48];
} collection_t;

typedef struct collectionConfig_t {
	uint8_t lpm;	/**< Low power mode configuration, see MpuUtil.h*/
	uint8_t rate;	/**< Sampling rate in ms for each measurement, if lpm > 0 then GPS will be polled at 1 Hz*/
} collectionConfig_t;

void setup_collection(collectionConfig_t* _config);
void stop_collection(void);
void collect(collection_t* data);
