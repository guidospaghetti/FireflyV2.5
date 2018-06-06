#include "retrieval.h"
#include "uart.h"
#include "storage.h"
#include "collection.h"

#define STOP_BYTE	0x03

void outputCollection(collection_t* data);
void retrievalCallback(uint8_t byte);

uint8_t stop = 0;

void run_retrieval(void) {

	addRxHandler(retrievalCallback, 1);
	collection_t data;
	while (readNext(&data)) {
		if (stop == 1) {
			break;
		}
		outputCollection(&data);
	}
}

void outputCollection(collection_t* data) {
	sendString(1, "%d,%d,%d,%f,%f,%c,%f,%c,%f,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
			data->data.gps.time.day,
			data->data.gps.time.month,
			data->data.gps.time.year,
			data->data.gps.time.utcTime,
			data->data.gps.location.latitude,
			data->data.gps.location.NS,
			data->data.gps.location.longitude,
			data->data.gps.location.EW,
			data->data.gps.location.altitude,
			data->data.gps.fix,
			data->data.gps.status,
			data->data.accel.x,
			data->data.accel.y,
			data->data.accel.z,
			data->data.gyro.x,
			data->data.gyro.y,
			data->data.gyro.z,
			data->data.altitude,
			data->data.temp);
}

void retrievalCallback(uint8_t byte) {
	if (byte == STOP_BYTE) {
		stop = 1;
	}
}
