#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "MTK3339.h"
#include "uart.h"


typedef enum parserState_t {
	START,
	COMMAND,
	DATA,
	CHECKSUM,
	END
} parserState_t;

typedef char fieldBuffer[PMTK_COMMAND_MAX_FIELD_LENGTH];

void parseResponse(char* response, gpsData_t* gps);
void sendGPSMessage(char* message);
uint8_t genChecksum(char* message);

void initGPS(gpsParams_t* params) {
	char message[50];
	sprintf(message, PMTK_SET_NMEA_UPDATERATE ",%d", params->updateRate);
	sendGPSMessage(message);

	sprintf(message, PMTK_API_SET_NMEA_OUTPUT ",0,%d,%d,%d,%d,%d,0,0,0,0,0,0,0,0",
			(params->outputFrames & PMTK_RMC) >> 0,
			(params->outputFrames & PMTK_VTG) >> 1,
			(params->outputFrames & PMTK_GGA) >> 2,
			(params->outputFrames & PMTK_GSA) >> 3,
			(params->outputFrames & PMTK_GSV) >> 4);
	sendGPSMessage(message);
}

void sendGPSMessage(char* message) {
	char checksumBytes[3];
	uint32_t length = strlen(message);
	char buffer[150] = {};
	uint8_t checksum = genChecksum(message);
	sprintf(checksumBytes, "%d", checksum);

	// Create the message that will be sent to the GPS
	buffer[0] = '$';
	strcat(buffer, message);
	strcat(buffer, "*");
	strcat(buffer, checksumBytes);
	strcat(buffer, "\r\n");

	sendUARTA0(buffer, strlen(buffer));
}

uint8_t genChecksum(char* message) {
	uint8_t checksum = 0;

	while (*message != '\0') {
		checksum ^= *message;
		message++;
	}

	return checksum;
}

uint8_t checkForUpdate(gpsData_t* gps) {
	// Check if a message is coming from the GPS
	if (lastByte0 == '$') {
		// Initialize a buffer
		char bufferStart[150] = {};
		char* buffer = bufferStart;
		*buffer = '$';
		//buffer++;
		while(hal_UART_DataAvailable(0) == 0);//wait
		volatile uint8_t counter = 1;
		// The GPS messages end with a new line character
		// so loop until that happens
		while (*buffer != '\n') {
			if (counter > 150) {
				break;
			}
			// Wait for the next byte
			while(hal_UART_DataAvailable(0) == 0);//wait, moved from line 65
			// Increment the buffer
			buffer++;
			// Add the new character to the buffer
			*buffer = lastByte0;

			counter++;
		}

		// Parse the respone from the GPS
		parseResponse(bufferStart, gps);
		return 1;
	}
	else {
		return 0;
	}
}

void parseResponse(char* response, gpsData_t* gps) {
	char buffer[100];
	char* bufferPtr = buffer;

	int dataPos = 0;
	int curDataLen = 0;
	uint8_t checksumDone = 0;
	parserState_t state = START;

	fieldBuffer allData[PMTK_COMMAND_MAX_NUM_FIELDS] = {};

	while (*response != '\r') {
		switch (state) {
		case START:
			if (*response == '$') {
				state = COMMAND;
			}
			break;
		case COMMAND:
			if (*response != ',') {
				*bufferPtr = *response;
				bufferPtr++;
			}
			else {
				strncpy(allData[dataPos], buffer, PMTK_COMMAND_NAME_LENGTH);
				bufferPtr = buffer;
				state = DATA;
				dataPos++;
			}
			break;
		case DATA:
			if (*response == '*') {
				strncpy(allData[dataPos], buffer, curDataLen);
				bufferPtr = buffer;
				curDataLen = 0;
				dataPos++;
				state = CHECKSUM;
			}
			else if (*response == ',') {
				strncpy(allData[dataPos], buffer, curDataLen);
				bufferPtr = buffer;
				curDataLen = 0;
				dataPos++;
			}
			else {
				*bufferPtr = *response;
				bufferPtr++;
				curDataLen++;
			}
			break;
		case CHECKSUM:
			*bufferPtr = *response;
			bufferPtr++;
			if (checksumDone) {
				strncpy(allData[dataPos], buffer, PMTK_CHECKSUM_LENGTH);
				bufferPtr = buffer;
			}
			checksumDone = 1;
			break;
		default:
			break;
		}
		response++;
	}

	if (strcmp(allData[0], PMTK_GGA_RETURN) == 0) {
		gps->time.utcTime = atof(allData[PMTK_GGA_UTC_TIME]);
		gps->location.latitude = atof(allData[PMTK_GGA_LATITUDE]);
		gps->location.NS = *allData[PMTK_GGA_NS_INDICATOR];
		gps->location.longitude = atof(allData[PMTK_GGA_LONGITUDE]);
		gps->location.EW = *allData[PMTK_GGA_EW_INDICATOR];
		gps->fix = atoi(allData[PMTK_GGA_FIX]);
		gps->numSatellites = atoi(allData[PMTK_GGA_NUM_SATELLITES]);
		gps->location.altitude = atof(allData[PMTK_GGA_ALTITUDE]);
	}
	else if (strcmp(allData[0], PMTK_GSA_RETURN) == 0) {
		// Don't use any of the information yet
	}
	else if (strcmp(allData[0], PMTK_GSV_RETURN) == 0) {
		uint8_t numMessages = atoi(allData[PMTK_GSV_NUM_MESSAGES]);
		uint8_t messageNum = atoi(allData[PMTK_GSV_MESSAGE_NUM]);
		volatile int i;
		for (i = 0; i < 4; i++) {
			gps->allSatellites[messageNum * 4 + i].satelliteID = atoi(allData[PMTK_GSV_SATELLITE_ID_1 + 4 * i]);
			gps->allSatellites[messageNum * 4 + i].elevation = atoi(allData[PMTK_GSV_ELEVATION_1 + 4 * i]);
			gps->allSatellites[messageNum * 4 + i].azimuth = atoi(allData[PMTK_GSV_AZIMUTH_1 + 4 * i]);
			gps->allSatellites[messageNum * 4 + i].SNR = atoi(allData[PMTK_GSV_SNR_1 + 4 * i]);
		}

	}
	else if (strcmp(allData[0], PMTK_RMC_RETURN) == 0) {
		gps->time.utcTime = atof(allData[PMTK_RMC_UTC_TIME]);
		gps->status = (*allData[PMTK_RMC_STATUS] == 'A' ? 1 : 0);
		gps->location.latitude = atof(allData[PMTK_RMC_LATITUDE]);
		gps->location.NS = *allData[PMTK_RMC_NS_INDICATOR];
		gps->location.longitude = atof(allData[PMTK_RMC_LONGITUDE]);
		gps->location.EW = *allData[PMTK_RMC_EW_INDICATOR];
		gps->speed = atof(allData[PMTK_RMC_SPEED_OVER_GND]);
		gps->heading = atof(allData[PMTK_RMC_COURSE_OVER_GND]);
		uint32_t date = strtoul(allData[PMTK_RMC_DATE], NULL, 0);
		gps->time.year = date % 100;
		date /= 100;
		gps->time.month = date % 100;
		date /= 100;
		gps->time.day = date % 100;

	}
	else if (strcmp(allData[0], PMTK_VTG_RETURN) == 0) {
		// Don't use any of the information yet
	}

}
