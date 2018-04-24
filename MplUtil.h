/*
 * MplUtil.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Aaron
 */

#ifndef MPLUTIL_H_
#define MPLUTIL_H_

typedef enum modeMPL_t {
	ALTITUDE_MODE,
	PRESSURE_MODE
} modeMPL_t;

typedef enum measurementMPL_t {
	PRESSURE,		/**< Pressure for pressure mode (kPa), Altitude for altitude mode (m) */
	TEMP_MPL,		/**< Temperature in Celsius */
	ALL_MPL				/**< Get both measurements */
} measurementMPL_t;

typedef struct allMPLData_t {
	float pressure;
	float temp;
} allMPLData_t;

void mplInit(modeMPL_t mode);
void readMeasurementMPL(measurementMPL_t mm, void* output);


#endif /* MPLUTIL_H_ */
