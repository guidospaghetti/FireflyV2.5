/*
 * MplUtil.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Aaron
 */

#ifndef MPLUTIL_H_
#define MPLUTIL_H_

typedef enum modeMPL_t {
	ALTITUDE_MODE,		/**< Set the MPL to output the altitude in meters*/
	PRESSURE_MODE		/**< Set the MPL to output the absolute pressuire in Pascals*/
} modeMPL_t;

typedef enum measurementMPL_t {
	PRESSURE,			/**< Pressure for pressure mode (kPa), Altitude for altitude mode (m) */
	TEMP_MPL,			/**< Temperature in Celsius */
	ALL_MPL				/**< Get both measurements */
} measurementMPL_t;

typedef struct allMPLData_t {
	float pressure;		/**< The altitude in meters or pressure in Pascals depending on initialization*/
	float temp;			/**< Temperature in Celsius*/
} allMPLData_t;

/**
 *  @fn mplInit(modeMPL_t mode)
 *  @param mode Which mode the MPL will be set to, either ALTITUDE_MODE or PRESSURE_MODE 
 *  @brief Initializes the MPL3115A2 to be ready to record data
 */
void mplInit(modeMPL_t mode);

/**
 *  @fn 
 *  @param mm Which measurement to output 
 *  @param output Pointer to where the output data will be stored
 *  @brief Outputs the desired measurement to the location given 
 *  
 *  The possible output data types are:
 *  PRESSURE - float
 *  TEMP_MPL - float
 *  ALL_MPL  - allMPLData_t
 */
void readMeasurementMPL(measurementMPL_t mm, void* output);


#endif /* MPLUTIL_H_ */
