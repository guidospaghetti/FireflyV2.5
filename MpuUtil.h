/**
 * @file MpuRead.h
 * @author Aaron, Breanna
 * Code by Aaron, refactored by Breanna
 *
 * @brief Methods to utilize the MPU
 */

#include <stdint.h>
#include "I2C.h"
#include "MPU6050.h"

#define MPU6050_I2C_ADDRESS MPU6050_ADDRESS_AD0_HIGH
#define MPU6050_I2C_CHANNEL 0

typedef struct mpuConfig_t {
	uint8_t LPM; 	/**< Low Power Mode Configuration, 0 - Off, 1 - 1.25 Hz, 2 - 5 Hz, 3 - 20 Hz, 4 - 40 Hz*/
	uint8_t accel;	/**< Accelerometer Scale, 0 - +-2g, 1 - +-4g, 2 - +- 8g, 3 - +-16g*/
	uint8_t gyro; 	/**< Gyroscope Scale, 0 - +-250deg/s, 1 - +- 500deg/s, 2 - +-1kdeg/s, 3 - +-2kdeg/s*/
} mpuConfig_t;

/**
 * @enum measurement_t
 * @brief All the types of measurements from the MPU6050
 *
 */
typedef enum measurementMPU_t {
    ACCEL_X,        /**< Get the acceleration in the X direction*/
    ACCEL_Y,        /**< Get the acceleration in the Y direction*/
    ACCEL_Z,        /**< Get the acceleration in the Z direction*/
    GYRO_X,         /**< Get the gyroscope in the X direction*/
    GYRO_Y,         /**< Get the gyroscope in the Y direction*/
    GYRO_Z,         /**< Get the gyroscope in the Z direction*/
    TEMP_MPU,        /**< Get the current temperature*/
	ALL_MPU			/**< All the measurements*/
} measurementMPU_t;

typedef struct allMPUData_t {
	float accelX;	/**< Acceleration in X direction in g's*/
	float accelY;	/**< Acceleration in Y direction in g's*/
	float accelZ;	/**< Acceleration in Z direction in g's*/
	float gyroX;	/**< Change in gyroscopic position in X direction in deg/sec*/
	float gyroY;	/**< Change in gyroscopic position in Y direction in deg/sec*/
	float gyroZ;	/**< Change in gyroscopic position in Z direction in deg/sec*/
	float temp;		/**< Temperature in Celsius*/
} allMPUData_t;

/**
 * @param mm Type of measurement to read
 * @param data Pointer to where the output data will be stored
 * @brief This function stores the value of the measurement requested into the location
 * specified by the data parameter
 *
 * The possible output data types are:
 * ACCEL_X  - float
 * ACCEL_X  - float
 * ACCEL_X  - float
 * GYRO_X   - float
 * GYRO_X   - float
 * GYRO_X   - float
 * TEMP_MPU - float
 * ALL_MPU  - allMPUData_t
 */
void readMeasurementMPU(measurementMPU_t mm, void* data);

/**
 *  @param _config How the MPU6050 will be initialized
 *  @brief Initializes the MPU6050 according to the configuration given
 */
void mpuInit(mpuConfig_t* _config);
