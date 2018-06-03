#include <stdint.h>
#include <msp430.h>
#include "MpuUtil.h"

mpuConfig_t config;

/**
 * @param regAddress 8-bit address of the register to read from
 * @param slave 7-bit slave address of the chip
 * @brief Reads a specified register assuming a repeated start condition
 */
uint8_t readRegister(uint8_t regAddress, uint8_t slave);

void writeRegister(uint8_t regAddress, uint8_t data);

void burstReadRegister(uint8_t startRegAddress, uint8_t slave, uint8_t* data, uint8_t dataLen);

/**
 * @fn cvtTemp(int16_t temp)
 * @param temp Raw temperature data
 * @brief Returns the converted temperature
 */
inline float cvtTemp(int16_t temp);

/**
 * @fn cvtAccel(int16_t accel)
 * @param accel Raw acceleration data
 * @brief Returns the converted acceleration
 */
inline float cvtAccel(int16_t accel);

/**
 * @fn cvtGyro(int16_t gyro)
 * @param gyro Raw gyroscope data
 * *brief Returns the converted gyroscope
 */
inline float cvtGyro(int16_t gyro);

float getAccelX(void);
float getAccelY(void);
float getAccelZ(void);
float getGyroX(void);
float getGyroY(void);
float getGyroZ(void);
float getTempMPU(void);
void getAllData(allMPUData_t* data);

uint8_t readRegister(uint8_t regAddress, uint8_t slave)
{
	i2cTransaction_t trans;
	uint8_t txData[1] = {};
	uint8_t rxData[2] = {};
	txData[0] = regAddress;
	trans.data = txData;
	trans.dataLen = 1;
	trans.repeatedStart = 1;
	trans.slaveAddress = slave;
	trans.channel = MPU6050_I2C_CHANNEL;
	i2cWrite(&trans);

	trans.data = rxData;
	trans.dataLen = 5;
	trans.repeatedStart = 1;
	i2cRead(&trans);

	return rxData[0];
}

void writeRegister(uint8_t regAddress, uint8_t data) {
	i2cTransaction_t trans;
	uint8_t txData[2];
	txData[0] = regAddress;
	txData[1] = data;
	trans.data = txData;
	trans.dataLen = 2;
	trans.repeatedStart = 0;
	trans.slaveAddress = MPU6050_I2C_ADDRESS;
	trans.channel = MPU6050_I2C_CHANNEL;
	i2cWrite(&trans);
}

void burstReadRegister(uint8_t startRegAddress, uint8_t slave, uint8_t* data, uint8_t dataLen) {
	i2cTransaction_t trans;
	uint8_t txData[1] = {};
	txData[0] = startRegAddress;
	trans.data = txData;
	trans.dataLen = 1;
	trans.repeatedStart = 1;
	trans.slaveAddress = slave;
	trans.channel = MPU6050_I2C_CHANNEL;
	i2cWrite(&trans);

	trans.data = data;
	trans.dataLen = dataLen;
	trans.repeatedStart = 1;
	i2cRead(&trans);
}

void readMeasurementMPU(measurementMPU_t mm, void* data) {
    switch(mm) {
    case ACCEL_X:
    	*(float*)data = getAccelX();
    	break;
    case ACCEL_Y:
    	*(float*)data = getAccelY();
    	break;
    case ACCEL_Z:
    	*(float*)data = getAccelZ();
    	break;
    case GYRO_X:
    	*(float*)data = getGyroX();
    	break;
    case GYRO_Y:
    	*(float*)data = getGyroY();
    	break;
    case GYRO_Z:
    	*(float*)data = getGyroZ();
    	break;
    case TEMP_MPU:
    	*(float*)data = getTempMPU();
    	break;
    case ALL_MPU:
    {
    	allMPUData_t allData;
    	getAllData(&allData);

    	*(allMPUData_t*)data = allData;
    	return;
    }
    default:
        return;
    }
}

float getAccelX() {
    uint8_t buffer[2];
    // Read the high bits
    buffer[0] = readRegister(MPU6050_RA_ACCEL_XOUT_H, MPU6050_I2C_ADDRESS);
    // Read the low bits
    buffer[1] = readRegister(MPU6050_RA_ACCEL_XOUT_L, MPU6050_I2C_ADDRESS);
    // Concatenate and convert to true value
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getAccelY() {
    uint8_t buffer[2];
    // Read the high bits
    buffer[0] = readRegister(MPU6050_RA_ACCEL_YOUT_H, MPU6050_I2C_ADDRESS);
    // Read the low bits
    buffer[1] = readRegister(MPU6050_RA_ACCEL_YOUT_L, MPU6050_I2C_ADDRESS);
    // Concatenate and convert to true value, Same for the rest of the measurements
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getAccelZ() {
    uint8_t buffer[2];
    buffer[0] = readRegister(MPU6050_RA_ACCEL_ZOUT_H, MPU6050_I2C_ADDRESS);
    buffer[1] = readRegister(MPU6050_RA_ACCEL_ZOUT_L, MPU6050_I2C_ADDRESS);
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getGyroX() {
    uint8_t buffer[2];
    buffer[0] = readRegister(MPU6050_RA_GYRO_XOUT_H, MPU6050_I2C_ADDRESS);
    buffer[1] = readRegister(MPU6050_RA_GYRO_XOUT_L, MPU6050_I2C_ADDRESS);
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getGyroY() {
    uint8_t buffer[2];
    buffer[0] = readRegister(MPU6050_RA_GYRO_YOUT_H, MPU6050_I2C_ADDRESS);
    buffer[1] = readRegister(MPU6050_RA_GYRO_YOUT_L, MPU6050_I2C_ADDRESS);
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getGyroZ() {
    uint8_t buffer[2];
    buffer[0] = readRegister(MPU6050_RA_GYRO_ZOUT_H, MPU6050_I2C_ADDRESS);
    buffer[1] = readRegister(MPU6050_RA_GYRO_ZOUT_L, MPU6050_I2C_ADDRESS);
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getTempMPU() {
    uint8_t buffer[2];
    buffer[0] = readRegister(MPU6050_RA_TEMP_OUT_H, MPU6050_I2C_ADDRESS);
    buffer[1] = readRegister(MPU6050_RA_TEMP_OUT_L, MPU6050_I2C_ADDRESS);
    return cvtTemp(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

void getAllData(allMPUData_t* data) {
	uint8_t buffer[14];
	burstReadRegister(MPU6050_RA_ACCEL_XOUT_H, MPU6050_I2C_ADDRESS, buffer, 14);
	data->accelX = cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
	data->accelY = cvtAccel(((int16_t)buffer[2] << 8) | ((int16_t)buffer[3]));
	data->accelZ = cvtAccel(((int16_t)buffer[4] << 8) | ((int16_t)buffer[5]));
	data->temp = cvtTemp(((int16_t)buffer[6] << 8) | ((int16_t)buffer[7]));
	data->gyroX = cvtAccel(((int16_t)buffer[8] << 8) | ((int16_t)buffer[9]));
	data->gyroY = cvtAccel(((int16_t)buffer[10] << 8) | ((int16_t)buffer[11]));
	data->gyroZ = cvtAccel(((int16_t)buffer[12] << 8) | ((int16_t)buffer[13]));

}

inline float cvtTemp(int16_t temp) {
    // Formula from datasheet
    return ((temp / 340.0f) + 36.53);
}

inline float cvtAccel(int16_t accel) {
    // Raw data is the two's complement scaled according
    // to the range selected
    switch (config.accel) {
    case 0:
        return (~accel + 0x01) / (16384.0f);
    case 1:
        return (~accel + 0x01) / (8192.0f);
    case 2:
        return (~accel + 0x01) / (4096.0f);
    case 3:
        return (~accel + 0x01) / (2048.0f);
    default:
    	return (~accel + 0x01) / (2048.0f);
    }
}

inline float cvtGyro(int16_t gyro) {
    // Raw data is the two's complement scaled according
    // to the range selected
    switch (config.gyro) {
    case 0:
        return (~gyro + 0x01) / (131.0f);
    case 1:
        return (~gyro + 0x01) / (65.5f);
    case 2:
        return (~gyro + 0x01) / (32.8f);
    case 3:
        return (~gyro + 0x01) / (16.4f);
    default:
    	return (~gyro + 0x01) / (32.8f);
    }
}

void mpuInit(mpuConfig_t* _config) {

    volatile int i;
    config = *_config;

    // Set register to 0x80, resets all register values
    writeRegister(MPU6050_RA_PWR_MGMT_1, 0x80);

    // Delay to ensure correct data
    __delay_cycles(50000);

    // Reset the signal paths and sensor registers for all sensors
    writeRegister(MPU6050_RA_USER_CTRL, 0x01);

    if (config.LPM > 0) {
    	// Set CYCLE = 1, SLEEP = 0, TEMP_DIS = 1
    	// Required for LPM operation
    	writeRegister(MPU6050_RA_PWR_MGMT_1, 0x28);
    	uint8_t lpm = config.LPM - 1;
    	uint8_t pwm2 = lpm << 6;
    	pwm2 += 0x07;
    	// Set STBY_(X,Y,Z)G = 1 and the LP_WAKE_CTRL bits according to user input
    	writeRegister(MPU6050_RA_PWR_MGMT_2, pwm2);
    }
    else {
    	// Ensure the device is not in sleep or cycle mode
    	writeRegister(MPU6050_RA_PWR_MGMT_1, 0x00);
    	// Ensure that none of the sensors are in standby mode
    	writeRegister(MPU6050_RA_PWR_MGMT_2, 0x00);
    }

    // Place the accelerometer in mode defined by the scale
    writeRegister(MPU6050_RA_ACCEL_CONFIG, config.accel << 3);

    // Place the gyroscope in mode defined by scale
    writeRegister(MPU6050_RA_GYRO_CONFIG, config.gyro << 3);

    // First data is always garbage
    allMPUData_t firstData;
    readMeasurementMPU(ALL_MPU, (void*)&firstData);
}

