#include <stdint.h>
#include <msp430.h>
#include "MpuUtil.h"

/**
 * @fn i2cReadReg(uint8_t regAddress, uint8_t slave)
 * @param regAddress 8-bit address of the register to read from
 * @param slave 7-bit slave address of the chip
 * @brief Reads a specified register assuming a repeated start condition
 */
uint8_t readRegister(uint8_t regAddress, uint8_t slave);

void writeRegister(uint8_t regAddress, uint8_t data);

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

uint8_t readRegister(uint8_t regAddress, uint8_t slave)
{
	i2cTransaction_t trans;
	uint8_t txData[1];
	uint8_t rxData[2];
	txData[0] = regAddress;
	trans.data = txData;
	trans.dataLen = 1;
	trans.repeatedStart = 1;
	trans.slaveAddress = slave;
	i2cWrite(&trans);

	trans.data = rxData;
	trans.dataLen = 2;
	trans.repeatedStart = 0;
	i2cRead(&trans);

	return rxData[1];
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
	i2cWrite(&trans);
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
    	allData.accelX = getAccelX();
    	allData.accelY = getAccelY();
    	allData.accelZ = getAccelZ();
    	allData.gyroX = getGyroX();
    	allData.gyroY = getGyroY();
    	allData.gyroZ = getGyroZ();
    	allData.temp = getTempMPU();

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

inline float cvtTemp(int16_t temp) {
    // Formula from datasheet
    return ((temp / 340.0f) + 36.53);
}

inline float cvtAccel(int16_t accel) {
    // Raw data is the two's complement scaled according
    // to the range selected
    switch (SCALE) {
    case 0x00:
        return (~accel + 0x01) / (16384.0f / 2);
    case 0x08:
        return (~accel + 0x01) / (8192.0f / 2);
    case 0x10:
        return (~accel + 0x01) / (4096.0f / 2);
    case 0x18:
        return (~accel + 0x01) / (2048.0f / 2);
    default:
        return 0;
    }
}

inline float cvtGyro(int16_t gyro) {
    // Raw data is the two's complement scaled according
    // to the range selected
    switch (SCALE) {
    case 0x00:
        return (~gyro + 0x01) / (131.0f / 2);
    case 0x08:
        return (~gyro + 0x01) / (65.5f / 2);
    case 0x10:
        return (~gyro + 0x01) / (32.8f / 2);
    case 0x18:
        return (~gyro + 0x01) / (16.4f / 2);
    default:
        return 0;
    }
}

void mpuInit() {

    volatile int i;

    // Enable pins 3.0 and 3.1 for I2C operation
    //P3SEL |= BIT0 | BIT1;
    // Enable pins 4.1 and 4.2 for I2C operation
    P4SEL |= BIT1 | BIT2;

    // Initialize the I2C peripheral
    i2cInit();

    // For some reason, the chip doesn't respond if a register isn't read first
    uint8_t buffer[2];
    buffer[0] = readRegister(MPU6050_RA_PWR_MGMT_1, MPU6050_I2C_ADDRESS);

    // Set register to 0x80, resets all register values
    writeRegister(MPU6050_RA_PWR_MGMT_1, 0x80);

    // Delay to ensure correct data
    for(i = 0; i < 1000; i++);

    // Clear register reset pin (unsure if this is required)
    writeRegister(MPU6050_RA_PWR_MGMT_1, 0x00);

    // Place the accelerometer in mode defined by the scale
    writeRegister(MPU6050_RA_ACCEL_CONFIG, SCALE);

    // Place the gyroscope in mode defined by scale
    writeRegister(MPU6050_RA_GYRO_CONFIG, SCALE);

    // First data is always garbage
    allMPUData_t firstData;
    readMeasurementMPU(ALL_MPU, (void*)&firstData);
}

