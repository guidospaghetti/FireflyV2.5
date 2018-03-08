#include <stdint.h>
#include <msp430.h>
#include "MpuUtil.h"


float getAccelX(void);
float getAccelY(void);
float getAccelZ(void);
float getGyroX(void);
float getGyroY(void);
float getGyroZ(void);
float getTempMPU(void);

uint8_t i2cReadReg(uint8_t regAddress, uint8_t slave)
{
    // Set the slave address
    slaveAddress = slave;
    // Send the register to read from
    TX_Data[0] = regAddress;
    // Only transmitting one byte
    TX_ByteCtr = 1;
    // Receiving 2-1 bytes
    RX_ByteCtr = 2;
    // Ensure a repeated start condition
    repeatedStart = 0x01;
    // Send the register to read
    i2cWrite(slaveAddress);

    // Read from the IC
    i2cRead(slaveAddress);
    // Remove repeated start condition
    repeatedStart = 0x00;
    // Return the data received
    return RX_Data[1];
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
    buffer[0] = i2cReadReg(MPU6050_RA_ACCEL_XOUT_H, slaveAddress);
    // Read the low bits
    buffer[1] = i2cReadReg(MPU6050_RA_ACCEL_XOUT_L, slaveAddress);
    // Concatenate and convert to true value
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getAccelY() {
    uint8_t buffer[2];
    // Read the high bits
    buffer[0] = i2cReadReg(MPU6050_RA_ACCEL_YOUT_H, slaveAddress);
    // Read the low bits
    buffer[1] = i2cReadReg(MPU6050_RA_ACCEL_YOUT_L, slaveAddress);
    // Concatenate and convert to true value, Same for the rest of the measurements
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getAccelZ() {
    uint8_t buffer[2];
    buffer[0] = i2cReadReg(MPU6050_RA_ACCEL_ZOUT_H, slaveAddress);
    buffer[1] = i2cReadReg(MPU6050_RA_ACCEL_ZOUT_L, slaveAddress);
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getGyroX() {
    uint8_t buffer[2];
    buffer[0] = i2cReadReg(MPU6050_RA_GYRO_XOUT_H, slaveAddress);
    buffer[1] = i2cReadReg(MPU6050_RA_GYRO_XOUT_L, slaveAddress);
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getGyroY() {
    uint8_t buffer[2];
    buffer[0] = i2cReadReg(MPU6050_RA_GYRO_YOUT_H, slaveAddress);
    buffer[1] = i2cReadReg(MPU6050_RA_GYRO_YOUT_L, slaveAddress);
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getGyroZ() {
    uint8_t buffer[2];
    buffer[0] = i2cReadReg(MPU6050_RA_GYRO_ZOUT_H, slaveAddress);
    buffer[1] = i2cReadReg(MPU6050_RA_GYRO_ZOUT_L, slaveAddress);
    return cvtAccel(((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]));
}

float getTempMPU() {
    uint8_t buffer[2];
    buffer[0] = i2cReadReg(MPU6050_RA_TEMP_OUT_H, slaveAddress);
    buffer[1] = i2cReadReg(MPU6050_RA_TEMP_OUT_L, slaveAddress);
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
    P3SEL |= BIT0 | BIT1;

    // To read from registers, the MPU6050 requires a repeated start condition
    repeatedStart = 0x01;
    // When pin AD0 on MPU6050 is low, slave address is 0x68
    // When it is high, the slave address is 0x69
    slaveAddress = MPU6050_ADDRESS_AD0_HIGH;

    // Initialize the I2C peripheral
    i2cInit();

    // For some reason, the chip doesn't respond if a register isn't read first
    uint8_t buffer[2];
    buffer[0] = i2cReadReg(MPU6050_RA_PWR_MGMT_1, slaveAddress);

    // Address of PWR_MGMT_1 register
    TX_Data[1] = MPU6050_RA_PWR_MGMT_1;
    // Set register to 0x80, resets all register values
    TX_Data[0] = 0x80;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);

    // Delay to ensure correct data
    for(i = 0; i < 1000; i++);

    // Address of PWR_MGMT_1 register
    TX_Data[1] = MPU6050_RA_PWR_MGMT_1;
    // Clear register reset pin (unsure if this is required)
    TX_Data[0] = 0x00;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);

    // Place the accelerometer in mode 1, +/- 4g
    TX_Data[1] = MPU6050_RA_ACCEL_CONFIG;
    TX_Data[0] = SCALE;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);

    // Place the gyroscope in mode 1, +/- 500 degree/seconds
    TX_Data[1] = MPU6050_RA_GYRO_CONFIG;
    TX_Data[0] = SCALE;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);

    // First data is always garbage
    allMPUData_t firstData;
    readMeasurementMPU(ALL_MPU, (void*)&firstData);
}

