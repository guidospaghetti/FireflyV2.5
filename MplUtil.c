/*
 * MplUtil.c
 *
 *  Created on: Feb 6, 2018
 *      Author: Aaron
 */

#include "stdint.h"
#include "MplUtil.h"
#include "MPL3115A2_reg.h"
#include "MPL3115A2.h"
#include "I2C.h"

#define Q16_4_FRACTION	0.0625f
#define Q18_2_FRACTION	0.25f
#define Q8_4_FRACTION	Q16_4_FRACTION

void mplWriteReg(uint8_t address, uint8_t data);
uint8_t mplReadReg(uint8_t address);
void mplBurstReadReg(uint8_t address, uint8_t* data, uint8_t dataLen);
float getPressure(void);
float getAltitude(void);
float getTempMPL(void);
void altitudeMode(void);
void pressureMode(void);

modeMPL_t mode;

void mplInit(modeMPL_t mode_) {
	mode = mode_;

	uint8_t who = mplReadReg(MPL3115A2_WHO_AM_I);
	if (who == 0xC4) {
		if (mode == PRESSURE_MODE) {
			pressureMode();
		}
		else if (mode == ALTITUDE_MODE) {
			altitudeMode();
		}
	}
}

void altitudeMode(void) {
	// Write 1 0 111 0 0 0, Put in altitude mode, No RAW data, Oversampling rate of 128, reset disabled, do not use one shot, put in standby mode
	mplWriteReg(MPL3115A2_CTRL_REG1, 0xB8);
	// Write 00000 1 1 1, Reserved, Enable data ready flag, pressure/altitude data ready flag, and temperature data ready flag
	mplWriteReg(MPL3115A2_PT_DATA_CFG, 0x07);
	// Everything from before but the put in active mode instead of standby, determined by last bit
	mplWriteReg(MPL3115A2_CTRL_REG1, 0xB9);
}

void pressureMode(void) {
	// Write 0 0 111 0 0 0, Put in pressure mode, No RAW data, Oversampling rate of 128, reset disabled, do not use one shot, put in standby mode
	mplWriteReg(MPL3115A2_CTRL_REG1, 0x38);
	// Write 00000 1 1 1, Reserved, Enable data ready flag, pressure/altitude data ready flag, and temperature data ready flag
	mplWriteReg(MPL3115A2_PT_DATA_CFG, 0x07);
	// Everything from before but the put in active mode instead of standby, determined by last bit
	mplWriteReg(MPL3115A2_CTRL_REG1, 0x39);
}

void readMeasurementMPL(measurementMPL_t mm, void* output) {
	uint8_t status;
	volatile int counter = 0;

	status = mplReadReg(MPL3115A2_STATUS);
	if (status & 0x08 == 0) {
		return;
	}

	switch (mm) {
	case PRESSURE:
		if (mode == PRESSURE_MODE) {
			*(float*)output = getPressure();
		}
		else {
			*(float*)output = getAltitude();
		}
		return;
	case TEMP_MPL:
		*(float*)output = getTempMPL();
		return;
	case ALL_MPL:
	{
		float pressure;
		float temp;
		allMPLData_t all;
		uint8_t buffer[5];
		mplBurstReadReg(MPL3115A2_OUT_P_MSB, buffer, 5);

		if (mode == PRESSURE_MODE) {

			int32_t whole = ((int32_t)buffer[0] << 10) | ((int32_t)buffer[1] << 2) | (int32_t)buffer[2] >> 6;
			float fraction = ((buffer[2] & 0x30) >> 4) * Q18_2_FRACTION;
			pressure = (float)whole + fraction;
		}
		else {

			int16_t whole = ((int16_t)buffer[0] << 8) | (int16_t)buffer[1];
			float fraction = (buffer[2] >> 4) * Q16_4_FRACTION;
			pressure = (float)whole + fraction;
		}

		int8_t whole = (int8_t)buffer[3];
		float fraction = (buffer[4] >> 4) * Q8_4_FRACTION;
		temp = (float)whole + fraction;

		all.pressure = pressure;
		all.temp = temp;
		*(allMPLData_t*)output = all;
		return;
	}
	default:
		break;
	}

}

float getPressure(void) {
	uint8_t output[3];

	output[2] = mplReadReg(MPL3115A2_OUT_P_MSB);
	output[1] = mplReadReg(MPL3115A2_OUT_P_CSB);
	output[0] = mplReadReg(MPL3115A2_OUT_P_LSB);

	int32_t whole = ((int32_t)output[2] << 10) | ((int32_t)output[1] << 2) | (int32_t)output[0] >> 6;
	float fraction = ((output[0] & 0x30) >> 4) * Q18_2_FRACTION;

	return (float)whole + fraction;
}

float getAltitude(void) {

	uint8_t output[3];

	output[2] = mplReadReg(MPL3115A2_OUT_P_MSB);
	output[1] = mplReadReg(MPL3115A2_OUT_P_CSB);
	output[0] = mplReadReg(MPL3115A2_OUT_P_LSB);

	int16_t whole = ((int16_t)output[2] << 8) | (int16_t)output[1];
	float fraction = (output[0] >> 4) * Q16_4_FRACTION;

	return (float)whole + fraction;
}

float getTempMPL(void) {
	uint8_t output[2];
	output[1] = mplReadReg(MPL3115A2_OUT_T_MSB);
	output[0] = mplReadReg(MPL3115A2_OUT_T_LSB);

	int8_t whole = (int8_t)output[1];
	float fraction = (output[0] >> 4) * Q8_4_FRACTION;

	return (float)whole + fraction;
}

void setModeStandby(void)
{
	mplWriteReg(MPL3115A2_CTRL_REG1, 0x00);
}

void setModeAltimeter(void)
{
	mplWriteReg(MPL3115A2_CTRL_REG1, 0x80);
}

void enableEventFlags(void)
{
	mplWriteReg(MPL3115A2_PT_DATA_CFG, 0x07);
}

void setModeActive(void)
{
	mplWriteReg(MPL3115A2_CTRL_REG1, 0x01);
}

void setOversampleRate(int rate)
{
	if (rate > 7) {
		rate = 7;
	}

	rate <<= 3;

	uint8_t temp = mplReadReg(MPL3115A2_CTRL_REG1);
	temp &= 0xC7;
	temp |= rate;

	mplWriteReg(MPL3115A2_CTRL_REG1, temp);
}

void mplWriteReg(uint8_t address, uint8_t data) {
	i2cTransaction_t trans;
	uint8_t txData[2];
	txData[0] = address;
	txData[1] = data;
	trans.data = txData;
	trans.dataLen = 2;
	trans.repeatedStart = 0;
	trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
	trans.channel = MPL3115A2_I2C_CHANNEL;
	i2cWrite(&trans);
}

uint8_t mplReadReg(uint8_t address) {
	i2cTransaction_t trans;
	uint8_t txData[1];
	uint8_t rxData[2];
	txData[0] = address;
	trans.data = txData;
	trans.dataLen = 1;
	trans.repeatedStart = 1;
	trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
	trans.channel = MPL3115A2_I2C_CHANNEL;

	i2cWrite(&trans);

	trans.data = rxData;
	trans.dataLen = 2;
	trans.repeatedStart = 1;
	trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
	i2cRead(&trans);

	return rxData[0];
}

void mplBurstReadReg(uint8_t address, uint8_t* data, uint8_t dataLen) {
	i2cTransaction_t trans;
	uint8_t txData[1];
	txData[0] = address;
	trans.data = txData;
	trans.dataLen = 1;
	trans.repeatedStart = 1;
	trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
	trans.channel = MPL3115A2_I2C_CHANNEL;
	i2cWrite(&trans);

	trans.data = data;
	trans.dataLen = dataLen;
	trans.repeatedStart = 1;
	trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
	i2cRead(&trans);
}
