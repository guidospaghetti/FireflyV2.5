/*
 * MPL3115A2.c
 *
 *  Created on: Dec 13, 2016
 *      Author: Aaron
 */

#include <msp430f5529.h>
#include "I2C.h"
#include "MPL3115A2.h"
#include "MPL3115A2_reg.h"


void setModeStandby(void)
{

	i2cTransaction_t trans;
	uint8_t txData[2];
	txData[1] = MPL3115A2_CTRL_REG1;
	txData[0] = 0x00;
    trans.data = txData;
    trans.dataLen = 2;
    trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
    i2cWrite(&trans);
}

void setModeAltimeter(void)
{
	i2cTransaction_t trans;
	uint8_t txData[2];
	txData[1] = MPL3115A2_CTRL_REG1;
	txData[0] = 0x80;
    trans.data = txData;
    trans.dataLen = 2;
    trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
    i2cWrite(&trans);
}

void enableEventFlags(void)
{
	i2cTransaction_t trans;
	uint8_t txData[2];
	txData[1] = MPL3115A2_PT_DATA_CFG;
	txData[0] = 0x07;
    trans.data = txData;
    trans.dataLen = 2;
    trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
	i2cWrite(&trans);
}

void setModeActive(void)
{
	i2cTransaction_t trans;
	uint8_t txData[2];
	txData[1] = MPL3115A2_CTRL_REG1;
	txData[0] = 0x01;
    trans.data = txData;
    trans.dataLen = 2;
    trans.slaveAddress = MPL3115A2_I2C_ADDRESS;
    i2cWrite(&trans);
}

void setOversampleRate(int rate)
{
	if (rate > 7)
		rate = 7;

	rate <<= 3;
	uint8_t slaveAddress = 0xC1 >> 1;
	i2cTransaction_t trans;
	uint8_t txData[1];
	txData[0] = MPL3115A2_CTRL_REG1;
	trans.data = txData;
	trans.dataLen = 1;
	trans.repeatedStart = 1;
	trans.slaveAddress = slaveAddress;
	i2cWrite(&trans);

    unsigned char temp;
    uint8_t rxData[2];
    trans.data = rxData;
    trans.dataLen = 2;
    trans.repeatedStart = 0;
	i2cRead(&trans);
	temp = rxData[0];
	temp &= 0xC7;
	temp |= rate;

	slaveAddress = 0xC0 >> 1;
	txData[1] = MPL3115A2_CTRL_REG1;
	txData[0] = temp;
	trans.dataLen = 2;
	i2cWrite(slaveAddress);
}

