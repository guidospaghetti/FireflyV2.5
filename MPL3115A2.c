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


    TX_Data[1] = MPL3115A2_CTRL_REG1;
    TX_Data[0] = 0x00;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);
}

void setModeAltimeter(void)
{
    TX_Data[1] = MPL3115A2_CTRL_REG1;
    TX_Data[0] = 0x80;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);
}

void enableEventFlags(void)
{
	TX_Data[1] = MPL3115A2_PT_DATA_CFG;
	TX_Data[0] = 0x07;
	TX_ByteCtr = 2;
	i2cWrite(slaveAddress);
}

void setModeActive(void)
{
    TX_Data[1] = MPL3115A2_CTRL_REG1;
    TX_Data[0] = 0x01;
    TX_ByteCtr = 2;
    i2cWrite(slaveAddress);
}

void setOversampleRate(int rate)
{
	if (rate > 7)
		rate = 7;

	rate <<= 3;
	slaveAddress = 0xC1 >> 1;
	TX_Data[0] = MPL3115A2_CTRL_REG1;
	TX_ByteCtr = 1;
	i2cWrite(slaveAddress);

    unsigned char temp;
    RX_ByteCtr = 2;
	i2cRead(slaveAddress);
	temp = RX_Data[0];
	temp &= 0xC7;
	temp |= rate;

	slaveAddress = 0xC0 >> 1;
	TX_Data[1] = MPL3115A2_CTRL_REG1;
	TX_Data[0] = temp;
	TX_ByteCtr = 2;
	i2cWrite(slaveAddress);
}

