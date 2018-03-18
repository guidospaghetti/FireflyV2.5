/*
* Copyright (C) 2014 PHYTEC Messtechnik GmbH
*
* This file is subject to the terms and conditions of the GNU Lesser
* General Public License v2.1. See the file LICENSE in the top level
* directory for more details.
*/

#ifndef MPL3115A2_H
#define MPL3115A2_H

#include <stdint.h>
#include <stdbool.h>
//#include "periph/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MPL3115A2_I2C_ADDRESS 0x60
#define MPL3115A2_I2C_8BIT_READ_ADDRESS 0xC1
#define MPL3115A2_I2C_8BIT_WRITE_ADDRESS 0xC0
#define MPL3115A2_I2C_CHANNEL 1

#define MPL3115A2_OS_RATIO_1 0
#define MPL3115A2_OS_RATIO_2 1
#define MPL3115A2_OS_RATIO_4 2
#define MPL3115A2_OS_RATIO_8 3
#define MPL3115A2_OS_RATIO_16 4
#define MPL3115A2_OS_RATIO_32 5
#define MPL3115A2_OS_RATIO_64 6
#define MPL3115A2_OS_RATIO_128 7
#define MPL3115A2_OS_RATIO_DEFAULT MPL3115A2_OS_RATIO_128
#ifndef MPL3115A2_CONVERSION_TIME
#define MPL3115A2_CONVERSION_TIME 512000
#endif

//
//typedef struct {
//i2c_t i2c;
//uint8_t addr;
//bool initialized;
//} mpl3115a2_t;
//
//int mpl3115a2_test(mpl3115a2_t *dev);
//
//int mpl3115a2_init(mpl3115a2_t *dev, i2c_t i2c, uint8_t address, uint8_t os_ratio);
//
//int mpl3115a2_reset(mpl3115a2_t *dev);
//
//int mpl3115a2_set_active(mpl3115a2_t *dev);
//
//int mpl3115a2_set_standby(mpl3115a2_t *dev);
//
//int mpl3115a2_is_ready(mpl3115a2_t *dev);
//
//int mpl3115a2_read_pressure(mpl3115a2_t *dev, uint32_t *pres, uint8_t *status);
//
//int mpl3115a2_read_temp(mpl3115a2_t *dev, int16_t *temp);

#ifdef __cplusplus
}
#endif

#endif
