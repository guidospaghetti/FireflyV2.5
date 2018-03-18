
#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

typedef struct i2cTransaction_t {
	uint8_t* data;
	uint8_t dataLen;
	uint8_t slaveAddress;
	uint8_t repeatedStart;
	uint8_t channel;
} i2cTransaction_t;

void i2cInit(uint8_t channel);
void i2cWrite(i2cTransaction_t* trans);
void i2cRead(i2cTransaction_t* trans);


#endif /* I2C_H_ */




