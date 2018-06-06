/*
 * storage.h
 *
 *  Created on: Jun 2, 2018
 *      Author: Aaron
 */
#include <stdint.h>
#include "collection.h"
#ifndef STORAGE_H_
#define STORAGE_H_

void save(collection_t* data);
void saveEnd(void);
uint8_t readNext(collection_t* data);

#endif /* STORAGE_H_ */
