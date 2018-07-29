/*
 * settings.h
 *
 *  Created on: Jun 4, 2018
 *      Author: Aaron
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#define SYS_CLOCK						16000000

//#define NO_SAVING
//#define NO_TRANSMITTING

#define WAIT_FOR_LAUNCH_SAMPLE_RATE		0.025f
#define WAIT_FOR_LAUNCH_STORAGE_RATE	400		// 400 * 0.025 = 10s
#define WAIT_FOR_LAUNCH_TRANSMIT_RATE	200		// 200 * 0.025 = 5s

#define UPWARDS_SAMPLE_RATE				0.025f
#define UPWARDS_STORAGE_RATE			1
#define UPWARDS_TRANSMIT_RATE			40		// 40 * 0.025 = 1s

#define DOWNWARDS_SAMPLE_RATE			2.0f
#define DOWNWARDS_STORAGE_RATE			2

#define DOWNWARDS_TRANSMIT_RATE			2

#define LANDED_SAMPLE_RATE				5.0f
#define LANDED_STORAGE_RATE				0
#define LANDED_TRANSMIT_RATE			1

#endif /* SETTINGS_H_ */
