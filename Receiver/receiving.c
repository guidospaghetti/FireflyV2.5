/*
 * receiving.c
 *
 *  Created on: Jun 6, 2018
 *      Author: Aaron
 */

#include "receiving.h"
#include "hal_spi_rf.h"
#include "radio_drv.h"
#include "cc1x_utils.h"
#include "LaunchPad_trx_demo.h"
#include "receiver_uart.h"
#include "collection.h"
#include "LED.h"
#include <stdint.h>

void setToUART(void);
void setToCS(void);

trx_cfg_struct trx_cfg;
uint8_t txBuffer[TX_BUFF_SIZE];
unsigned char rand_data[60] =  {49,231,121,199,255,153,138,232,220,203,
		51,253,117,172,161,191,79,58,225,215,149,251,15,163,153,236,141,172,3,
		186,224,37,224,210,75,69,117,58,153,207,61,203,19,125,76,90,143,226,11,
		208,16,72,109,217,100,12,128,228,185,142};

void CC1120Init(void) {
	trx_cfg.bit_rate = radio_init(4);
	trx_cfg.bit_rate = trx_cfg.bit_rate * 100;

	rf_default_setup(&trx_cfg);
	trx_cfg.b_length = TX_BUFF_SIZE;
	trx_cfg.start_freq = 433500;
}

void run_receiving(void) {

	CC1120Init();
	collection_t data;
	int16_t cc_status;

	set_rf_packet_length(trx_cfg.b_length);
	radio_set_freq(trx_cfg.start_freq+(trx_cfg.ch_spc*trx_cfg.rf_channel));

	while (1) {
		GREEN_ON();
		radio_send(rand_data, trx_cfg.b_length);
		radio_wait_for_idle(0);
		GREEN_OFF();
	}

	while (1) {
		setToCS();
		GREEN_OFF();
		radio_receive_on();
		radio_wait_for_idle(0);
		GREEN_ON();
		unsigned short b_length = trx_cfg.b_length;
		cc_status = radio_read(txBuffer, &b_length);
		int16_t rssi = radio_get_rssi();


		if (cc_status > 0) {
			setToUART();
			volatile uint8_t i;
			for (i = 0; i < TX_BUFF_SIZE; i++) {
				data.bytes[i] = txBuffer[i];
			}
			sendCollection(&data);
		}
	}
}

void setToUART(void) {
    // Enable pins for UART
    // Only Pin 4.4 (Tx to computer) is attached
    P4SEL |= BIT4 + BIT5;

	// Set software reset pin Computer
	UCA1CTL1 = UCSWRST;
	// Set clock to SMCLK
	UCA1CTL1 |= UCSSEL1;
	// Set baud rate, 115200 baud, Assuming 16MHz / (138 + 0*256) = ~115200
	UCA1BR0 = 138;
	UCA1BR1 = 0;
	UCA1MCTL = UCBRS_7 + UCBRF_0;
	// Turn on UART
	UCA1CTL1 &= ~UCSWRST;
}

void setToCS(void) {
	// select chip select bit RF_CS_N_PORT_SEL as a port
	RF_CS_N_PORT_SEL &= ~RF_CS_N_PIN;
	RF_CS_N_PORT_DIR |= RF_CS_N_PIN;
	RF_CS_N_PORT_OUT |= RF_CS_N_PIN;
}
