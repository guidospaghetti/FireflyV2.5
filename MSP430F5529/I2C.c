#include <msp430.h>
#include "I2C.h"

i2cTransaction_t* curTrans;

void i2cInit(void)
{
	// Enable SW reset
	UCB1CTL1 = UCSWRST;
	// I2C Master, synchronous mode
	UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;
	// Use SMCLK, keep SW reset
	UCB1CTL1 |= UCSSEL__SMCLK;
	// fSCL = SMCLK/(160 + 0*256) = 100kHz Assuming SMCLK = 16 MHz
	UCB1BR0 = 160;
	UCB1BR1 = 0;
	// Clear SW reset, resume operation
	UCB1CTL1 &= ~UCSWRST;
}

void i2cWrite(i2cTransaction_t* trans)
{
	__disable_interrupt();
	// Set the current transaction
	curTrans = trans;
	// Store slave address
	UCB1I2CSA = trans->slaveAddress;
	// Enable TX interrupt
	UCB1IE |= UCTXIE;
	// Ensure stop condition sent
	while(UCB1CTL1 & UCTXSTP);
	// TX mode and START condition
	UCB1CTL1 |= UCTR + UCTXSTT;
	// sleep until UCB1TXIFG is set
	__bis_SR_register(CPUOFF + GIE);
}

void i2cRead(i2cTransaction_t* trans)
{
	__disable_interrupt();
	// Set the current transaction
	curTrans = trans;
	// Load slave address
	UCB1I2CSA = trans->slaveAddress;
	// Enable RX interrupt
	UCB1IE |= UCRXIE;
	// If there is no repeated start condition
	if (trans->repeatedStart != 1) {
		// Ensure stop condition sent
		while(UCB1CTL1 & UCTXSTP);
	}
	// RX mode
	UCB1CTL1 &= ~UCTR;
	// Start Condition
	UCB1CTL1 |= UCTXSTT;
	// sleep until UCB1RXIFG is set ...
    __bis_SR_register(CPUOFF + GIE);
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
	switch(UCB0IV) {
	case USCI_I2C_UCRXIFG:
		if (--(curTrans->dataLen)) {
			// Get received byte
			*(curTrans->data) = UCB0RXBUF;
			curTrans->data++;
			if (curTrans->dataLen == 1) {
				// Generate I2C stop condition
				UCB0CTL1 |= UCTXSTP;
			}
		}
		else {
			*(curTrans->data) = UCB0RXBUF;
			// Exit LPM0
			__bic_SR_register_on_exit(CPUOFF);
		}
		break;
	case USCI_I2C_UCTXIFG:
		// TRUE if more bytes remain
		if (curTrans->dataLen--) {
			UCB0TXBUF = *(curTrans->data);
			curTrans->data++;
		}
		else {
			if (curTrans->repeatedStart == 0) {
				// I2C stop condition
				UCB0CTL1 |= UCTXSTP;
			}
			// Clear TX interrupt flag
			UCB0IFG &= ~UCTXIFG;
			__bic_SR_register_on_exit(CPUOFF);
		}
		break;
	case USCI_I2C_UCNACKIFG:
		//UCB0IFG &= ~UCNACKIFG;
		//UCB0CTL1 |= UCSWRST;
		//__bic_SR_register_on_exit(CPUOFF);
		break;
	default:
		break;
	}
}


#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{
	switch(UCB1IV) {
	case USCI_I2C_UCRXIFG:
		if (--(curTrans->dataLen)) {
			// Get received byte
			*(curTrans->data) = UCB1RXBUF;
			curTrans->data++;
			if (curTrans->dataLen == 1) {
				// Generate I2C stop condition
				UCB1CTL1 |= UCTXSTP;
			}
		}
		else {
			*(curTrans->data) = UCB1RXBUF;
			// Exit LPM0
			__bic_SR_register_on_exit(CPUOFF);
		}
		break;
	case USCI_I2C_UCTXIFG:
		// TRUE if more bytes remain
		if (curTrans->dataLen--) {
			UCB1TXBUF = *(curTrans->data);
			curTrans->data++;
		}
		else {
			if (curTrans->repeatedStart == 0) {
				// I2C stop condition
				UCB1CTL1 |= UCTXSTP;
			}
			// Clear TX interrupt flag
			UCB1IFG &= ~UCTXIFG;
			__bic_SR_register_on_exit(CPUOFF);
		}
		break;
	case USCI_I2C_UCNACKIFG:
		//UCB1IFG &= ~UCNACKIFG;
		//UCB1CTL1 |= UCSWRST;
		//__bic_SR_register_on_exit(CPUOFF);
		break;
	default:
		break;
	}
}




