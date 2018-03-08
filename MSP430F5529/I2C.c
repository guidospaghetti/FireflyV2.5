#include <msp430.h>
#include "I2C.h"

unsigned char RX_Data[6];
unsigned char TX_Data[2];
unsigned char RX_ByteCtr;
unsigned char TX_ByteCtr;
unsigned char slaveAddress;
unsigned char repeatedStart;

void i2cInit(void)
{
	// set up I2C module
	UCB1CTL1 = UCSWRST;								// Enable SW reset
	UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;			// I2C Master, synchronous mode
	UCB1CTL1 |= UCSSEL_2;							// Use SMCLK, keep SW reset
	UCB1BR0 = 160;									// fSCL = SMCLK/(160 + 0*256) = 100kHz Assuming SMCLK = 16 MHz
	UCB1BR1 = 0;
	//UCB1IE = UCNACKIE;
	UCB1CTL1 &= ~UCSWRST;							// Clear SW reset, resume operation
}

void i2cWrite(unsigned char address)
{
	__disable_interrupt();
	UCB1I2CSA = address;							// Load slave address
	UCB1IE |= UCTXIE;								// Enable TX interrupt
	while(UCB1CTL1 & UCTXSTP);						// Ensure stop condition sent
	UCB1CTL1 |= UCTR + UCTXSTT;						// TX mode and START condition
	__bis_SR_register(CPUOFF + GIE);				// sleep until UCB0TXIFG is set ...
}

void i2cRead(unsigned char address)
{
	__disable_interrupt();
	UCB1I2CSA = address;							// Load slave address
	UCB1IE |= UCRXIE;								// Enable RX interrupt
	if (repeatedStart != 0x01)
		while(UCB1CTL1 & UCTXSTP);					// Ensure stop condition sent
	UCB1CTL1 &= ~UCTR;								// RX mode
	UCB1CTL1 |= UCTXSTT;							// Start Condition
    __bis_SR_register(CPUOFF + GIE);				// sleep until UCB0RXIFG is set ...
}

/**********************************************************************************************/
// USCIAB0TX_ISR
#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{

	switch(UCB1IV) {
	case USCI_I2C_UCRXIFG:
	case USCI_I2C_UCTXIFG:
		if (repeatedStart != 0x01)
		{
			if(UCB1CTL1 & UCTR)							// TX mode (UCTR == 1)
			{
				if (TX_ByteCtr)				  	  	    // TRUE if more bytes remain
				{
					TX_ByteCtr--;						// Decrement TX byte counter
					UCB1TXBUF = TX_Data[TX_ByteCtr];	// Load TX buffer
				}
				else									// no more bytes to send
				{
					UCB1CTL1 |= UCTXSTP;				// I2C stop condition
					UCB1IFG &= ~UCTXIFG;				// Clear USCI_B0 TX int flag
					__bic_SR_register_on_exit(CPUOFF);	// Exit LPM0
				}
			}
			else // (UCTR == 0)							// RX mode
			{
				RX_ByteCtr--;					        // Decrement RX byte counter
				if (RX_ByteCtr)					        // RxByteCtr != 0
				{
					RX_Data[RX_ByteCtr] = UCB1RXBUF;	// Get received byte
					if (RX_ByteCtr == 1)				// Only one byte left?
					UCB1CTL1 |= UCTXSTP;				// Generate I2C stop condition
				}
				else									// RxByteCtr == 0
				{
					RX_Data[RX_ByteCtr] = UCB1RXBUF;	// Get final received byte
					__bic_SR_register_on_exit(CPUOFF);	// Exit LPM0
				}
			}

		}
		else if (repeatedStart == 0x01)
		{
			if(UCB1CTL1 & UCTR)							// TX mode (UCTR == 1)
			{
				if (TX_ByteCtr)				    	    // TRUE if more bytes remain
				{
					TX_ByteCtr--;						// Decrement TX byte counter
					UCB1TXBUF = TX_Data[TX_ByteCtr];	// Load TX buffer
				}
				else									// no more bytes to send
				{
					//UCB0CTL1 |= UCTXSTP;				// I2C stop condition
					UCB1IFG &= ~UCTXIFG;				// Clear USCI_B0 TX int flag
					__bic_SR_register_on_exit(CPUOFF);	// Exit LPM0
				}
			}
			else // (UCTR == 0)							// RX mode
			{
				RX_ByteCtr--;				  		    // Decrement RX byte counter
				if (RX_ByteCtr)				  		    // RxByteCtr != 0
				{
					RX_Data[RX_ByteCtr] = UCB1RXBUF;	// Get received byte
					if (RX_ByteCtr == 1)				// Only one byte left?
					UCB1CTL1 |= UCTXSTP;				// Generate I2C stop condition
				}
				else									// RxByteCtr == 0
				{
					RX_Data[RX_ByteCtr] = UCB1RXBUF;	// Get final received byte
					__bic_SR_register_on_exit(CPUOFF);	// Exit LPM0
				}
			}
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




