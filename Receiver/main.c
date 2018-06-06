#include <msp430.h> 
#include "hal_spi_rf.h"
#include "radio_drv.h"
#include "cc1x_utils.h"
#include "LaunchPad_trx_demo.h"
#include "receiving.h"
#include "LED.h"

/*
 * main.c
 */

#define ENTER_LPM0()	__bis_SR_register(CPUOFF + GIE)

extern unsigned long volatile time_counter;
unsigned char wakeup_on_wdt = 0;

void msp_setup(void);
void setClock16MHz(void);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    msp_setup();

    run_receiving();

//	TA1CCR0 = 32768;
//	TA1CTL = TASSEL__ACLK + MC__UP + TACLR + ID__1 + TAIE;
//	TA1CCTL1 = CCIE;
//
//	while(1) {
//		ENTER_LPM0();
//		UCA1TXBUF = 'A';
//
//	}

	return 0;
}

void msp_setup(void) {

	INIT_LEDS();
	GREEN_ON();
	RED_ON();

	setClock16MHz();

	/////////////////////////////////////////////////////////////////////
	///
	/// Because pin 4.5 does not have interrupts, the port mapping is used
	/// to set it to Timer B CCR0 input and turn on interrupts for that port.
	/// Will be modified on Rev 2.5 to use a normal interrupt port.
	///
	/////////////////////////////////////////////////////////////////////

	// Set P4.5 to a port mapped input
	P4SEL |= BIT5;
	P4DIR &= ~BIT5;
	// Unlock port mapping
	PMAPKEYID = PMAPKEY;
	// Set pin 4.5 to TB0CCR0A input
	P4MAP5 = PM_TB0CCR0A;
	// Lock port mapping
	PMAPKEYID = 0;

	// Setup timer
	TB0CTL = TBIE + TBSSEL__ACLK + MC__UP;
	TB0CCTL0 = CM1 + CAP + CCIE;

	// Setup Watch dog timer for 1 second tick using 32Khz internal reference oscillator
	// ACLK is set to REFO
	WDTCTL = WDT_ADLY_1000;					// WDT 15.6ms, ACLK, interval timer
	SFRIE1 |= WDTIE;                        // Enable WDT interrupt
}

void setClock16MHz(void) {
	UCSCTL0 = 0x00;                           // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_6;                      // Select suitable range
	UCSCTL2 = FLLD_0 + 488;							  // DCO = 488 * 32768Hz ~= 16MHz

	UCSCTL3 = SELREF_2;
	UCSCTL4 = SELA__REFOCLK | SELS__DCOCLK | SELM__DCOCLK ;

	 __bic_SR_register(SCG0);

	// Worst-case settling time for the DCO when the DCO range bits have been
	// changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	// UG for optimization.
	// 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
	__delay_cycles(500000);
	// Loop until XT1,XT2 & DCO fault flag is cleared
	do {
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
		SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	} while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

	return;
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void TimerA1_ISR(void) {
	switch(__even_in_range(TA1IV, 4)) {
	case TA1IV_NONE:
		break;
	case TA1IV_TACCR1:
		_BIC_SR_IRQ(LPM3_bits);
		break;
	case TA1IV_TACCR2:
		break;
	default:
		break;
	}
}

#pragma vector=WDT_VECTOR
__interrupt void wdt_isr (void)
{
	/* global "1-second" counter used for printing time stamped packet sniffer data */
	time_counter++;

	/* check to see if wake on wdt is enabled */
	if(wakeup_on_wdt == 1)
	{

		/* exit from low power mode on ISR exit */
		_BIC_SR_IRQ(LPM3_bits);
	}
}
