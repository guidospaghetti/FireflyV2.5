#include <msp430.h> 
#include "MpuUtil.h"
#include "MplUtil.h"
#include "MTK3339.h"

void setClock16MHz(void);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    setClock16MHz();

    // Enable pins 3.0 and 3.1 for I2C operation
    //P3SEL |= BIT0 | BIT1;
    // Enable pins 4.1 and 4.2 for I2C operation
    P4SEL |= BIT1 | BIT2;

    // Initialize the I2C peripheral
    i2cInit(1);

    // Initialize MPU6050
    mpuInit();

    // Initialize MPL3115A2
    //mplInit(ALTITUDE_MODE);
    allMPUData_t dataMPU;
    //allMPLData_t dataMPL;
    //gpsData_t gps;

    while(1) {
    	readMeasurementMPU(ALL_MPU, (void*)&dataMPU);
    	//readMeasurementMPL(ALL_MPL, (void*)&dataMPL);
    	//checkForUpdate(&gps);
    }
	return (int)dataMPU.temp;
}

void setClock16MHz(void) {
	UCSCTL0 = 0x00;                           // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_6;                      // Select suitable range
	UCSCTL2 = FLLD_0 + 488;							  // DCO = 488 * 32768Hz ~= 16MHz

	/* MODIFICATION BEGIN*/
	//UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK ;
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
