#include <msp430.h> 
#include "MpuUtil.h"
#include "MplUtil.h"
#include "MTK3339.h"
#include "hal_uart.h"
#include "hal_spi_rf.h"
#include "radio_drv.h"
#include "cc1x_utils.h"
#include "LaunchPad_trx_demo.h"
#include "uart.h"
#include "string.h"
#include "stdio.h"

#define LED_PORT_DIR	P5DIR
#define LED_PORT_OUT	P5OUT
#define LED1			BIT0
#define LED2			BIT1

trx_cfg_struct trx_cfg;
extern unsigned long volatile time_counter;
unsigned char wakeup_on_wdt;

typedef union packet_t {
	struct gps {
		location_t location;
		float speed;
		uint8_t fix;
		uint8_t status;
	} gps;
	uint8_t bytes[20];
} packet_t;

void msp_setup(void);
void cc1120Init(void);
void sendMPUMPL(allMPUData_t* mpu, allMPLData_t* mpl);
void setClock16MHz(void);
void sendGPSDataUART(gpsData_t* gps);
void sendGPSDataTrx(gpsData_t* gps);

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    msp_setup();

    gpsParams_t params;
    params.outputFrames = PMTK_RMC | PMTK_GGA;
    params.updateRate = 1000;

    // Initialize GPS
    initGPS(&params);
    allMPUData_t dataMPU;
    allMPLData_t dataMPL;
    gpsData_t gps;

    // Initialize MPU6050
    mpuInit();

    // Initialize MPL3115A2
    mplInit(ALTITUDE_MODE);

    cc1120Init();

    while(1) {
    	readMeasurementMPU(ALL_MPU, (void*)&dataMPU);
    	readMeasurementMPL(ALL_MPL, (void*)&dataMPL);
    	uint8_t update = checkForUpdate(&gps);
    	//sendMPUMPL(&dataMPU, &dataMPL);
    	if (update) {
    		sendGPSDataUART(&gps);
    		sendGPSDataTrx(&gps);
    	}
    	__no_operation();
    }

	return (int)dataMPL.temp + (int)dataMPU.temp;
}

void msp_setup(void) {

    LED_PORT_DIR |= LED1 + LED2;
    LED_PORT_OUT &= ~LED1;
    LED_PORT_OUT |= LED2;

	setClock16MHz();

	/////////////////////////////////////////////////////////////////////
	///
	/// Because pin 4.5 does not have interrupts, the port mapping is used
	/// to set it to Timer B CCR0 input and turn on interrupts for that port.
	/// Will be modified on Rev 2.5 to use a normal interrupt port.
	///
	/////////////////////////////////////////////////////////////////////

	// Set P4.5 to a port mapped input
//	P4SEL |= BIT5;
//	P4DIR &= ~BIT5;
//	// Unlock port mapping
//	PMAPKEYID = PMAPKEY;
//	// Set pin 4.5 to TB0CCR0A input
//	P4MAP5 = PM_TB0CCR0A;
//	// Lock port mapping
//	PMAPKEYID = 0;

	// Setup timer
//	TB0CTL = TBIE + TBSSEL__ACLK + MC__UP;
//	TB0CCTL0 = CM1 + CAP + CCIE;

	// Setup Watch dog timer for 1 second tick using 32Khz internal reference oscillator
	// ACLK is set to REFO
	WDTCTL = WDT_ADLY_1000;					// WDT 15.6ms, ACLK, interval timer
	SFRIE1 |= WDTIE;                        // Enable WDT interrupt

    // Initialize the I2C peripheral
    i2cInit(0);

    // Initialize the UART peripheral
    hal_UART_Init();
}

void cc1120Init(void) {
	trx_cfg.bit_rate = radio_init(4);
	trx_cfg.bit_rate *= 100;

	trx_cfg.b_length = TX_BUFF_SIZE;
	rf_default_setup(&trx_cfg);

	set_rf_packet_length(trx_cfg.b_length);
	radio_set_freq(trx_cfg.start_freq+(trx_cfg.ch_spc*trx_cfg.rf_channel));

}

void sendMPUMPL(allMPUData_t* mpu, allMPLData_t* mpl) {
	char buffer[100];
	sprintf(buffer, "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n",
					mpl->pressure, mpl->temp, mpu->temp,
					mpu->accelX, mpu->accelY, mpu->accelZ,
					mpu->gyroX, mpu->gyroY, mpu->gyroZ);

	sendUARTA1(buffer, strlen(buffer));
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

void sendGPSDataUART(gpsData_t* gps) {
	char buffer[150];
	sprintf(buffer,
			"Latitude: %f %c Longitude: %f %c Altitude: %f\r\n"
			"Time: %f MM/DD/YY: %d/%d/%d Fix: %d\r\n",
			gps->location.latitude,
			(gps->location.NS ? gps->location.NS : 'X'),
			gps->location.longitude,
			(gps->location.EW ? gps->location.EW : 'X'),
			gps->location.altitude,
			gps->time.utcTime,
			gps->time.month,
			gps->time.day,
			gps->time.year,
			gps->fix);

	sendUARTA1(buffer, strlen(buffer));
}

void sendGPSDataTrx(gpsData_t* gps) {
	packet_t packet;
	packet.gps.fix = gps->fix;
	packet.gps.speed = gps->speed;
	packet.gps.status = gps->status;
	packet.gps.location = gps->location;
	HAL_LED2_ON();
	radio_send(packet.bytes, trx_cfg.b_length);
	radio_wait_for_idle(0);         // 0 = no timeout, just wait
	HAL_LED2_OFF();
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
