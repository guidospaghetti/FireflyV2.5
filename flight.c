#include <msp430.h>
#include "MpuUtil.h"
#include "MplUtil.h"
#include "MTK3339.h"
#include "hal_uart.h"

void sensor_setup(void);
void wait_for_launch(void);
void upwards(void);
void downwards(void);
void landed(void);


void run_flight(void) {
	sensor_setup();
	wait_for_launch();
	upwards();
	downwards();
	landed();
}
