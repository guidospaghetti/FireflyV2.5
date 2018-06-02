#include "transmission.h"
#include "LED.h"
#include "hal_spi_rf.h"
#include "radio_drv.h"
#include "cc1x_utils.h"
#include "LaunchPad_trx_demo.h"

trx_cfg_struct trx_cfg;

void CC1120Init(void) {
	trx_cfg.bit_rate = radio_init(4);
	trx_cfg.bit_rate *= 100;

	trx_cfg.b_length = TX_BUFF_SIZE;
	rf_default_setup(&trx_cfg);

	set_rf_packet_length(trx_cfg.b_length);
	radio_set_freq(trx_cfg.start_freq+(trx_cfg.ch_spc*trx_cfg.rf_channel));

}

void transmit(collection_t* data) {
	GREEN_ON();
	radio_send(data->bytes, trx_cfg.b_length);
	radio_wait_for_idle(0);
	GREEN_OFF();
}

//void sendGPSDataTrx(gpsData_t* gps) {
//	packet_t packet;
//	packet.gps.fix = gps->fix;
//	packet.gps.speed = gps->speed;
//	packet.gps.status = gps->status;
//	packet.gps.location = gps->location;
//	HAL_LED2_ON();
//	radio_send(packet.bytes, trx_cfg.b_length);
//	radio_wait_for_idle(0);         // 0 = no timeout, just wait
//	HAL_LED2_OFF();
//}
