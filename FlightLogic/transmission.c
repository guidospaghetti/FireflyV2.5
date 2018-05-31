#include "transmission.h"

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
