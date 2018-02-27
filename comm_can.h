#ifndef _COMM_CAN_H_
#define _COMM_CAN_H_

#include "ch.h"
#include "datatypes.h"

#ifndef ESCISVESC
	#include "can_data.h"
#endif

#define CAN_BROADCAST 0xFF
#define CAN_STATUS_MSGS_TO_STORE 10

void comm_can_init(void);
void comm_can_update(void);
void comm_can_transmit(uint8_t receiver, CANPacketID packetID, uint8_t *data, uint8_t len);
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send);
void comm_can_set_current(uint8_t controller_id, float current);
//float comm_can_get_infinity_current(void);

#endif /* _COMM_CAN_H_ */
