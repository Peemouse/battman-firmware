#include "comm_can.h"
#include "ch.h"
#include "hal.h"
#include "hw_conf.h"
#include "datatypes.h"
#include <string.h>
#include "config.h"
#include "utils.h"
#include "power.h"

#define RX_FRAMES_SIZE  100

static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(8) | CAN_BTR_BRP(5)
};
static THD_WORKING_AREA(can_read_thread_wa, 512);
static THD_WORKING_AREA(can_process_thread_wa, 4096);
static THD_FUNCTION(can_read_thread, arg);
static THD_FUNCTION(can_process_thread, arg);

static CANRxFrame rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read;
static int rx_frame_write;
static thread_t *process_tp;
static mutex_t can_mtx;
static volatile Config *config;

void comm_can_init(void)
{
    config = config_get_configuration();
    chMtxObjectInit(&can_mtx);
    canStart(&CAND1, &cancfg);
    chThdCreateStatic(can_read_thread_wa, sizeof(can_read_thread_wa), NORMALPRIO + 1, can_read_thread, NULL);
    chThdCreateStatic(can_process_thread_wa, sizeof(can_process_thread_wa), NORMALPRIO, can_process_thread, NULL);
}

//static float infinity_current = 99;

static THD_FUNCTION(can_read_thread, arg) {
    (void)arg;
    chRegSetThreadName("CAN read");

    event_listener_t el;
    CANRxFrame rxmsg;

    chEvtRegister(&CAND1.rxfull_event, &el, 0);

    while(!chThdShouldTerminateX()) {
        if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(10)) == 0) {
            continue;
        }

        msg_t result = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);

        while (result == MSG_OK) {
            rx_frames[rx_frame_write++] = rxmsg;
            if (rx_frame_write == RX_FRAMES_SIZE) {
                rx_frame_write = 0;
            }

            chEvtSignal(process_tp, (eventmask_t) 1);

            result = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);
        }
    }

    chEvtUnregister(&CAND1.rxfull_event, &el);
}

static THD_FUNCTION(can_process_thread, arg) {
    (void)arg;

    chRegSetThreadName("CAN process");
    process_tp = chThdGetSelfX();

    int32_t ind = 0;

    for(;;) {
	chEvtWaitAny((eventmask_t) 1);

	static int32_t ind = 0;
	while (rx_frame_read != rx_frame_write) {
	    CANRxFrame rxmsg = rx_frames[rx_frame_read++];

	    if (rxmsg.IDE == CAN_IDE_EXT) {
		uint8_t sender = rxmsg.EID & 0xFF;
		uint8_t receiver = (rxmsg.EID >> 8) & 0xFF;
		CANPacketID id = rxmsg.EID >> 16;
		if ((receiver == CAN_BROADCAST || receiver == config->CANDeviceID) && sender != config->CANDeviceID) {
		    switch (id) { 
				case CAN_PACKET_STATUS:
						for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
							stat_tmp = &stat_msgs[i];
							if (stat_tmp->id == id || stat_tmp->id == -1) {
								ind = 0;
								stat_tmp->id = id;
								stat_tmp->rx_time = chVTGetSystemTime();
								stat_tmp->rpm = (float)buffer_get_int32(rxmsg.data8, &ind);
								stat_tmp->current = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
								stat_tmp->duty = (float)buffer_get_int16(rxmsg.data8, &ind) / 1000.0;
								break;
							}
						}
				break;
				
			// case CAN_PACKET_PROCESS_SHORT_BUFFER:
// 				ind = 0;
// 				rx_buffer_last_id = rxmsg.data8[ind++];
// 				commands_send = rxmsg.data8[ind++];
// 
// 				if (commands_send) {
// 					commands_send_packet(rxmsg.data8 + ind, rxmsg.DLC - ind);
// 				} else {
// 					commands_set_send_func(send_packet_wrapper);
// 					commands_process_packet(rxmsg.data8 + ind, rxmsg.DLC - ind);
// 				}
// 				break;			
			default:
			    break;
		    }
		}
	    }
	    if (rx_frame_read == RX_FRAMES_SIZE) {
		rx_frame_read = 0;
	    }
	}
    }
}

// float comm_can_get_infinity_current(void)
// {
    // return (float)infinity_current;
// }

void comm_can_update(void)
{
}

void comm_can_transmit(uint8_t receiver, CANPacketID packetID, uint8_t *data, uint8_t len)
{
	if (config->enVescCanComm) {
		CANTxFrame txmsg;
		txmsg.IDE = CAN_IDE_EXT;
		txmsg.EID = config->CANDeviceID | (receiver << 8) | (packetID << 16);
		txmsg.RTR = CAN_RTR_DATA;
		txmsg.DLC = len;
		memcpy(txmsg.data8, data, len);

		chMtxLock(&can_mtx);
		canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(20));
		chMtxUnlock(&can_mtx);
	}	
	else {
		return;
	}
}

void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = config->CANDeviceID;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		comm_can_transmit_eid(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = config->CANDeviceID;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		comm_can_transmit_eid(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
	}
	
void comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	comm_can_transmit(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
	}
}
