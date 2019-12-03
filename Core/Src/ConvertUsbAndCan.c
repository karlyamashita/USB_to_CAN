#include "main.h"
#include "ConvertUsbAndCan.h"
#include "CAN_Buffer.h"
#include "protocolCommands.h"

/*
 USB data format

 // CAN Type ExID = 4, StdID = 0
	tmp_buf[0] = EXT_ID;

	// Arb ID 29/11 bit
	tmp_buf[1] = 0x01; MSB
	tmp_buf[2] = 0x01;
	tmp_buf[3] = 0x01;
	tmp_buf[4] = 0x01; LSB

	tmp_buf[5] = 0x00; // RTR

	//DLC
	tmp_buf[6] = 8;

	// data bytes
	tmp_buf[7] = 0x01;
	tmp_buf[8] = 0x0A;
	tmp_buf[9] = 0x00;
	tmp_buf[10] = 0x0C;
	tmp_buf[11] = 0x00;
	tmp_buf[12] = 0x4D;
	tmp_buf[13] = 0x00;
	tmp_buf[14] = 0x11;


	// time stamp. When sending msg to PC
	tmp_buf[15] MSB
	tmp_buf[16]
	tmp_buf[17]
	tmp_buf[18] LSB

 */


/* Description: parse usb data and populate msg in correct CAN format
 * Input msg: pointer to array to save data in CAN format
 * Input data: The USB data
 * Output none
 */
void ConvertUsbDataToCanData(uint8_t *data) {

	CanTxMsgTypeDef msg;
	// data[0] is command from USB
	msg.CAN_TxHeaderTypeDef.IDE = data[1];
	if(data[1] == CAN_ID_STD) {
		msg.CAN_TxHeaderTypeDef.StdId = ( (data[4] <<8) | data[5]) & 0x000007FF;
	} else {
		msg.CAN_TxHeaderTypeDef.ExtId = ( (data[2] << 24) | (data[3] << 16) | (data[4] <<8) | data[5]) & 0x1FFFFFFF;// data[2] is MSB and data[5] is LSB
	}
	msg.CAN_TxHeaderTypeDef.RTR = data[6];// RTR
	msg.CAN_TxHeaderTypeDef.DLC = data[7];//

	for(int i = 0; i < 8; i++) { // copy 8 bytes even though DLC could be less
		msg.Data[i] = data[8 + i]; // index 8
	}

	AddCanTxBuffer1(&msg);
}

// just the opposite, copy CAN to USB data
void ConvertCanDataToUsbData(uint8_t *data, CanRxMsgTypeDef *msg) {
	uint8_t i = 0;
	data[0] = COMMAND_MESSAGE;
	data[1] = msg->CAN_RxHeaderTypeDef.IDE;

	if(data[1] == CAN_EXT_ID) {
		data[2] = msg->CAN_RxHeaderTypeDef.ExtId >> 24 & 0xFF;
		data[3] = msg->CAN_RxHeaderTypeDef.ExtId >> 16 & 0xFF;
		data[4] = msg->CAN_RxHeaderTypeDef.ExtId >> 8 & 0xFF;
		data[5] = msg->CAN_RxHeaderTypeDef.ExtId & 0xFF;
	} else {
		data[2] = msg->CAN_RxHeaderTypeDef.StdId >> 24 & 0xFF;
		data[3] = msg->CAN_RxHeaderTypeDef.StdId >> 16 & 0xFF;
		data[4] = msg->CAN_RxHeaderTypeDef.StdId >> 8 & 0xFF;
		data[5] = msg->CAN_RxHeaderTypeDef.StdId & 0xFF;
	}

	data[6] = msg->CAN_RxHeaderTypeDef.RTR;
	data[7] = msg->CAN_RxHeaderTypeDef.DLC;

	for(i = 0; i < 8; i++) {
		data[8+i] = msg->Data[i];
	}

	// time stamp
	//data[15] = msg->CAN_RxHeaderTypeDef.Timestamp >> 24 & 0xFF;
	//data[16] = msg->CAN_RxHeaderTypeDef.Timestamp >> 16 & 0xFF;
	//data[17] = msg->CAN_RxHeaderTypeDef.Timestamp >> 8 & 0xFF;
	//data[18] = msg->CAN_RxHeaderTypeDef.Timestamp & 0xFF;

}
