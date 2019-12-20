#include "PollingRoutines.h"
#include "main.h"
#include "usbd_conf.h" // path to USBD_CUSTOMHID_OUTREPORT_BUF_SIZE define

#include "CAN_Buffer.h"
#include "protocolCommands.h"
#include "USB_Buffer.h"
#include "ConvertUsbAndCan.h"
#include "GPIO_Ports.h"

#include "IntArrayToString.h"


// these variables are defined in main.c
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
extern uint8_t usbRxBuffer[USB_MAX_RING_BUFF][USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
extern RING_BUFF_INFO usbRxRingBuffPtr;

uint8_t USB_TX_Buffer[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE]; // To send usb data to PC
uint8_t canBusActive = 0;

const char* Version = "v1.0.1";
const char* Hardware = "STM32F407-Discovery";


char uartHelloMsg[] = {"Hello From STM32\n\r"};
char uartButtonPressedMsg[] = {"User Button is PRESSED!\n\r"};


/*
 *  Description: The main entry point. Typically in all my projects I call this routine from main all do all my polling here, if i am not using Tasks.
 *
 */
void PollingRoutine(void){
	// polling is currently done in Tasks
}

// just a useless thing to do something with the user button
void ButtonPressed(void) {
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)uartButtonPressedMsg, sizeof(uartButtonPressedMsg) -1 );
}

void HelloFromSTM32(void) {
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)uartHelloMsg, sizeof(uartHelloMsg) -1 );
}

// check for user button press
int CheckButtonUser(void) {
	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET ){ // Read button
		return 1;
	} else return 0;
}

/*
 * function: Parse the USB data in the buffer.
 * input: none
 * output: none
 *
 */
void ParseUsbRec(void) {
	uint8_t usbData[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
	if(UsbDataAvailable(usbData)) {
		uint8_t node = GetNode(usbData);
		switch(usbData[0])
		{
		case COMMAND_MESSAGE:
			switch(node) {
			case CAN1_NODE:
				SendUsbDataToCanBus(CAN1_NODE, usbData);
				break;
			case CAN2_NODE:
				break;
			}
			break;
		case COMMAND_BAUD:
			switch(usbData[5]) { // index 5 is node
			case CAN1_NODE:
				CanSnifferCanInit(&hcan1, usbData);
				break;
			case CAN2_NODE:
				break;
			}
			break;
		case COMMAND_INFO:
			SendHardwareInfo();
			SendVersionInfo();
			switch(node) {
			case CAN1_NODE:
				Send_CAN_BTR(&hcan1);
				break;
			case CAN2_NODE:
				break;
			}
			break;
		}
	}
}

void SendHardwareInfo(void) {
	uint8_t data[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};
	uint8_t i = 0;
	data[0] = COMMAND_HARDWARE;
	while( Hardware[i] != '\0') {
		data[i + 1] = (uint8_t) Hardware[i];
		i++;
	}
	AddUsbTxBuffer(data);
}

void SendVersionInfo(void) {
	uint8_t data[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};
	uint8_t i = 0;
	data[0] = COMMAND_VERSION;
	while( Version[i] != '\0') {
		data[i + 1] = (uint8_t) Version[i];
		i++;
	}
	AddUsbTxBuffer(data);
}

void Send_CAN_BTR(CAN_HandleTypeDef *hcan) {
	uint8_t data[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};
	uint32_t btrValue = READ_REG(hcan->Instance->BTR);

	data[0] = COMMAND_CAN_BTR;
	data[1] = btrValue >> 24 & 0xFF;
	data[2] = btrValue >> 16 & 0xFF;
	data[3] = btrValue >> 8 & 0xFF;
	data[4] = btrValue & 0xFF;
	AddUsbTxBuffer(data);
}

/*
 * function: Parse the CAN data in the buffer.
 * input: none
 * output: none
 *
 */
void ParseCanRec(void) {
	uint8_t canMsgAvailableFlag = 0;
	CanRxMsgTypeDef canRxMsg;
	uint8_t usbData[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];

	memset(&usbData, 0, USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

	canMsgAvailableFlag = Can1DataAvailable(&canRxMsg); // check ring buffer for new message
	if(canMsgAvailableFlag) {
		if(canRxMsg.CAN_RxHeaderTypeDef.IDE == CAN_EXT_ID) { // EXT ID
			SendCanDataToUsb(&canRxMsg);
		} else { // STD ID
			SendCanDataToUsb(&canRxMsg);
		}
	}
}

/*
 * function: Send CAN message to uart. Note: Not currently working
 * input: the CAN message
 * output: none
 *
 */
void SendCanToUart(CanTxMsgTypeDef *msg)
{
	char buf[100] = "";
	int written = -1;

	uint8_t uMsg[14];

	uMsg[0] = msg->CAN_TxHeaderTypeDef.IDE;
	// todo check for std or ext id
	uMsg[1] = msg->CAN_TxHeaderTypeDef.ExtId >> 24 & 0xFF;
	uMsg[2] = msg->CAN_TxHeaderTypeDef.ExtId >> 16 & 0xFF;
	uMsg[3] = msg->CAN_TxHeaderTypeDef.ExtId >> 8 & 0xFF;
	uMsg[4] = msg->CAN_TxHeaderTypeDef.ExtId & 0xFF;
	uMsg[5] = msg->CAN_TxHeaderTypeDef.DLC;

	for(int i = 0; i < 8; i++) {
		uMsg[6 + i] = msg->Data[i];
	}

	// todo - add a uart ring buffer
	written = UintArrayToString(uMsg, sizeof(uMsg), buf, sizeof(buf) );

	if(written == -1) return;

	//HAL_UART_Transmit_IT(&huart2, &buf, sizeof(written) );
}

/*
 * function: This is copied from the CAN_Buffer.c file. You can use this to toggle LED to indicate CAN bus activity
 * input: On or Off state of LED
 * output: none
 */
void CanBusActivityStatus(uint8_t status) {
	canBusActive = status;
}

uint8_t GetCanBusActive(void) {
	return canBusActive;
}

/*
 * Description: Changes the CAN handle baud rate received from the PC. Use the calculator from "bittiming.can-wiki.info" to get the CAN_BTR value
 * Input: the CAN Handle and the CAN_BTR value
 * Output: none
 */
void CanSnifferCanInit(CAN_HandleTypeDef *hcan, uint8_t *data) {

	uint32_t btrValue = 0;
	uint8_t usbData[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE] = {0};

	btrValue = data[1] << 24 | data[2] << 16 | data[3] << 8 | data[4]; // parse the BTR data

	// some of these snippets were copied from HAL_CAN_Init()
	HAL_CAN_DeInit(hcan);

	if (hcan->State == HAL_CAN_STATE_RESET)
	{
		/* Init the low level hardware: CLOCK, NVIC */
		HAL_CAN_MspInit(hcan);
	}

	/* Set the bit timing register */
	WRITE_REG(hcan->Instance->BTR, (uint32_t)(btrValue));

	/* Initialize the error code */
	hcan->ErrorCode = HAL_CAN_ERROR_NONE;

	/* Initialize the CAN state */
	hcan->State = HAL_CAN_STATE_READY;

	if(HAL_CAN_Start(&hcan1) != HAL_OK) { // start the CAN module
		usbData[0] = COMMAND_NAK; // NAK PC
		AddUsbTxBuffer(usbData);
		return;
	}

	usbData[0] = COMMAND_ACK; // ACK PC back
	AddUsbTxBuffer(usbData);
}

// todo - send PC error message. Maybe send as special command instead of in CAN message.
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->ErrorCode != HAL_CAN_ERROR_NONE) {

	}
}

