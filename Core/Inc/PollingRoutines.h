#ifndef POLLING_ROUTINE_H
#define POLLING_ROUTINE_H

#include "main.h"
#include "CAN_Buffer.h"


void PollingRoutine(void); // main entry

void HelloFromSTM32(void);
void ButtonPressed(void);
void HelloFromFreeRTOS(void);

int CheckButtonUser(void);

void ParseUsbRec(void);
void ParseCanRec(void);

void SendUsbMsgToCan(CanTxMsgTypeDef *msg);
void SendCanToUart(CanTxMsgTypeDef *msg);

void CanSnifferCanInit(CAN_HandleTypeDef *hcan, uint8_t *data);



#endif // POLLING_ROUTINE_H
