#ifndef CONVERT_USB_AND_CAN_H
#define CONVERT_USB_AND_CAN_H

#include "CAN_Buffer.h"

void ConvertUsbDataToCanData(uint8_t *data);
void ConvertCanDataToUsbData(uint8_t *data, CanRxMsgTypeDef *msg);


#endif // CONVERT_USB_AND_CAN_H
