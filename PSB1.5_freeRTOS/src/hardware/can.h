/* CAN.h */

#ifndef _CAN_H
#define _CAN_H

#include "error.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"

/* Exported functions */
BaseType_t xHardwareCANInit(void);
BaseType_t xHardwareCANDeinit(void);
BaseType_t xHardwareCANTx(char* data, uint8_t data_length);
BaseType_t xHardwareCANRx(char* dest, uint8_t data_length);



#endif /* _CAN_H */

