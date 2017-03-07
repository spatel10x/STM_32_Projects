/* file: uart.h */

#ifndef _HARDWARE_UART_H
#define _HARDWARE_UART_H

#include "error.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

/* Exported functions */
BaseType_t xHardwareUartInit(void);
BaseType_t xHardwareUartDeinit(void);
BaseType_t xHardwareUartTx(char* data, uint8_t data_length);
BaseType_t xHardwareUartRx(char* dest, uint8_t data_length);

#endif /* _HARDWARE_UART_H */

