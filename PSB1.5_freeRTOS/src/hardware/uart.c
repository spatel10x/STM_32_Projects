/* file: uart.c */

#include <stdint.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "PSBv1_5.h"
#include "error.h"
#include "hardware/uart.h"
extern UART_HandleTypeDef UartHandle;
BaseType_t xHardwareUartInit(void)
{
	BSP_UART_Init();
	return pdPASS;
}

BaseType_t xHardwareUartDeinit(void)
{
	HAL_UART_MspDeInit(&UartHandle);
    return pdPASS;
}

BaseType_t xHardwareUartTx(char* data, uint8_t data_length)
{
    if(HAL_UART_Transmit(&UartHandle, (uint8_t*)data, data_length, PSB_UART_TIMEOUT) != HAL_OK)
    {
        vErrorWarning("UART: transmit failed");
        return pdFAIL;
    }
    return pdPASS;
}

BaseType_t xHardwareUartRx(char* dest, uint8_t data_length)
{
    if(HAL_UART_Receive(&UartHandle, (uint8_t*)dest, data_length, PSB_UART_TIMEOUT) != HAL_OK)
    {
        vErrorWarning("UART: recieve failed");
        return pdFAIL;
    }
    return pdPASS;
}

