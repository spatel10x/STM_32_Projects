/* heartbeat.c */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "error.h"
#include "hardware/can.h"
#include "hardware/hardware.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_can.h"
extern CAN_HandleTypeDef CanHandle;
BaseType_t xHardwareCANInit(void)
{
	BSP_CAN_Init();


	  if (HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0) != HAL_OK)
	  {
	    Error_Handler();
	  }


	return pdPASS;
}
//
//BaseType_t xHardwareCANDeinit(void)
//{
//	HAL_CAN_MspDeInit(&CANHandle);
//    return pdPASS;
//}
//
//BaseType_t xHardwareCANTx(char* data, uint8_t data_length)
//{
//    if(HAL_CAN_Transmit(&CANHandle,PSB_CAN_TIMEOUT) != HAL_OK)
//    {
//        vErrorWarning("CAN: transmit failed");
//        return pdFAIL;
//    }
//    return pdPASS;
//}
//
//BaseType_t xHardwareCANRx(char* dest, uint8_t data_length)
//{
//    if(HAL_CAN_Receive(&CANHandle,PSB_CAN_TIMEOUT) != HAL_OK)
//    {
//        vErrorWarning("CAN: recieve failed");
//        return pdFAIL;
//    }
//    return pdPASS;
//}

