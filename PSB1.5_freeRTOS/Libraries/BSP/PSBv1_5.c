
/* Includes ------------------------------------------------------------------*/
#include "PSBv1_5.h"

#define __PSB_BSP_VERSION            (0x01)

/** @defgroup Private_Variables Private Variables
  * @{
  */ 
/**
 * @brief LED variables
 */
GPIO_TypeDef* LED_PORT[LEDn] = {LED1_GPIO_PORT, 
                                LED2_GPIO_PORT};

const uint16_t LED_PIN[LEDn] = {LED1_PIN, 
                                LED2_PIN };
/**
 * @brief BUS variables
 */

#ifdef HAL_SPI_MODULE_ENABLED
uint32_t SpixTimeout = PSB_SPIx_TIMEOUT_MAX;        /*<! Value of Timeout when SPI communication fails */
static SPI_HandleTypeDef hPSB_Spi;
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
uint32_t I2cxTimeout = PSB_I2Cx_TIMEOUT_MAX;   /*<! Value of Timeout when I2C communication fails */
I2C_HandleTypeDef hPSB_I2c;
I2C_HandleTypeDef hi2c;
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
UART_HandleTypeDef UartHandle;

DMA_HandleTypeDef hdma_tx;
DMA_HandleTypeDef hdma_rx;

#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
CAN_HandleTypeDef CanHandle;
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef TimHandle;
#endif /* HAL_TIM_MODULE_ENABLED */


/**
  * @}
  */ 

/* I2Cx bus function */
#ifdef HAL_I2C_MODULE_ENABLED
/* Link function for I2C EEPROM peripheral */
void               BSP_I2C_Init(void);
static void               I2Cx_ITConfig(void);
static HAL_StatusTypeDef  I2Cx_ReadMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef  I2Cx_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static void               I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value);
static HAL_StatusTypeDef  I2Cx_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length);
static uint8_t            I2Cx_ReadData(uint16_t Addr, uint8_t Reg);
static HAL_StatusTypeDef  I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials);
static void               I2Cx_Error(uint8_t Addr);
static void               I2Cx_MspInit(I2C_HandleTypeDef *hi2c);

/* Link function for IO Expander over I2C */
void                      IOE_Init(void);
void                      IOE_ITConfig(void);
void                      IOE_Delay(uint32_t Delay);
void                      IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t                   IOE_Read(uint8_t Addr, uint8_t Reg);
uint16_t                  IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);

#endif /* HAL_I2C_MODULE_ENABLED */

/* SPIx bus function */
#ifdef HAL_SPI_MODULE_ENABLED
static void               SPIx_Init(void);
static void               SPIx_Write(uint8_t Value);
static uint32_t           SPIx_Read(void);
static void               SPIx_Error (void);
static void               SPIx_MspInit(SPI_HandleTypeDef *hspi);

#endif /* HAL_SPI_MODULE_ENABLED */

/* SPIx bus function */



void BSP_System_Init(void)
{
	SystemClock_Config();

	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);

}



uint32_t BSP_GetVersion(void)
{
  return __PSB_BSP_VERSION;
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  gpioinitstruct.Pin    = LED_PIN[Led];
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull   = GPIO_NOPULL;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(LED_PORT[Led], &gpioinitstruct);

  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}


#ifdef HAL_UART_MODULE_ENABLED


void BSP_UART_Init(void)
{

  UartHandle.Instance = PSB_UART;
  UartHandle.Init.BaudRate = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

}



void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO clock */
    PSB_UART_TX_GPIO_CLK_ENABLE();
    PSB_UART_RX_GPIO_CLK_ENABLE();

    /* Enable PSB_UART clock */
    PSB_UART_CLK_ENABLE();

    /* Enable DMA clock */
    DMAx_CLK_ENABLE();

    /*## Configure peripheral GPIO ##########################################*/
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = PSB_UART_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(PSB_UART_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin 	  = PSB_UART_RX_PIN;
    GPIO_InitStruct.Mode	  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull	  = GPIO_NOPULL;

    HAL_GPIO_Init(PSB_UART_RX_GPIO_PORT, &GPIO_InitStruct);

    /*## Configure the DMA ##################################################*/
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = PSB_UART_TX_DMA_CHANNEL;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_DISABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdma_tx);
    /* Configure the DMA handler for reception process */

    hdma_rx.Instance                 = PSB_UART_RX_DMA_CHANNEL;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_DISABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;

    HAL_DMA_Init(&hdma_rx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdma_rx);

    /* NVIC configuration for USART, to catch the TX complete */
    HAL_NVIC_SetPriority(PSB_UART_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(PSB_UART_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

	/*##-1- Reset peripherals ##################################################*/
	PSB_UART_FORCE_RESET();
	PSB_UART_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks ################################*/
	/* Configure UART Tx as alternate function  */
	HAL_GPIO_DeInit(PSB_UART_TX_GPIO_PORT, PSB_UART_TX_PIN);
	/* Configure UART Rx as alternate function  */
	HAL_GPIO_DeInit(PSB_UART_RX_GPIO_PORT, PSB_UART_RX_PIN);

	/*##-3- Disable the DMA Channels ###########################################*/
	/* De-Initialize the DMA Channel associated to transmission process */
	HAL_DMA_DeInit(&hdma_tx);
	/* De-Initialize the DMA Channel associated to reception process */
	HAL_DMA_DeInit(&hdma_rx);

	/*##-4- Disable the NVIC for DMA ###########################################*/
	HAL_NVIC_DisableIRQ(PSB_UART_DMA_TX_IRQn);
	HAL_NVIC_DisableIRQ(PSB_UART_DMA_RX_IRQn);
}


#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
/**
  * @brief  Configures CAN port.
  * @param  CAN: Specifies the CAN port to be configured.
  *   This parameter can be one of following parameters:
  *     @arg PSB_CAN1
  * @param  CAN: pointer to a CAN_HandleTypeDef structure that
  *   contains the configuration information for the specified CAN peripheral.
  * @retval None
  */


void BSP_CAN_Init(void)
{

  CAN_FilterConfTypeDef  sFilterConfig;
  static CanTxMsgTypeDef        TxMessage;
  static CanRxMsgTypeDef        RxMessage;

  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = ((CAN_TypeDef*)CAN1_BASE);
  CanHandle.pTxMsg = &TxMessage;
  CanHandle.pRxMsg = &RxMessage;

  CanHandle.Init.TTCM = DISABLE;
  CanHandle.Init.ABOM = ENABLE;
  CanHandle.Init.AWUM = DISABLE;
  CanHandle.Init.NART = DISABLE;
  CanHandle.Init.RFLM = DISABLE;
  CanHandle.Init.TXFP = DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SJW = CAN_SJW_1TQ;
  CanHandle.Init.BS1 = CAN_BS1_8TQ;
  CanHandle.Init.BS2 = CAN_BS2_3TQ;
  CanHandle.Init.Prescaler = 3;

  if (HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initiliazation Error */
    Error_Handler();
  }

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Configure Transmission process #####################################*/
  CanHandle.pTxMsg->StdId = 0x321;
  CanHandle.pTxMsg->ExtId = 0x01;
  CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
  CanHandle.pTxMsg->IDE = CAN_ID_STD;
  CanHandle.pTxMsg->DLC = 2;

}


void HAL_CAN_MspInit(CAN_HandleTypeDef* CanHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* CAN1 Periph clock enable */
  __HAL_RCC_CAN1_CLK_ENABLE();

  /* Enable GPIO clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Enable AFIO clock and remap CAN PINs to PB_8 and PB_9*/
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_CAN1_2();

  /* CAN1 RX GPIO pin configuration */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /* Peripheral interrupt init */
//    HAL_NVIC_SetPriority(PSB_CAN_TX_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(PSB_CAN_TX_IRQn);

    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    HAL_NVIC_SetPriority(PSB_CAN_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(PSB_CAN_RX1_IRQn);

/*    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);*/
  /* USER CODE BEGIN CAN1_MspInit 1 */
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN1)
  {

    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);

    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);

    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);

  }
}

int CANx_Write(CAN_Message *TXmsg)
{
	 int i = 0;

	    if(TXmsg->format== CANStandard) {
	        CanHandle.pTxMsg->StdId = TXmsg->id;
	        CanHandle.pTxMsg->ExtId = 0x00;
	    }
	    else {
	        CanHandle.pTxMsg->StdId = 0x00;
	        CanHandle.pTxMsg->ExtId = TXmsg->id;
	    }

	    CanHandle.pTxMsg->RTR = TXmsg->type == CANData ? CAN_RTR_DATA : CAN_RTR_REMOTE;
	    CanHandle.pTxMsg->IDE = TXmsg->format == CANStandard ? CAN_ID_STD : CAN_ID_EXT;
	    CanHandle.pTxMsg->DLC = TXmsg->len;

	    for(i = 0; i < TXmsg->len; i++)
	        CanHandle.pTxMsg->Data[i] = TXmsg->data[i];

	    if(HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK) {
	#ifdef DEBUG
	        printf("Transmission error\r\n");
	#endif
	        return 0;
	    }
	    else
	        return 1;
}

int CANx_Read(CAN_Message *RXmsg)
{
    int i;
    RXmsg->id = CanHandle.pRxMsg->IDE == CAN_ID_STD ? CanHandle.pRxMsg->StdId : CanHandle.pRxMsg->ExtId;
    RXmsg->type = CanHandle.pRxMsg->RTR == CAN_RTR_DATA ? CANData : CANRemote;
    RXmsg->format = CanHandle.pRxMsg->IDE == CAN_ID_STD ? CANStandard : CANExtended;
    RXmsg->len = CanHandle.pRxMsg->DLC;
    for(i = 0; i < RXmsg->len; i++)
        RXmsg->data[i] = CanHandle.pRxMsg->Data[i];

    return RXmsg->len;
}



#endif /* HAL_CAN_MODULE_ENABLED */


#ifdef HAL_ADC_MODULE_ENABLED
/**
  * @brief  Configures CAN port.
  * @param  CAN: Specifies the CAN port to be configured.
  *   This parameter can be one of following parameters:
  *     @arg PSB_CAN1
  * @param  CAN: pointer to a CAN_HandleTypeDef structure that
  *   contains the configuration information for the specified CAN peripheral.
  * @retval None
  */
void BSP_ADC1_Init(void)
{


	  ADC_ChannelConfTypeDef sConfig;

	    /**Common config
	    */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	  hadc1.Init.ContinuousConvMode = ENABLE;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.NbrOfConversion = 9;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }


	    /**Configure Regular Channel
	    */
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	//sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 2;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	    /**Configure Regular Channel
	    */
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 3;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	    /**Configure Regular Channel
	    */
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 4;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	    /**Configure Regular Channel
	    */
	  sConfig.Channel = ADC_CHANNEL_10;
	  sConfig.Rank = 5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	    /**Configure Regular Channel
	    */
	  sConfig.Channel = ADC_CHANNEL_11;
	  sConfig.Rank = 6;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	    /**Configure Regular Channel
	    */
	  sConfig.Channel = ADC_CHANNEL_12;
	  sConfig.Rank = 7;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	    /**Configure Regular Channel
	    */
	  sConfig.Channel = ADC_CHANNEL_13;
	  sConfig.Rank = 8;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&rawADC[1],9);
	//	print("  \r \n[OK] ADC conversions started");
	//	print("  \r \n");
}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA4     ------> ADC1_IN4

    PB0     ------> ADC1_IN8
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /* Peripheral DMA init*/

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  }

}



void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA0		------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA4     ------> ADC1_IN4
    PB0     ------> ADC1_IN8 */

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
}


#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */ 

/*================ DMA Configuration ================================  */

void PSB_DMA_Init(void)
{
	  /* DMA controller clock enable */
	  __HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
//	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	/* NVIC configuration for DMA transfer complete interrupt (PSB_UART_TX) */
	HAL_NVIC_SetPriority(PSB_UART_DMA_TX_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(PSB_UART_DMA_TX_IRQn);

	/* NVIC configuration for DMA transfer complete interrupt (PSB_UART_RX) */
	HAL_NVIC_SetPriority(PSB_UART_DMA_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(PSB_UART_DMA_RX_IRQn);

}



/** @defgroup STM32F107RC_PSB_BusOperations_Functions Bus Operations Functions
  * @{
  */ 

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

#ifdef HAL_I2C_MODULE_ENABLED
/******************************* I2C Routines**********************************/

void BSP_I2C_Init(void)
{
	hi2c.Instance = I2C2;
	hi2c.Init.ClockSpeed = 100000;
	hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	    if (HAL_I2C_Init(&hi2c) != HAL_OK)
	    {
	      Error_Handler();
	    }

}

static void I2Cx_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};  

  if (hi2c->Instance == PSB_I2Cx)
  {
    /*## Configure the GPIOs ################################################*/  

	/* Enable PSB_I2Cx clock */
	PSB_I2Cx_CLK_ENABLE();

	/* Add delay related to RCC workaround */
	while (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN) != RCC_APB1ENR_I2C2EN) {};

	/* Force the I2C Periheral Clock Reset */
	PSB_I2Cx_FORCE_RESET();

	/* Release the I2C Periheral Clock Reset */
	PSB_I2Cx_RELEASE_RESET();

    /* Enable GPIO clock */
    PSB_I2Cx_SDA_GPIO_CLK_ENABLE();
    PSB_I2Cx_SCL_GPIO_CLK_ENABLE();
      
    /* Configure I2C Tx as alternate function  */
    gpioinitstruct.Pin       = PSB_I2Cx_SCL_PIN;
    gpioinitstruct.Mode      = GPIO_MODE_AF_OD;
    gpioinitstruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PSB_I2Cx_SCL_GPIO_PORT, &gpioinitstruct);
      
    /* Configure I2C Rx as alternate function  */
    gpioinitstruct.Pin       = PSB_I2Cx_SDA_PIN;
    gpioinitstruct.Mode      = GPIO_MODE_AF_OD;
    gpioinitstruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PSB_I2Cx_SDA_GPIO_PORT, &gpioinitstruct);
    
    /*## Configure the Eval I2Cx peripheral #######################################*/ 
    /* Enable PSB_I2Cx clock */
    PSB_I2Cx_CLK_ENABLE();

    /* Add delay related to RCC workaround */
    while (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN) != RCC_APB1ENR_I2C2EN) {};
    
    /* Force the I2C Periheral Clock Reset */  
    PSB_I2Cx_FORCE_RESET();
      
    /* Release the I2C Periheral Clock Reset */  
    PSB_I2Cx_RELEASE_RESET();
    
    /* Enable and set Eval I2Cx Interrupt to the highest priority */
    HAL_NVIC_SetPriority(PSB_I2Cx_EV_IRQn, 0xE, 0);
    HAL_NVIC_EnableIRQ(PSB_I2Cx_EV_IRQn);
    
    /* Enable and set Eval I2Cx Interrupt to the highest priority */
    HAL_NVIC_SetPriority(PSB_I2Cx_ER_IRQn, 0xE, 0);
    HAL_NVIC_EnableIRQ(PSB_I2Cx_ER_IRQn);
  }
}

/**
  * @brief Eval I2Cx Bus initialization
  * @retval None
  */
static void I2Cx_Init(void)
{
  if(HAL_I2C_GetState(&hPSB_I2c) == HAL_I2C_STATE_RESET)
  {
    hPSB_I2c.Instance              = PSB_I2Cx;
    hPSB_I2c.Init.ClockSpeed       = BSP_I2C_SPEED;
    hPSB_I2c.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    hPSB_I2c.Init.OwnAddress1      = 0;
    hPSB_I2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hPSB_I2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hPSB_I2c.Init.OwnAddress2      = 0;
    hPSB_I2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hPSB_I2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
    I2Cx_MspInit(&hPSB_I2c);
    HAL_I2C_Init(&hPSB_I2c);
  }
}

/**
  * @brief  Configures I2C Interrupt.
  * @retval None
  */
static void I2Cx_ITConfig(void)
{
  static uint8_t I2C_IT_Enabled = 0;  
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  if(I2C_IT_Enabled == 0)
  {
    I2C_IT_Enabled = 1;  
    
    /* Enable the GPIO EXTI clock */
    IOE_IT_GPIO_CLK_ENABLE();
    
    gpioinitstruct.Pin   = IOE_IT_PIN;
    gpioinitstruct.Pull  = GPIO_NOPULL;
    gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioinitstruct.Mode  = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(IOE_IT_GPIO_PORT, &gpioinitstruct);
    
    /* Set priority and Enable GPIO EXTI Interrupt */
    HAL_NVIC_SetPriority((IRQn_Type)(IOE_IT_EXTI_IRQn), 0xE, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(IOE_IT_EXTI_IRQn));
  }
}

/**
  * @brief  Reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  MemAddress: Internal memory address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static HAL_StatusTypeDef I2Cx_ReadMultiple(uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(&hPSB_I2c, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, I2cxTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* I2C error occured */
    I2Cx_Error(Addr);
  }
  return status;    
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written 
  * @retval  None
  */
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hPSB_I2c, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(Addr);
  }
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  RegSize: The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: The target register value to be written 
  * @param  Length: buffer size to be written
  * @retval None
  */
static HAL_StatusTypeDef I2Cx_WriteBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hPSB_I2c, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2cxTimeout);

/* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2Cx_Error(Addr);
  }        
  return status;
}

/**
  * @brief  Read a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @retval Data read at register @
  */
static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  
  status = HAL_I2C_Mem_Read(&hPSB_I2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);
 
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(Addr);
  
  }
  return value;
}

/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address 
  * @param  RegSize : The target register size (can be 8BIT or 16BIT)
  * @param  pBuffer: pointer to read data buffer
  * @param  Length: length of the data
  * @retval 0 if no problems to read multiple data
  */
static HAL_StatusTypeDef I2Cx_ReadBuffer(uint16_t Addr, uint8_t Reg, uint16_t RegSize, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hPSB_I2c, Addr, (uint16_t)Reg, RegSize, pBuffer, Length, I2cxTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2Cx_Error(Addr);
  }        
  return status;
}

/**
* @brief  Checks if target device is ready for communication. 
* @note   This function is used with Memory devices
* @param  DevAddress: Target device address
* @param  Trials: Number of trials
* @retval HAL status
*/
static HAL_StatusTypeDef I2Cx_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(&hPSB_I2c, DevAddress, Trials, I2cxTimeout));
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  Addr: I2C Address
  * @retval None
  */
static void I2Cx_Error(uint8_t Addr)
{
  /* De-initialize the IOE comunication BUS */
  HAL_I2C_DeInit(&hPSB_I2c);
  
  /* Re-Initiaize the IOE comunication BUS */
  I2Cx_Init();  
}

#endif /* HAL_I2C_MODULE_ENABLED */

/******************************* SPI Routines**********************************/
#ifdef HAL_SPI_MODULE_ENABLED
/**
  * @brief  Initializes SPI MSP.
  * @retval None
  */
static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /*** Configure the GPIOs ***/  
  /* Enable GPIO clock */
  PSB_SPIx_SCK_GPIO_CLK_ENABLE();
  PSB_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SPI3_ENABLE();
  
  /* configure SPI SCK */
  gpioinitstruct.Pin        = PSB_SPIx_SCK_PIN;
  gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull       = GPIO_NOPULL;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PSB_SPIx_SCK_GPIO_PORT, &gpioinitstruct);

  /* configure SPI MISO and MOSI */
  gpioinitstruct.Pin        = (PSB_SPIx_MISO_PIN | PSB_SPIx_MOSI_PIN);
  gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
  gpioinitstruct.Pull       = GPIO_NOPULL;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PSB_SPIx_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

  /*** Configure the SPI peripheral ***/ 
  /* Enable SPI clock */
  PSB_SPIx_CLK_ENABLE();
}

/**
  * @brief  Initializes SPI HAL.
  * @retval None
  */
static void SPIx_Init(void)
{
  /* DeInitializes the SPI peripheral */
  hPSB_Spi.Instance = PSB_SPIx;
  HAL_SPI_DeInit(&hPSB_Spi);

  /* SPI Config */
  /* SPI baudrate is set to 9 MHz (PCLK2/SPI_BaudRatePrescaler = 72/8 = 9 MHz) */
  hPSB_Spi.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_8;
  hPSB_Spi.Init.Direction          = SPI_DIRECTION_2LINES;
  hPSB_Spi.Init.CLKPhase           = SPI_PHASE_2EDGE;
  hPSB_Spi.Init.CLKPolarity        = SPI_POLARITY_HIGH;
  hPSB_Spi.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  hPSB_Spi.Init.CRCPolynomial      = 7;
  hPSB_Spi.Init.DataSize           = SPI_DATASIZE_8BIT;
  hPSB_Spi.Init.FirstBit           = SPI_FIRSTBIT_MSB;
  hPSB_Spi.Init.NSS                = SPI_NSS_SOFT;
  hPSB_Spi.Init.TIMode             = SPI_TIMODE_DISABLE;
  hPSB_Spi.Init.Mode               = SPI_MODE_MASTER;
  
  SPIx_MspInit(&hPSB_Spi);
  if (HAL_SPI_Init(&hPSB_Spi) != HAL_OK)
  {
    /* Should not occur */
    while(1) {};
  }
}

/**
  * @brief SPI Read 4 bytes from device
  * @retval Read data
*/
static uint32_t SPIx_Read(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t          readvalue = 0;
  uint32_t          writevalue = 0xFFFFFFFF;
  
  status = HAL_SPI_TransmitReceive(&hPSB_Spi, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }

  return readvalue;
}

/**
  * @brief SPI Write a byte to device
  * @param Value: value to be written
  * @retval None
  */
static void SPIx_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&hPSB_Spi, (uint8_t*) &Value, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief SPI error treatment function
  * @retval None
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hPSB_Spi);
  
  /* Re- Initiaize the SPI communication BUS */
  SPIx_Init();
}
#endif /* HAL_SPI_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32F107RC_PSB_LinkOperations_Functions Link Operations Functions
  * @{
  */ 

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

#ifdef HAL_I2C_MODULE_ENABLED
/***************************** LINK IOE ***************************************/

/**
  * @brief  Initializes IOE low level.
  * @retval None
  */
void IOE_Init(void) 
{
  I2Cx_Init();
}

/**
  * @brief  Configures IOE low level Interrupt.
  * @retval None
  */
void IOE_ITConfig(void)
{
  I2Cx_ITConfig();
}

/**
  * @brief  IOE writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Value: Data to be written
  * @retval None
  */
void IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteData(Addr, Reg, Value);
}

/**
  * @brief  IOE reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @retval Read data
  */
uint8_t IOE_Read(uint8_t Addr, uint8_t Reg)
{
  return I2Cx_ReadData(Addr, Reg);
}

/**
  * @brief  IOE reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
 return I2Cx_ReadMultiple(Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  IOE delay. 
  * @param  Delay: Delay in ms
  * @retval None
  */
void IOE_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

#endif /* HAL_I2C_MODULE_ENABLED */

void Error_Handler(void)
{
	  /* Turn LED2 on */
	  BSP_LED_On(LED1);
	  BSP_LED_On(LED2);
	  while(1)
	  {
	    /* Error if LED1 AND LED2 is FAST blinking  */
		BSP_LED_On(LED1);
	    BSP_LED_Toggle(LED2);
	    HAL_Delay(200);
	  }
}


/***************************END OF FILE****/
