
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _PSBV1_5_H
#define _PSBV1_5_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_conf.h"

#ifdef HAL_I2C_MODULE_ENABLED
//#include "PSBv1_5_io.h"
#endif /* HAL_I2C_MODULE_ENABLED */

/** @defgroup STM32F107RC_PSB_Exported_Types Exported Types
  * @{
  */

/**
 * @brief LED Types Definition
 */
typedef enum 
{
  LED1 = 0,
  LED2 = 1,


  LED_YELLOW  = LED1,
  LED_RED = LED2,


} Led_TypeDef;



/*================ LED Defines ================================  */


#define LEDn                             2

#define LED1_PIN                         GPIO_PIN_5             /* PB.5*/
#define LED1_GPIO_PORT                   GPIOB
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()
  
#define LED2_PIN                         GPIO_PIN_6            /* PB.6*/
#define LED2_GPIO_PORT                   GPIOB
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()


#define LEDx_GPIO_CLK_ENABLE(__LED__)    do { if ((__LED__) == LED1) LED1_GPIO_CLK_ENABLE(); else \
                                              if ((__LED__) == LED2) LED2_GPIO_CLK_ENABLE();} while(0)


#define LEDx_GPIO_CLK_DISABLE(__LED__)   (((__LED__) == LED1) ? LED1_GPIO_CLK_DISABLE() :\
                                          ((__LED__) == LED2) ? LED2_GPIO_CLK_DISABLE() : 0 )


/*================ CAN Defines ================================  */

/* Definition for PSB_CAN clock resources */
#define PSB_CAN                            CAN1
#define PSB_CAN_CLK_ENABLE()               __HAL_RCC_CAN1_CLK_ENABLE()
#define PSB_CAN_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()

#define PSB_CAN_FORCE_RESET()              __HAL_RCC_CAN1_FORCE_RESET()
#define PSB_CAN_RELEASE_RESET()            __HAL_RCC_CAN1_RELEASE_RESET()

/* Definition for PSB_UART Pins */
#define PSB_CAN_TX_PIN                    GPIO_PIN_8
#define PSB_CAN_TX_GPIO_PORT              GPIOB
#define PSB_CAN_RX_PIN                    GPIO_PIN_9
#define PSB_CAN_RX_GPIO_PORT              GPIOB

/* Definition for AFIO Remap */
#define PSB_CAN_AFIO_REMAP_CLK_ENABLE()   __HAL_RCC_AFIO_CLK_ENABLE()
#define PSB_CAN_AFIO_REMAP_RX_TX_PIN()    __HAL_AFIO_REMAP_CAN1_2()

/* Definition for PSB_UART's NVIC */

#define PSB_CAN_RX0_IRQn                   USB_LP_CAN1_RX0_IRQn
#define PSB_CAN_RX0_IRQHandler             USB_LP_CAN1_RX0_IRQHandler

#define PSB_CAN_RX1_IRQn                   CAN1_RX1_IRQn
#define PSB_CAN_RX1_IRQHandler             CAN1_RX1_IRQHandler

#define PSB_CAN_TX_IRQn                   USB_HP_CAN1_TX_IRQn
#define PSB_CAN_TX_IRQHandler             USB_HP_CAN1_TX_IRQHandler

/*================ UART Defines ================================  */

/* Definition for PSB_UART clock resources */
#define PSB_UART                           USART2
#define PSB_UART_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define PSB_UART_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define PSB_UART_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define PSB_UART_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define PSB_UART_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for PSB_UART Pins */
#define PSB_UART_TX_PIN                    GPIO_PIN_2
#define PSB_UART_TX_GPIO_PORT              GPIOA
#define PSB_UART_RX_PIN                    GPIO_PIN_3
#define PSB_UART_RX_GPIO_PORT              GPIOA

/* Definition for PSB_UART's DMA */

#define PSB_UART_TX_DMA_CHANNEL             DMA1_Channel7
#define PSB_UART_RX_DMA_CHANNEL             DMA1_Channel6



/* Definition for PSB_UART's NVIC */
#define PSB_UART_DMA_TX_IRQn                DMA1_Channel7_IRQn
#define PSB_UART_DMA_RX_IRQn                DMA1_Channel6_IRQn
#define PSB_UART_DMA_TX_IRQHandler          DMA1_Channel7_IRQHandler
#define PSB_UART_DMA_RX_IRQHandler          DMA1_Channel6_IRQHandler

/* Definition for PSB_UART's NVIC */
#define PSB_UART_IRQn                      USART2_IRQn
#define PSB_UART_IRQHandler                USART2_IRQHandler
#define PSB_UART_TIMEOUT				   100
/** 
  * @brief  IO Expander Interrupt line on EXTI  
  */ 
#define IOE_IT_PIN                       GPIO_PIN_14
#define IOE_IT_GPIO_PORT                 GPIOB
#define IOE_IT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define IOE_IT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()
#define IOE_IT_EXTI_IRQn                 EXTI15_10_IRQn       
#define IOE_IT_EXTI_IRQHANDLER           EXTI15_10_IRQHandler


#define LTC2309_I2C_ADDRESS                  0x10


//============================ ADC DEFINES ============================================================================================//

/**ADC1 GPIO Configuration
PC0     ------> ADC1_IN10
PC1     ------> ADC1_IN11
PC2     ------> ADC1_IN12
PC3     ------> ADC1_IN13
PA0     ------> ADC1_IN0
PA1     ------> ADC1_IN1
PA4     ------> ADC1_IN4
PB0     ------> ADC1_IN8 **/
#define ADCXn                             1
#define PSB_ADC                       	 ADC1
#define PSB_ADC_CLK_ENABLE()          	 __HAL_RCC_ADC1_CLK_ENABLE()
#define PSB_CAN1_CLK_DISABLE()        	 __HAL_RCC_ADC1_CLK_DISABLE()

#define PSB_ADC_PINS_PORTC            	 GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
#define PSB_ADC_PORTC         		  	 GPIOC
#define PSB_ADC_PORTC_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()
#define PSB_ADC_PORTC_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOC_CLK_DISABLE()

#define PSB_ADC_PINS_PORTA           	  GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4
#define PSB_ADC_PORTA         		 	  GPIOA
#define PSB_ADC_PORTA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define PSB_ADC_PORTA_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()

#define PSB_ADC_PINS_PORTB          	   GPIO_PIN_0
#define PSB_ADC_PORTB         		 	  GPIOB
#define PSB_ADC_PORTB_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define PSB_ADC_PORTB_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()


#define ADCx_ENABLE(__INDEX__)              do { if((__INDEX__) ==ADCX1) PSB_ADC_CLK_ENABLE();} while(0)
#define ADCx_DISABLE(__INDEX__)             (((__INDEX__) ==ADCX1) ? PSB_ADC_CLK_ENABLE() : 0)

#define ADCx_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) ==ADCX1) __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();} while(0)
#define ADCx_GPIO_CLK_DISABLE(__INDEX__)     do { if((__INDEX__) ==ADCX1) __HAL_RCC_GPIOC_CLK_DISABLE();__HAL_RCC_GPIOA_CLK_DISABLE();__HAL_RCC_GPIOB_CLK_DISABLE(); } while(0)

/*############################# I2Cx ####################################*/
/* User can use this section to tailor I2Cx instance used and associated 
   resources */
/* Definition for I2Cx Pins */
#define PSB_I2Cx_SCL_PIN                       GPIO_PIN_10        /* PB.10*/
#define PSB_I2Cx_SCL_GPIO_PORT                 GPIOB
#define PSB_I2Cx_SDA_PIN                       GPIO_PIN_11        /* PB.11*/
#define PSB_I2Cx_SDA_GPIO_PORT                 GPIOB

/* Definition for I2Cx clock resources */
#define PSB_I2Cx                               I2C2
#define PSB_I2Cx_CLK_ENABLE()                  __HAL_RCC_I2C2_CLK_ENABLE()
#define PSB_I2Cx_SDA_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define PSB_I2Cx_SCL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define PSB_I2Cx_FORCE_RESET()                 __HAL_RCC_I2C2_FORCE_RESET()
#define PSB_I2Cx_RELEASE_RESET()               __HAL_RCC_I2C2_RELEASE_RESET()
    
/* Definition for I2Cx's NVIC */
#define PSB_I2Cx_EV_IRQn                       I2C2_EV_IRQn
#define PSB_I2Cx_EV_IRQHandler                 I2C2_EV_IRQHandler
#define PSB_I2Cx_ER_IRQn                       I2C2_ER_IRQn
#define PSB_I2Cx_ER_IRQHandler                 I2C2_ER_IRQHandler

/* I2C clock speed configuration (in Hz) */
#ifndef BSP_I2C_SPEED
 #define BSP_I2C_SPEED                            400000
#endif /* I2C_SPEED */


/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define PSB_I2Cx_TIMEOUT_MAX                   3000

/*##################### SPI3 ###################################*/
#define PSB_SPIx                               SPI3
#define PSB_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI3_CLK_ENABLE()

#define PSB_SPIx_SCK_GPIO_PORT                 GPIOC             /* PC.10*/
#define PSB_SPIx_SCK_PIN                       GPIO_PIN_10
#define PSB_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define PSB_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()

#define PSB_SPIx_MISO_MOSI_GPIO_PORT           GPIOC
#define PSB_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()
#define PSB_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOC_CLK_DISABLE()
#define PSB_SPIx_MISO_PIN                      GPIO_PIN_11       /* PC.11*/
#define PSB_SPIx_MOSI_PIN                      GPIO_PIN_12       /* PC.12*/
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define PSB_SPIx_TIMEOUT_MAX                   1000




uint32_t                BSP_GetVersion(void);
void                    BSP_LED_Init(Led_TypeDef Led);
void                    BSP_LED_On(Led_TypeDef Led);
void                    BSP_LED_Off(Led_TypeDef Led);
void                    BSP_LED_Toggle(Led_TypeDef Led);

#ifdef HAL_UART_MODULE_ENABLED
void                    BSP_UART_Init();
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
void 					BSP_CAN_Init();
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
void 					BSP_ADC1_Init();
#endif /* HAL_CAN_MODULE_ENABLED */

void BSP_System_Init(void);
void PSB_DMA_Init(void);
void Error_Handler(void);
/**
  * @}
  */


#ifdef __cplusplus
}
#endif
  
#endif /* _PSBV1_5_H */

/****************************END OF FILE****/
