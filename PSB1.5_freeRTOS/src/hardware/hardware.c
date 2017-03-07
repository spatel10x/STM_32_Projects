/* hardware.c */

#include "stm32f1xx_hal.h"     /* HAL */
#include "PSBv1_5.h" /* BSP */
#include "hardware/hardware.h"
#include "hardware/uart.h"

void prvSystemClockConfig();

/* Main hardware-setup function, called from main.c */
void vHardwareSetup(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    prvSystemClockConfig();
    xHardwareUartInit();

    /* Initialize all configured peripherals */
    BSP_LED_Init(LED1);
    BSP_LED_Init(LED2);

	#ifdef HAL_UART_MODULE_ENABLED
	BSP_CAN_Init();
	#endif

	#ifdef HAL_UART_MODULE_ENABLED
	BSP_UART_Init();
	#endif

	#ifdef HAL_ADC_MODULE_ENABLED
	BSP_ADC1_Init();
	#endif

	#ifdef HAL_I2C_MODULE_ENABLED
	BSP_I2C_Init();
	#endif

	#ifdef HAL_SPI_MODULE_ENABLED
	void BSP_SPI_Init();
	#endif


}

/* System Clock Configuration by ST */


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 36000000
  *            HCLK(Hz)                       = 36000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 5
  *            HSE PREDIV2                    = 5
  *            PLL2MUL                        = 8
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */

void prvSystemClockConfig(void)
{

	  RCC_ClkInitTypeDef clkinitstruct = {0};
	  RCC_OscInitTypeDef oscinitstruct = {0};

	  /* Configure PLLs ------------------------------------------------------*/
	  /* PLL2 configuration: PLL2CLK = (HSE / HSEPrediv2Value) * PLL2MUL = (8 / 2) * 9 = 36 MHz */
	  /* PREDIV1 configuration: PREDIV1CLK = PLL2CLK / HSEPredivValue = 40 / 5 = 8 MHz */
	  /* PLL configuration: PLLCLK = PREDIV1CLK * PLLMUL = 8 * 9 = 72 MHz */

	  /* Enable HSE Oscillator and activate PLL with HSE as source */
	  oscinitstruct.OscillatorType        = RCC_OSCILLATORTYPE_HSE;
	  oscinitstruct.HSEState              = RCC_HSE_ON;
	  oscinitstruct.HSEPredivValue        = RCC_HSE_PREDIV_DIV2;
	  oscinitstruct.Prediv1Source         = RCC_PLLSOURCE_HSE;
	  oscinitstruct.PLL.PLLState          = RCC_PLL_ON;
	  oscinitstruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
	  oscinitstruct.PLL.PLLMUL            = RCC_PLL_MUL9;
	  oscinitstruct.PLL2.PLL2State        = RCC_PLL2_ON;
	  oscinitstruct.PLL2.PLL2MUL          = RCC_PLL2_MUL8;
	  oscinitstruct.PLL2.HSEPrediv2Value  = RCC_HSE_PREDIV2_DIV5;
	  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
	  {
	    /* Initialization Error */
	    while(1);
	  }

	  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	     clocks dividers */
	  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
	  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
	  {
	    /* Initialization Error */
	    while(1);
	  }
}

