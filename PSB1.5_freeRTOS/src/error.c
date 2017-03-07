/* file: error.c */

#include <stdio.h>

#include "error.h"

#include "hardware/hardware.h"
#include "hardware/uart.h"

/* 
 * Error handling function.
 *
 */

/* Signal fatal error by blinking all LEDs */
void vErrorFatalLoop(void)
{
    while(1)
    {
        /* Flash all LEDs */
        BSP_LED_Toggle(LED1);
        BSP_LED_Toggle(LED2);
        HAL_Delay(200);
    }
}

void vErrorFatal(char* comment)
{
    /* printf("vFatalError: %s\n", comment); */
    vErrorFatalLoop();
}

void vErrorWarning(char* comment)
{
    /* printf("vFatalWarning: %s\n", comment); */
}

