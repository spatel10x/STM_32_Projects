/* heartbeat.c */
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "error.h"
#include "task/heartbeat.h"
#include "hardware/hardware.h"

void vTaskHeartbeatStart(void);
void prvTaskHeartbeat(void *pvParameters);

/* Run LEDs in circle */
void vTaskHeartbeatStart(void)
{
    if(xTaskCreate(prvTaskHeartbeat, "tHBEAT", configMINIMAL_STACK_SIZE, NULL, 1, NULL) != pdPASS)
    {
        vErrorFatal("prvTaskHeartbeat: Unable to start");
    }
}

void prvTaskHeartbeat(void *pvParams)
{
    while(1)
    {
        BSP_LED_Toggle(LED1);
        HAL_Delay(100);
        BSP_LED_Toggle(LED2);
        HAL_Delay(100);


    }
}
