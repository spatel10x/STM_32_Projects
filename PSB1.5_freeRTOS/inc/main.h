

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "PSBv1_5.h"
#include "stm32f1xx_it.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

#include "error.h"
#include "hardware/hardware.h"

/* Tasks */
#include "task/cli.h"
#include "task/heartbeat.h"

/* CLI */
#include "cli/tasks_info.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
