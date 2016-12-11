/*
 * TIM10_general_task.c
 *
 *  Created on: 11 gru 2016
 *      Author: Tymoteusz
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

// When TIM10 counter meets AutoReload Value this callback is called by HAL

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	 if (htim->Instance == TIM10)
	 {
		 //temporary:
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	 }
}
