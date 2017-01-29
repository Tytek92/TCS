/*
 * timeout.c
 *
 *  Created on: 23 sty 2017
 *      Author: lipinskt
 */

#include "My_code/timeout.h"

void Timeout_TIM16_Init(void)
{
	/*
	 * MCU clock: 48MHz
	 * Timeout: 50ms
	 * PSC: 100
	 * ARR: 23999
	 *
	 * Timeout: 10ms
	 * PSC: 10
	 * ARR: 47999
	 */
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; // turn on clock for TIM16
	TIM16->CR1 |= TIM_CR1_ARPE; //auto reload preload
	//when ARR value is changed, then it is buffered until next overflow/compara match ect.
	//to prevent for exampe missing ARR value and couting up to max CNT value
	TIM16->CR1 |= TIM_CR1_OPM; //one pulse mode. Timer stops after update event.
	TIM16->DIER |= TIM_DIER_UIE; // update interrupt enable
	TIM16->PSC = 100;
	TIM16->ARR = 23999;
    HAL_NVIC_SetPriority(TIM16_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM16_IRQn);
}

void Timeout_start()
{
	  TIM16->CR1 |= TIM_CR1_CEN;
}

void Timeout_abort()
{
	  TIM16->CR1 &= ~TIM_CR1_CEN;
	  TIM16->CNT = 0;
}
