/*
 * timeout.c
 *
 *  Created on: 28 sty 2017
 *      Author: Tymoteusz
 */
#include "My_code/timeout.h"


volatile uint8_t TimeoutEventFlag = 0;

/*
 * TIM9 timeout timer config.
 */
/* TIM9 init function */
void Timeout_TIM9_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN; // turn on clock for TIM9
	TIM9->CR1 |= TIM_CR1_ARPE; //auto reload preload
	//when ARR value is changed, then it is buffered until next overflow/compara match ect.
	//to prevent for exampe missing ARR value and couting up to max CNT value
	TIM9->CR1 |= TIM_CR1_OPM; //one pulse mode. Timer stops after update event.
	TIM9->DIER |= TIM_DIER_UIE; // update interrupt enable
	TIM9->PSC = 10;
	TIM9->ARR = 63999;
}

void Timeout_start()
{
	  TIM9->CR1 |= TIM_CR1_CEN;
}

void Timeout_abort()
{
	  TIM9->CR1 &= ~TIM_CR1_CEN;
	  TIM9->CNT = 0;
}
