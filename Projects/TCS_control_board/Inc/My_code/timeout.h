/*
 * timeout.h
 *
 *  Created on: 28 sty 2017
 *      Author: Tymoteusz
 */

#ifndef MY_CODE_TIMEOUT_H_
#define MY_CODE_TIMEOUT_H_


#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

void Timeout_TIM9_Init(void);
void Timeout_start();
void Timeout_abort();

#endif /* MY_CODE_TIMEOUT_H_ */
