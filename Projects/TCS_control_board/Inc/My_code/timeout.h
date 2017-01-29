/*
 * timeout.h
 *
 *  Created on: 28 sty 2017
 *      Author: Tymoteusz
 */

#ifndef MY_CODE_TIMEOUT_H_
#define MY_CODE_TIMEOUT_H_

typedef enum
{
  RECV_MODE		= 0x00U,
  RESP_MODE		= 0x01U,
} Timeout_Mode;

typedef enum
{
  ACK		= 0x00U,
  ERR		= 0x01U,
  NONE		= 0x02U,
} Timeout_Message;


#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

void Timeout_TIM9_Init(void);
void Timeout_start(Timeout_Mode mode, Timeout_Message message);
void Timeout_abort();

#endif /* MY_CODE_TIMEOUT_H_ */
