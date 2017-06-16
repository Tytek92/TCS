/*
 * callbacks.h
 *
 *  Created on: 29 sty 2017
 *      Author: Tymoteusz
 */

#ifndef MY_CODE_CALLBACKS_H_
#define MY_CODE_CALLBACKS_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"

#include "Basic_global_structures/global_structures.h"
#include "My_code/BCD_display_driver.h"
#include "My_code/crc.h"
#include "My_code/timeout.h"

void TransferComplete();
void DMA_CRC_COMPLETE_Callback();
void HOST_COMMAND_COMPLETE_Callback();

#endif /* MY_CODE_CALLBACKS_H_ */
