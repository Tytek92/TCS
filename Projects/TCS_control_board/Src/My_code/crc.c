/*
 * crc.c
 *
 *  Created on: 25 gru 2016
 *      Author: Tymoteusz
 */

#include "My_code/crc.h"
#include "stdint.h"
#include "stm32f4xx.h"

__IO uint32_t ProcessTimeConfCRC = 0;
__IO uint32_t ProcessTimeConfDMA = 0;
__IO uint32_t ProcessTimeDMA = 0;
__IO uint32_t ProcessTimeCPU = 0;
__IO uint32_t ProcessTimeIRQDMA = 0;
__IO POLYRETURN CRCValue = 0;
__IO POLYRETURN ComputedCRCDMA = 0;


