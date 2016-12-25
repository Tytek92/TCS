/*
 * crc.c
 *
 *  Created on: 25 gru 2016
 *      Author: Tymoteusz
 */

#include "My_code/crc.h"
#include "stdint.h"
#include "stm32f4xx.h"
//#include "stm32f401xc.h"

__IO uint32_t ProcessTimeConfCRC = 0;
__IO uint32_t ProcessTimeConfDMA = 0;
__IO uint32_t ProcessTimeDMA = 0;
__IO uint32_t ProcessTimeCPU = 0;
__IO uint32_t ProcessTimeIRQDMA = 0;
__IO POLYRETURN CRCValue = 0;
__IO POLYRETURN ComputedCRCDMA = 0;

/*
 * pBuffer must be a straight multiplication of 32 bits:
 * 		* 4 bytes
 * 		* 8 bytes
 * 		* 12 bytes
 * 		* and so on
 * 		to ease up, use char pBuffer[4n]
 *
 * 		*size is the number of 32bit data parts
 */
POLYRETURN CRC_CalcBlockCRCxxbits(const char pBuffer[], const uint8_t size)
{
  uint32_t windex = 0;
  uint32_t dummy32=0;

  /* A loop to compute the CRC of Data Buffer */
  for (windex = LOWER; windex < size*4; windex += 4)
  {
	  //get 4 8_bit chars and merge them into one 32bit variable
	  dummy32 = (pBuffer[windex]<<24) | (pBuffer[windex+1]<<16) | (pBuffer[windex+2]<<8) | (pBuffer[windex+3]);
	  //this writes to CRC->DR (via cast to uint32_t the CRC_BASE address)
    //*(__IO DATATYPE*)(CRC_BASE) = (uint32_t)dummy32;
	  CRC->DR = dummy32;
  }
  /* Return the content of the CRC Data Register */
  return (POLYRETURN)((CRC->DR));
}
