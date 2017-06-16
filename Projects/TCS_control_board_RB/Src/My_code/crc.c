/*
 * crc.c
 *
 *  Created on: 25 gru 2016
 *      Author: Tymoteusz
 */

#include "My_code/crc.h"

//#include "stm32f401xc.h"

__IO uint32_t ProcessTimeConfCRC = 0;
__IO uint32_t ProcessTimeConfDMA = 0;
__IO uint32_t ProcessTimeDMA = 0;
__IO uint32_t ProcessTimeCPU = 0;
__IO uint32_t ProcessTimeIRQDMA = 0;
__IO POLYRETURN CRCValue = 0;
__IO POLYRETURN ComputedCRCDMA = 0;

union SERIAL_BUF serial_buffer;


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
POLYRETURN CRC_CalcBlockCRCxxbits()
{
  uint32_t windex = 0;
  uint32_t dummy32=0;

  /* A loop to compute the CRC of Data Buffer */
  for (windex = 0; windex < 2; windex ++)
  {
	  //this writes to CRC->DR (via cast to uint32_t the CRC_BASE address)
    //*(__IO DATATYPE*)(CRC_BASE) = (uint32_t)dummy32;
	  CRC->DR = serial_buffer.serial_buf_4char[windex];
  }
  /* Return the content of the CRC Data Register */
  return (POLYRETURN)((CRC->DR));
}
