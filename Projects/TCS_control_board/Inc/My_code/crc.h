/*
 * crc.h
 *
 *  Created on: 25 gru 2016
 *      Author: Tymoteusz
 */

#ifndef MY_CODE_CRC_H_
#define MY_CODE_CRC_H_

#include "stdint.h"
#include "stm32f4xx.h"

#define DATATYPE uint32_t

#define POLYSIZE CRC_PolSize_32
#define POLYRETURN uint32_t
#define POLYNOME 0x04C11DB7
//polynome in binary: 0000 0100 1100 0001 0001 1101 1011 0111

#define DMA_CHANNEL DMA_CHANNEL_0
#define DMA_STREAM DMA2_Stream0
#define DMA_IRQ DMA2_Stream0_IRQn

#define BUFFER_SIZE    8191 /* data buffer size */
#define LOWER 0 /* lower limit */
#define UPPER BUFFER_SIZE /* upper limit */
#define STEP 1 /* step size */
#define INITIAL_CRC_VALUE 0xFFFFFFFF /* Initial CRC Data register value */

/*
 * SERIAL BUFFER SIZE
 */
#define SERIAL_BUF_SIZE_Uint8t 8

/*
 * Union allows me to use char for interpretation
 * and uint32_t for CRC calculation
 */
union SERIAL_BUF{
	char serial_buf_char[SERIAL_BUF_SIZE_Uint8t];
	uint32_t serial_buf_4char[SERIAL_BUF_SIZE_Uint8t/4];
};

POLYRETURN CRC_CalcBlockCRCxxbits();
/*
 * THIS IS CODE THAT REPRESENTS STM32 CRC CALCULATION!!
 * uint32_t crc32(uint32_t crc, uint32_t data)
{
  int i;
  crc=crc^data;
  for(i=0;i<32;i++)
    {
      if(crc & 0x80000000)
        {
          crc=(crc<<1)^0x04c11db7; //polynomial used in STM32
        }
      else
        {
          crc=(crc<<1);
        }
    }
  return (crc);
}
 *
 */


#endif /* MY_CODE_CRC_H_ */
