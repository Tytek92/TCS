/*
 * crc.h
 *
 *  Created on: 25 gru 2016
 *      Author: Tymoteusz
 */

#ifndef MY_CODE_CRC_H_
#define MY_CODE_CRC_H_

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

//void CRC_DMA_Config(uint32_t Address);

#endif /* MY_CODE_CRC_H_ */
