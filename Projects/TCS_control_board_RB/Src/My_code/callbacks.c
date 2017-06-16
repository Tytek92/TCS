/*
 * callbacks.c
 *
 *  Created on: 29 sty 2017
 *      Author: Tymoteusz
 */

#include "My_code/callbacks.h"


extern volatile uint8_t TimeoutEventFlag;
extern char errorframe[];
extern char errorframe2[];

//First buffer for UART (DMA)
//volatile uint32_t FrameBuffer[20] = {0};
extern volatile union FrameBuffer FrameBuffer;
//Second buffer for UART (DMA)
extern volatile union FrameBuffer FrameBuffer2;
//volatile uint32_t FrameBuffer2[20] = {0};
//Which FrameBuffer is send to CRC block
extern volatile uint8_t FrameBufferIndicator;
//Temporary variable for indication of correct CRC
extern volatile uint8_t truth;

//data structure that holds data received from host controller
extern volatile union TCS_input_data TCS_input_data;

// Timeout mode variable
extern volatile uint8_t TimeoutMode;

/*
 * This function is called when FrameBuffer2[20] is full
 */
void TransferComplete()
{
	/*
	 * Here instruct DMA to calculate CRC, check it with what was received.
	 * Do that by setting a mutex/semaphore to let task execute. Task will wait for second DMA callback [TODO] to proceed
	 * If CRC matches proceed with filling proper structure [#########] fields with data from the frame
	 * does that in turns with void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) located in stm32f4xx_it.c
	 */
	//check if this function executes quickly enough, otherwise try doing this using registers!
	FrameBufferIndicator = 2;
	//reset CRC so it is 0xFFFFFFFF
	CRC->CR |= CRC_CR_RESET;
	//Start transfer to calculate CRC
	HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,(uint32_t)FrameBuffer2.word,(uint32_t)&CRC->DR,19);
}

/*
 * This function is called when DMA finishes transfering data from FrameBuffer/FrameBuffer2 to CRC data register
 */
void DMA_CRC_COMPLETE_Callback()
{
	HAL_StatusTypeDef status;
	SEG_A_REG = 0;
	SEG_B_REG = 0;
	SEG_C_REG = 0;
	SEG_D_REG = 0;
	SEG_E_REG = 0;
	SEG_F_REG = 0;
	SEG_G_REG = 0;
	//Display_char(FrameBufferIndicator);
	//Check the CRC value
	uint32_t dummy = FrameBuffer.word[0];
	uint32_t dummy2 = FrameBuffer2.word[0];
	if(FrameBufferIndicator == 2)//current buffer is M1 - FrameBuffer2
	{
		if(0==CRC->DR)//CRC is correct!
		{
			Timeout_abort(); //stop the timeout timer
			USART2->CR1 |= USART_CR1_RXNEIE; //enable incoming transmission interrupt
			HAL_UART_Transmit_IT(&huart2,errorframe,8);//SEND ACK
			Timeout_start(RESP_MODE, ACK);//Start the timeout timer with response mode
			truth = FrameBuffer2.byte[0]/25;
			Display_char(truth);
			//also setup mutex on TCS_input_data
			HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1,(uint32_t)FrameBuffer2.word,(uint32_t)TCS_input_data.Concatenated_Fields,18);
		}
		else
		{	//CRC is not correct
			Timeout_abort(); //stop the timeout timer
			USART2->CR1 |= USART_CR1_RXNEIE; //enable incoming transmission interrupt
			//Send ERR
			HAL_UART_Transmit_IT(&huart2,errorframe2,8);
			Timeout_start(RESP_MODE, ERR);//Start the timeout timer with response mode
			//check if DMA_SxNDTR is equal to 76 (no orpahned bytes)
			for(uint32_t i=0; i<1000; i++)
			{
				asm("nop");
			}
			if(76 != DMA1_Stream5 -> NDTR)
			{
				//something went wrong! Clean up mess
				DMA_Reset();
			}
			truth = 2;
			Display_char(truth);
		}
	}
	else//current buffer is M0 - FrameBuffer1
	{
		if(0==CRC->DR)//CRC is correct!
		{
			Timeout_abort(); //stop the timeout timer
			USART2->CR1 |= USART_CR1_RXNEIE; //enable incoming transmission interrupt
			HAL_UART_Transmit_IT(&huart2,errorframe,8);//SEND ACK
			Timeout_start(RESP_MODE, ACK);//Start the timeout timer with response mode
			truth = FrameBuffer2.byte[0]/25;
			Display_char(truth);
			//also setup mutex on TCS_input_data
			HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1,(uint32_t)FrameBuffer.word,(uint32_t)TCS_input_data.Concatenated_Fields,18);
		}
		else
		{	//CRC is not correct
			Timeout_abort(); //stop the timeout timer
			USART2->CR1 |= USART_CR1_RXNEIE; //enable incoming transmission interrupt
			//Send ERR
			HAL_UART_Transmit_IT(&huart2,errorframe2,8);
			Timeout_start(RESP_MODE, ERR);//Start the timeout timer with response mode
			//check if DMA_SxNDTR is equal to 76 (no orpahned bytes)
			for(uint32_t i=0; i<1000; i++)
			{
				asm("nop");
			}
			if(76 != DMA1_Stream5 -> NDTR)
			{
				//something went wrong! Clean up mess
				DMA_Reset();
			}
			truth = 4;
			Display_char(truth);
		}
	}
}

void HOST_COMMAND_COMPLETE_Callback()
{
	//here free mutexes off TCS_input_data
}
