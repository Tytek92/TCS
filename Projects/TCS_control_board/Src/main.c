/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "Basic_global_structures/global_structures.h"
#include "My_code/BCD_display_driver.h"
#include "My_code/crc.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//Global variable for USART RX DMA circular operation
char Rx_single_char = '\000';
//First buffer for UART (DMA)
//volatile uint32_t FrameBuffer[20] = {0};
volatile union FrameBuffer FrameBuffer;
//Second buffer for UART (DMA)
volatile union FrameBuffer FrameBuffer2;
//volatile uint32_t FrameBuffer2[20] = {0};
//Which FrameBuffer is send to CRC block
volatile uint8_t FrameBufferIndicator = 0;
//Temporary variable for indication of correct CRC
volatile uint8_t truth=0;

//data structure that holds data received from host controller

volatile union TCS_input_data TCS_input_data;

//Global variable for frame completition
volatile char Rx_whole_frame_buffer[SERIAL_BUF_SIZE_Uint8t];

//Global variable for frame completition
uint8_t serial_buffer_counter =0;
//
char errorframe[8] = {0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
	if(FrameBufferIndicator == 2)
	{
		if(0==CRC->DR)
		{
			truth = 1;
			Display_char(truth);
			//also setup mutex on TCS_input_data
			HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1,(uint32_t)FrameBuffer2.word,(uint32_t)TCS_input_data.Concatenated_Fields,18);
		}
		else
		{
			HAL_UART_Transmit(&huart2,errorframe,8,5000);
			HAL_Delay(8);
			int i = 0;
			truth = 2;
			Display_char(truth);
			USART2 -> CR3 &= ~USART_CR3_DMAR;
			USART2 -> CR1 &= ~USART_CR1_UE;
			DMA1_Stream5->CR &= ~DMA_SxCR_EN;
			while(!DMA1_Stream5->CR){}
			DMA1_Stream5 -> NDTR = 76;
			DMA1 -> HIFCR |= DMA_HIFCR_CTCIF5;

			USART2->SR &= ~USART_SR_TC;

			for(i=0; i<2000; i++)
			{
				asm("nop");
			}
			DMA1_Stream5->CR |= DMA_SxCR_EN;
			USART2 -> CR3 |= USART_CR3_DMAR;
			USART2 -> CR1 |= USART_CR1_UE;
		}
	}
	else
	{
		if(0==CRC->DR)
		{
			truth = 3;
			Display_char(truth);
			//also setup mutex on TCS_input_data
			HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1,(uint32_t)FrameBuffer.word,(uint32_t)TCS_input_data.Concatenated_Fields,18);
		}
		else
		{
			HAL_UART_Transmit(&huart2,errorframe,8,5000);
			HAL_Delay(8);
			int i = 0;
			truth = 4;
			Display_char(truth);
			USART2 -> CR3 &= ~USART_CR3_DMAR;
			USART2 -> CR1 &= ~USART_CR1_UE;
			DMA1_Stream5->CR &= ~DMA_SxCR_EN;
			while(!DMA1_Stream5){}
			DMA1_Stream5 -> NDTR = 76;
			DMA1 -> HIFCR |= DMA_HIFCR_CTCIF5;

			USART2->SR &= ~USART_SR_TC;

			for(i=0; i<2000; i++)
			{
				asm("nop");
			}
			DMA1_Stream5->CR |= DMA_SxCR_EN;
			USART2 -> CR3 |= USART_CR3_DMAR;
			USART2 -> CR1 |= USART_CR1_UE;

		}
	}
}

void HOST_COMMAND_COMPLETE_Callback()
{
	//here free mutexes off TCS_input_data
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  /*
   * Timeout timer test.
   */
  //run my config for timeout timer
  Timeout_TIM9_Init();
TIM9->CR1 |= TIM_CR1_CEN;

	/*
	 * Short description:
	 * DMA reads from UART2 Rx 8bits at a time, and moves that data
	 * in turns into memory M0 and M1. (does that N times, defined in HAL_UART_Receive_DMA(&huart2, &FrameBuffer, N);)
	 * When transfer to M0 is complete then HAL_UART_RxCpltCallback is invoked.
	 * When transfer to M1 is complete then my function is invoked. That is setup
	 * by calling HAL_DMA_RegisterCallback(&hdma_usart2_rx, HAL_DMA_XFER_M1CPLT_CB_ID,TransferComplete);
	 */
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW; // in future check if needs to be changed
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_usart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	hdma_usart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_usart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
	HAL_DMA_Init(&hdma_usart2_rx);

	// required link function
	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
	//make uart clean of trash
//	__HAL_UART_FLUSH_DRREGISTER(&huart2);
	HAL_StatusTypeDef status;
	//Register Callback (invoke my function TransferComplete) when transfer to memory M1 is completed
	HAL_DMA_RegisterCallback(&hdma_usart2_rx, HAL_DMA_XFER_M1CPLT_CB_ID,TransferComplete);
	//Set M1 target memory base adress
	DMA1_Stream5->M1AR = (uint32_t)FrameBuffer2.byte;
	//start DMA receiving to M0
	HAL_UART_Receive_DMA(&huart2, &FrameBuffer.byte, 76);
	//Turn off preconfigured DMA for HAL workaround
	DMA1_Stream5->CR &= ~DMA_SxCR_EN;
	//Setup Double buffer for UART_RX_DMA
	DMA1_Stream5->CR |= DMA_SxCR_DBM;
	//Run DMA
	DMA1_Stream5->CR |= DMA_SxCR_EN;
	//The workaround is complete!

	/*
	 * TEST OF DMA MEM TO MEM CONFIG FOR CRC CALCULATION
	 */
	//CALLBACK REGISTRATION
	osDelay(500); // this delay is necessary to set those callback, otherwise they have chance to not me eqecuted (HAL_BUSY or some shit)
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream0, HAL_DMA_XFER_CPLT_CB_ID, DMA_CRC_COMPLETE_Callback);
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream1, HAL_DMA_XFER_CPLT_CB_ID, HOST_COMMAND_COMPLETE_Callback);
	/*
	 * DMA FRAME TO SETTINGS AND SET VALUE struct/union
	 */
	TIM9->CR1 |= TIM_CR1_CEN;

	/*
	 * Test of CRC module
	 */
//	uint32_t dummy[19];
//	int i=0;
//	for(i=0; i<19; i++)
//	{
//		dummy[i] = 0x61626364;
//	}
//	CRC->CR |= CRC_CR_RESET;
//	for (i = 0; i<19; i++)
//	{
//		//this writes to CRC->DR (via cast to uint32_t the CRC_BASE address)
//		//*(__IO DATATYPE*)(CRC_BASE) = (uint32_t)dummy32;
//		CRC->DR = dummy[i];
//	}
//	uint32_t result = CRC->DR;
//	CRC->CR |= CRC_CR_RESET;
//	serial_buffer.serial_buf_char[0] = 'a'; //0x61
//	serial_buffer.serial_buf_char[1] = 'b'; //0x62
//	serial_buffer.serial_buf_char[2] = 'c'; //0x63
//	serial_buffer.serial_buf_char[3] = 'd'; //0x64
//
//	uint32_t dummy = serial_buffer.serial_buf_4char[0]; //??? 0x64636261 wtf...
//	uint32_t dummy_revd = __REV(dummy);
//
//	serial_buffer.serial_buf_char[4] = 'e';
//	serial_buffer.serial_buf_char[5] = 'f';
//	serial_buffer.serial_buf_char[6] = 'g';
//	serial_buffer.serial_buf_char[7] = 'h';
//
//	CRC->CR |= CRC_CR_RESET;
//
//	uint32_t result = CRC_CalcBlockCRCxxbits();
//	uint32_t blah = ~result;
//	uint32_t blah2 = blah ^ 0xFFFFFFFF;
	// CRC TEST PASSED!

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM9)
	{
		uint32_t dummy = 1;
	}

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
