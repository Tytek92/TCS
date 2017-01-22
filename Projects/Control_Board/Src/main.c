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
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
union FrameBuffer{
	char byte[76];
	uint32_t word[19];
};


volatile union FrameBuffer FrameBuffer;
volatile union InputFrame InputFrame;

volatile char Input_Buffer[40] = {0};
volatile uint8_t Input_Buffer_counter=0;

volatile uint8_t TransmissionError = 0;

extern ADC_HandleTypeDef hadc;

volatile uint16_t ADC_read[4] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Timeout_TIM16_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; // turn on clock for TIM16
	TIM16->CR1 |= TIM_CR1_ARPE; //auto reload preload
	//when ARR value is changed, then it is buffered until next overflow/compara match ect.
	//to prevent for exampe missing ARR value and couting up to max CNT value
	TIM16->CR1 |= TIM_CR1_OPM; //one pulse mode. Timer stops after update event.
	TIM16->DIER |= TIM_DIER_UIE; // update interrupt enable
	TIM16->PSC = 10;
	TIM16->ARR = 63999;
    HAL_NVIC_SetPriority(TIM16_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM16_IRQn);
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
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();

  /* USER CODE BEGIN 2 */
  Timeout_TIM16_Init();
  TIM16->CR1 |= TIM_CR1_CEN;
  //configure DMA for USART2 Rx
	hdma_usart2_rx.Instance = DMA1_Channel5;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW; // in future check if needs to be changed
	HAL_DMA_Init(&hdma_usart2_rx);

	// required link function
	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
	HAL_UART_Receive_DMA(&huart2, &InputFrame.byte, 8);

  //launch UART receive
  //HAL_UART_Receive_IT(&huart2,InputFrame.byte,8);


  char znak = 33;
  int i=0;

  for(i=0; i<72; i++)
  {
	  FrameBuffer.byte[i]=znak;
	  znak++;
  }

  CRC->CR |= CRC_CR_RESET;
  for(i=0; i<18; i++)
  {
	  CRC->DR = FrameBuffer.word[i];
  }


  uint32_t crc = CRC->DR;
  FrameBuffer.word[18]=crc;

  //HAL_UART_Transmit(&huart2,FrameBuffer.word,8,5000);
  TIM16->CR1 |= TIM_CR1_CEN;
  uint8_t counter = 0;
  uint8_t message_counter = 0;

 // HAL_ADC_Start_IT
  HAL_ADC_Start_DMA(&hadc,(uint32_t *)ADC_read,4);
  //HAL_ADC_Start(&hadc);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(200);
	  uint16_t dummy[4];
	  dummy[0]=ADC_read[0];
	  dummy[1]=ADC_read[1];
	  dummy[2]=ADC_read[2];
	  dummy[3]=ADC_read[3];
	  HAL_Delay(400);
	  HAL_UART_Transmit(&huart2,dummy,8,5000);
	 // char end[2] = {'\r','\n'};
	  //HAL_UART_Transmit(&huart2,end,2,5000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//	  if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
//	  {
//		  counter++;
//		  if(counter > 12)
//		  {
//			  if(1 == message_counter) // second message will be faulty
//			  {
//				  HAL_UART_Transmit(&huart2,FrameBuffer.word,75,5000);
//			  }
//			  else
//			  {
//				  HAL_UART_Transmit(&huart2,FrameBuffer.word,76,5000);
//				  counter=0;
//				  if(4 == message_counter)
//				  {
//					  message_counter = 0;
//				  }
//			  }
//			  message_counter++;
//			  //now play dead
//			  HAL_Delay(230);
//		  }
//	  }
//	  else
//	  {
//		  counter=0;
//	  }
//	  if(10 == message_counter)
//	  {
//		  HAL_UART_Transmit(&huart2,FrameBuffer.word,72,5000);
//	  }
//	  else
//	  {
//		  if(1 == TransmissionError)
//		  {
//			  HAL_Delay(250);
//			  TransmissionError = 0;
//		  }
//		  HAL_UART_Transmit(&huart2,FrameBuffer.word,76,5000);
//		  //counter=0;
//		  if(20 == message_counter)
//		  {
//			  message_counter = 0;
//		  }
//	  }
//	  message_counter++;
//	  HAL_Delay(10);


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

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
  while(1) 
  {
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
