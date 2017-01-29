/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "usart.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId MainTaskHandle;

/* USER CODE BEGIN Variables */
extern uint8_t Run_communication;
extern uint8_t comm_running;
extern uint8_t timeout_counter;
extern volatile uint8_t timeout_event;

extern volatile union FrameBuffer FrameBuffer;

union FrameBuffer{
	char byte[76];
	uint32_t word[19];
};

extern volatile uint16_t ADC_read[];

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartMainTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of MainTask */
  osThreadDef(MainTask, StartMainTask, osPriorityNormal, 0, 128);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartMainTask function */
void StartMainTask(void const * argument)
{

	/* USER CODE BEGIN StartMainTask */
	uint8_t button_loop_counter = 0;
	/* Infinite loop */
	for(;;)
	{
		if(0 == Run_communication)// if transmission is launched, ignore the button
		{
			if(0 == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))// wait for button press to launch transmission
			{
				button_loop_counter++;
				if(button_loop_counter >= 170)
				{
					Run_communication = 1;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET);
					timeout_counter=0;
				}
			}
			else
			{
				button_loop_counter = 0;
			}
		}
		else
		{
			if(0 == comm_running) //if there is no ongoing transmission or timeout was reached, send next frame
			{
				FrameBuffer.byte[0] = ADC_read[0];
				FrameBuffer.byte[1] = ADC_read[1];
				FrameBuffer.byte[2] = ADC_read[2];
				FrameBuffer.byte[3] = ADC_read[3];
				  CRC->CR |= CRC_CR_RESET;
				  for(uint8_t i=0; i<18; i++)
				  {
					  CRC->DR = FrameBuffer.word[i];
				  }
				  uint32_t crc = CRC->DR;
				  FrameBuffer.word[18]=crc;
				HAL_UART_Transmit_IT(&huart2,FrameBuffer.word,76); //transmit frame with CRC
				Timeout_start(); //start transmission timeout
				comm_running = 1; //indicate that transmission routine is running
				osDelay(40);
			}
			else
			{
				//comm is running, wait for ACK or timeout
				if(1 == timeout_event) //timeout event occured
				{
					if(timeout_counter >= 10) //too many timeouts, wait for manual start. Flash an LED
					{
						comm_running = 0;
						Run_communication = 0; //enable launching transmission using user button
						timeout_event = 0; //reset the event flag
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);

					}
					else
					{
						//this is simple timeout with no ACK or ERR
						//send another frame
						timeout_event = 0; //reset the event flag
						comm_running = 0;
					}
				}
			}
		}
		osDelay(1);
	}
	/* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
