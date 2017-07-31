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
#include "Basic_global_structures/global_structures.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId Display_TaskHandle;
osThreadId Dummy_display_uHandle;
osThreadId PIDloop_TaskHandle;
osMutexId Disp_BCD_StateHandle;

/* USER CODE BEGIN Variables */
extern union FrameBuffer FrameBuffer;
extern union FrameBuffer FrameBuffer2;
extern volatile union TCS_input_data TCS_input_data;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDisplay_Task(void const * argument);
void StartDummy_display_update(void const * argument);
void Start_PIDloop_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of Disp_BCD_State */
  osMutexDef(Disp_BCD_State);
  Disp_BCD_StateHandle = osMutexCreate(osMutex(Disp_BCD_State));

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
  /* definition and creation of Display_Task */
  osThreadDef(Display_Task, StartDisplay_Task, osPriorityNormal, 0, 128);
  Display_TaskHandle = osThreadCreate(osThread(Display_Task), NULL);

  /* definition and creation of Dummy_display_u */
  osThreadDef(Dummy_display_u, StartDummy_display_update, osPriorityIdle, 0, 128);
  Dummy_display_uHandle = osThreadCreate(osThread(Dummy_display_u), NULL);

  /* definition and creation of PIDloop_Task */
  osThreadDef(PIDloop_Task, Start_PIDloop_Task, osPriorityAboveNormal, 0, 128);
  PIDloop_TaskHandle = osThreadCreate(osThread(PIDloop_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDisplay_Task function */
void StartDisplay_Task(void const * argument)
{

  /* USER CODE BEGIN StartDisplay_Task */
  /* Infinite loop */
  for(;;)
  {
	  uint32_t val = FrameBuffer.word[0];
	  uint32_t val2 = FrameBuffer2.word[0];

    osDelay(1);
  }
  /* USER CODE END StartDisplay_Task */
}

/* StartDummy_display_update function */
void StartDummy_display_update(void const * argument)
{
  /* USER CODE BEGIN StartDummy_display_update */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDummy_display_update */
}

/* Start_PIDloop_Task function */
void Start_PIDloop_Task(void const * argument)
{
  /* USER CODE BEGIN Start_PIDloop_Task */
  /* Infinite loop */
  for(;;)
  {
	  PID_loop_right_wh();
	  PID_loop_left_wh();
	  osDelay(10);//100Hz
  }
  /* USER CODE END Start_PIDloop_Task */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
