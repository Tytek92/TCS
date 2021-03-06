/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "My_code/crc.h"
#include "My_code/timeout.h"
#include "Basic_global_structures/global_structures.h"
extern uint8_t serial_buffer_counter;

extern volatile uint32_t FrameBuffer[];
extern volatile uint8_t FrameBufferIndicator;
extern volatile uint8_t TimeoutEventFlag;

extern volatile uint8_t TimeoutMode;
extern volatile uint8_t TimeoutMessage;

extern char errorframe[];
extern char errorframe2[];

char a = '0';
char b = '0';
char c = '0';
char d = '0';
char e = '0';
char f = '0';


uint8_t counter=0;


/*
 * TIM5 PWM duty changing, new value memory
 */

int OldDutyRightWh = 0;
int PwmDutyStepRightWh = 0;
int OldDutyLeftWh = 0;
int PwmDutyStepLeftWh = 0;

uint8_t RightWhChangedDuty = 0;
uint8_t LeftWhChangedDuty = 0;

uint32_t rev_counter_rwh = 0;

/*
 * Veriables for storing revolution difference
 */
volatile int R_wh_CNT_prev = 0;
volatile int L_wh_CNT_prev = 0;

volatile int R_wh_CNT = 0;
volatile int L_wh_CNT = 0;

volatile int R_wh_CNT_diff = 0;
volatile int L_wh_CNT_diff = 0;


volatile uint8_t R_wh_CNT_overflow = 0;
volatile uint8_t L_wh_CNT_overflow = 0;

volatile uint16_t R_wh_memory[10] = 0;
volatile uint16_t L_wh_memory[10] = 0;
volatile uint8_t Filter_memory_steps = 0;


//TIM5 (200us) prescaler to 2ms
uint8_t TIM5_presc_velocity_measurment = 0;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim10;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */


  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

	if(serial_buffer_counter==SERIAL_BUF_SIZE_Uint8t)
	{

		// DEPLOY ANOTHER DMA TO TRASFER THIS ARRAY to union serial_buffer
		//HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,(uint32_t)Rx_whole_frame_buffer,(uint32_t)serial_buffer.serial_buf_4char,2U);
		//After this DMA request is complete, whole frame is transfered to the global union and can be processed
		//serial_buffer_counter=0;
	}
  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
* @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
*/
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
  if(TIM9->SR & TIM_SR_UIF_Msk)
  {
	  uint32_t dummy = 111;
	  TIM9->SR &= ~TIM_SR_UIF;//reset interrupt flag
	  if(RESP_MODE == TimeoutMode)
	  {
		  if(ACK == TimeoutMessage)
		  {
			  //Send ACK once again
			  HAL_UART_Transmit_IT(&huart2,errorframe,8);//SEND ACK
			  Timeout_start(RESP_MODE, ACK);//Start the timeout timer with response mode
		  }
		  else if(ERR == TimeoutMessage)//Means we had to send ERR
		  {
			  //Send ERR once again
			  HAL_UART_Transmit_IT(&huart2,errorframe2,8);
			  Timeout_start(RESP_MODE, ERR);//Start the timeout timer with response mode
		  }
	  }
	  else if(RECV_MODE == TimeoutMode)//RECV MODE
	  {
		  if(ERR == TimeoutMessage)
		  {
			  //Send ERR
			  HAL_UART_Transmit_IT(&huart2,errorframe2,8);
			  USART2->CR1 |= USART_CR1_RXNEIE; //enable incoming transmission interrupt
			  if(76 != DMA1_Stream5 -> NDTR)
			  {
				  //something went wrong! Clean up mess
				  DMA_Reset();
			  }
			  Timeout_start(RESP_MODE, ERR);//Start the timeout timer with response mode
		  }
		  else if(NONE == TimeoutMessage)
		  {

		  }
	  }

  }

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  /*
   * TODO: Counting function of angular velocity based on TIM5 and TIM3 CNT values
   * USE CCxIF bit in TIMx_SR
   * CCxIE in TIMx_DIER
   *
   * Timer setup to create interrupt every 100us
   *
   * Every 10 samples run filter and average velocity.
   */

  Filter_memory_steps++;

  if(TIM1->SR & TIM_SR_CC3IF_Msk)//interrupt came from Channel 3 of TIM1 - timebase for angular velocity
  {
	  R_wh_CNT = TIM3->CNT;
	  L_wh_CNT = TIM4->CNT;
	  /*
	   * Left wheel
	   */
	  if(0 == L_wh_CNT_overflow)//No overflow
	  {
		  if(L_wh_CNT < L_wh_CNT_prev)//motion was backwards
			  L_wh_CNT_diff = L_wh_CNT_prev - L_wh_CNT;
		  else//motion was forward
			  L_wh_CNT_diff = L_wh_CNT - L_wh_CNT_prev;
	  }
	  else//Counter overflow occured
	  {
		  if(L_wh_CNT < L_wh_CNT_prev)//overflow going forward
			  L_wh_CNT_diff = 65535 - L_wh_CNT_prev + L_wh_CNT;
		  else//overflow going backward
			  L_wh_CNT_diff = 65535 - L_wh_CNT + L_wh_CNT_prev;
		  L_wh_CNT_overflow = 0;
	  }
	  /*
	   * Right wheel
	   */
	  if(0 == R_wh_CNT_overflow)//No overflow
	  {
		  if(R_wh_CNT < R_wh_CNT_prev)//motion was backwards
			  R_wh_CNT_diff = R_wh_CNT_prev - R_wh_CNT;
		  else//motion was forward
			  R_wh_CNT_diff = R_wh_CNT - R_wh_CNT_prev;
	  }
	  else//Counter overflow occured
	  {
		  if(R_wh_CNT < R_wh_CNT_prev)//overflow going forward
			  R_wh_CNT_diff = 65535 - R_wh_CNT_prev + R_wh_CNT;
		  else//overflow going backward
			  R_wh_CNT_diff = 65535 - R_wh_CNT + R_wh_CNT_prev;
		  R_wh_CNT_overflow = 0;//bbb
	  }

  }
  /*
   * Filter section, first implementation - average value, in future implement median filter
   * runs every 1ms
   */
  R_wh_memory[Filter_memory_steps] = R_wh_CNT_diff;
  L_wh_memory[Filter_memory_steps] = L_wh_CNT_diff;
  if(10 <= Filter_memory_steps)
  {
	  Filter_memory_steps = 0;

  }
  /*
   * TODO Calculate velocity
   */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	if(TIM3->SR & TIM_SR_UIF_Msk)
	{
		R_wh_CNT_overflow = 1;
	}

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	if(TIM4->SR & TIM_SR_UIF_Msk)
	{
		L_wh_CNT_overflow = 1;
	}


  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	//disable receive interrupt (not needed during whole transfer)
	if(USART2->CR1 & USART_CR1_RXNEIE_Msk)
	{
		USART2->CR1 &= ~USART_CR1_RXNEIE;
		//Stop pending timeout timer if timer is running
		if(TIM9->CR1 & TIM_CR1_CEN_Msk)
		{
			Timeout_abort();
		}
		//Start timeout timer
		Timeout_start(RECV_MODE, ERR);
		//TIM9->SR &= ~TIM_SR_UIF;//reset interrupt flag
	}
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	/*
	 * 200us time interval
	 */

	//Part where angular velocity is measured.
	if(5 <= TIM5_presc_velocity_measurment)
	{
		//perform measurment on both wheels
		TIM5_presc_velocity_measurment=0;
	}
	else //start new prescaler period
		TIM5_presc_velocity_measurment++;
	//end of angular velocity measurment

	//TODO smooth PWM change for motors
	//target pwm right: System_State.TargetAngularVelocityRearRightWh
	//current pwm right: TIM1->CCR1
	//HAL_GPIO_TogglePin(Dummy_output_GPIO_Port, Dummy_output_Pin);
	Dummy_output_GPIO_Port->ODR ^= Dummy_output_Pin;

	static uint8_t step_down_rightwh = 0;

	if(1 == RightWhChangedDuty)
	{
		//if target duty cycle was not changed then execute smooth pwm change
		if(OldDutyRightWh==System_State.TargetAngularVelocityRearRightWh)
		{
			uint32_t ppp = TIM1->CCR1;
			if(step_down_rightwh == 0)//we are stepping up - OldDutyRightWh is > TIM1->CCR1
			{
				if((OldDutyRightWh-(TIM1->CCR1))>PwmDutyStepRightWh)//if difference is big enough do steps of incrementation
				{
					TIM1->CCR1 = (TIM1->CCR1)+PwmDutyStepRightWh;
				}
				else if((TIM1->CCR1 != OldDutyRightWh))//difference is less than step size, set register to proper value
				{
					TIM1->CCR1 = OldDutyRightWh;
					RightWhChangedDuty = 1;//should be 0
					//System_State.TargetAngularVelocityRearRightWh=0;
				}
			}
			else//we are stepping up - OldDutyRightWh is < TIM1->CCR1
			{
				if(((TIM1->CCR1)-OldDutyRightWh)>PwmDutyStepRightWh)//if difference is big enough do steps of incrementation
				{
					TIM1->CCR1 = (TIM1->CCR1)-PwmDutyStepRightWh;
				}
				else if((TIM1->CCR1 != OldDutyRightWh))//difference is less than step size, set register to proper value
				{
					TIM1->CCR1 = OldDutyRightWh;
					RightWhChangedDuty = 1;//should be 0
					//System_State.TargetAngularVelocityRearRightWh=0;
				}
			}
			//if(TIM1->CCR1 == 12800)
				//System_State.TargetAngularVelocityRearRightWh = 0;
			//if(TIM1->CCR1 == 0)
				//System_State.TargetAngularVelocityRearRightWh = 12800;
		}
		else//if it was changed calculate new step
		{
			OldDutyRightWh=System_State.TargetAngularVelocityRearRightWh;
			int dummy = (OldDutyRightWh-TIM1->CCR1)*10000;
			PwmDutyStepRightWh = dummy/12800;
			if(PwmDutyStepRightWh < 0)
			{
				step_down_rightwh = 1;
				PwmDutyStepRightWh *= -1;
			}
			else
			{
				step_down_rightwh = 0;
			}

			//PwmDutyStepRightWh = ((TIM1->CCR1-OldDutyRightWh)*100)/65535;
		}
	}
	if(1 == LeftWhChangedDuty)
	{
		//if target duty cycle was not changed then execute smooth pwm change
		if(OldDutyLeftWh==System_State.TargetAngularVelocityRearLeftWh)
		{
			if(((TIM1->CCR3)-OldDutyLeftWh)>100)//if difference is big enough do steps of incrementation
			{
				TIM1->CCR3 = (TIM1->CCR3)+PwmDutyStepLeftWh;
			}
			else if((TIM1->CCR3 != OldDutyLeftWh))//difference is less than 100, set register to proper value
			{
				TIM1->CCR3 = OldDutyLeftWh;
				LeftWhChangedDuty = 0;
			}
		}
		else
		{
			OldDutyLeftWh=System_State.TargetAngularVelocityRearLeftWh;
			PwmDutyStepLeftWh = ((TIM1->CCR3-OldDutyLeftWh)*100)/65535;
		}

	}



  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_memtomem_dma2_stream0);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/*
 * This function is called when FrameBuffer[20] is full
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//uint32_t zm= FrameBuffer[0];
	/*
	 * Here instruct DMA to calculate CRC, check it with what was received.
	 * Do that by setting a mutex/semaphore to let task execute. Task will wait for second DMA callback [TODO] to proceed
	 * If CRC matches proceed with filling proper structure [#########] fields with data from the frame
	 * does that in turns with void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) located in stm32f4xx_it.c
	 */
	//check if this function executes quickly enough, otherwise try doing this using registers!
	//HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,(uint32_t)Rx_whole_frame_buffer,(uint32_t)serial_buffer.serial_buf_4char,2U);
	FrameBufferIndicator = 1;
	//reset CRC so it is 0xFFFFFFFF
	CRC->CR |= CRC_CR_RESET;
	//Start transfer to calculate CRC
	HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0,(uint32_t)FrameBuffer,(uint32_t)&CRC->DR,19);

}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	uint32_t dummy_dummy=32;
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
