/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PRG_BTN_Pin GPIO_PIN_0
#define PRG_BTN_GPIO_Port GPIOA
#define SYS_BTN_Pin GPIO_PIN_1
#define SYS_BTN_GPIO_Port GPIOA
#define SET_Pin GPIO_PIN_4
#define SET_GPIO_Port GPIOA
#define B_R_Pin GPIO_PIN_6
#define B_R_GPIO_Port GPIOA
#define A_R_Pin GPIO_PIN_7
#define A_R_GPIO_Port GPIOA
#define ADC_R_Pin GPIO_PIN_5
#define ADC_R_GPIO_Port GPIOC
#define ADC_L_Pin GPIO_PIN_1
#define ADC_L_GPIO_Port GPIOB
#define PWMR_Pin GPIO_PIN_9
#define PWMR_GPIO_Port GPIOC
#define D2R_Pin GPIO_PIN_8
#define D2R_GPIO_Port GPIOA
#define D1R_Pin GPIO_PIN_9
#define D1R_GPIO_Port GPIOA
#define PWML_Pin GPIO_PIN_10
#define PWML_GPIO_Port GPIOA
#define D2L_Pin GPIO_PIN_11
#define D2L_GPIO_Port GPIOA
#define D1L_Pin GPIO_PIN_12
#define D1L_GPIO_Port GPIOA
#define B_L_Pin GPIO_PIN_6
#define B_L_GPIO_Port GPIOB
#define A_L_Pin GPIO_PIN_7
#define A_L_GPIO_Port GPIOB
#define TO_SERVO_Pin GPIO_PIN_9
#define TO_SERVO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
