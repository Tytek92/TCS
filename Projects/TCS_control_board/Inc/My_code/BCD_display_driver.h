/*
 * BCD_display_driver.h
 *
 *  Created on: 17 gru 2016
 *      Author: Tymoteusz
 */

#ifndef MY_CODE_BCD_DISPLAY_DRIVER_H_
#define MY_CODE_BCD_DISPLAY_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


#define SEG_A_BIT (1<<0)
#define SEG_B_BIT (1<<1)
#define SEG_C_BIT (1<<2)
#define SEG_D_BIT (1<<3)
#define SEG_E_BIT (1<<4)
#define SEG_F_BIT (1<<5)
#define SEG_G_BIT (1<<6)

#define PERIPHERAL_BASE 0x40000000
#define BITBAND_PERIPHERAL_BASE 0x42000000
#define BITBAND_PERIPHERAL(a,b) (BITBAND_PERIPHERAL_BASE + (a-PERIPHERAL_BASE)*0x20+(b*4))
//port D output register address
//#define GPIOD_ODR 0x4001140C
//create a pointer to bit 5 in bit band location
#define SEG_A_REG (*((volatile unsigned long *)(BITBAND_PERIPHERAL(((uint32_t)&(GPIOB->ODR)),14))))
#define SEG_B_REG (*((volatile unsigned long *)(BITBAND_PERIPHERAL(((uint32_t)&(GPIOB->ODR)),15))))
#define SEG_C_REG (*((volatile unsigned long *)(BITBAND_PERIPHERAL(((uint32_t)&(GPIOB->ODR)),12))))
#define SEG_D_REG (*((volatile unsigned long *)(BITBAND_PERIPHERAL(((uint32_t)&(GPIOB->ODR)),0))))
#define SEG_E_REG (*((volatile unsigned long *)(BITBAND_PERIPHERAL(((uint32_t)&(GPIOB->ODR)),2))))
#define SEG_F_REG (*((volatile unsigned long *)(BITBAND_PERIPHERAL(((uint32_t)&(GPIOC->ODR)),7))))
#define SEG_G_REG (*((volatile unsigned long *)(BITBAND_PERIPHERAL(((uint32_t)&(GPIOC->ODR)),6))))


//uint32_t test = (uint32_t)PORTBBIT14;


void Display_char(uint8_t char_number);

void Disp_0_();
void Disp_1_();
void Disp_2_();
void Disp_3_();
void Disp_4_();
void Disp_5_();
void Disp_6_();
void Disp_7_();
void Disp_8_();
void Disp_9_();
void Disp_A_();
void Disp_C_();
void Disp_U_();
void Disp_E_();
void Disp_c_();
void Disp_F_();
void Disp_r_();

#endif /* MY_CODE_BCD_DISPLAY_DRIVER_H_ */
