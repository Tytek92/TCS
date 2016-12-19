/*
 * BCD_display_driver.c
 *
 *  Created on: 17 gru 2016
 *      Author: Tymoteusz
 */
#include "Basic_global_structures/global_structures.h"
#include "My_code/BCD_display_driver.h"


/*
 * This is font for 7 segment display with 17 characters
 */
const uint8_t font[17] = {SEG_A_BIT|SEG_B_BIT|SEG_C_BIT|SEG_D_BIT|SEG_E_BIT|SEG_F_BIT,  //0
		SEG_B_BIT|SEG_C_BIT, 															//1
		SEG_A_BIT|SEG_B_BIT|SEG_G_BIT|SEG_E_BIT|SEG_D_BIT,								//2
		SEG_A_BIT|SEG_B_BIT|SEG_C_BIT|SEG_D_BIT|SEG_G_BIT,								//3
		SEG_B_BIT|SEG_C_BIT|SEG_F_BIT|SEG_G_BIT,										//4
		SEG_A_BIT|SEG_F_BIT|SEG_G_BIT|SEG_C_BIT|SEG_D_BIT,								//5
		SEG_A_BIT|SEG_F_BIT|SEG_C_BIT|SEG_D_BIT|SEG_G_BIT|SEG_E_BIT,					//6
		SEG_A_BIT|SEG_B_BIT|SEG_C_BIT,													//7
		SEG_A_BIT|SEG_B_BIT|SEG_C_BIT|SEG_D_BIT|SEG_E_BIT|SEG_F_BIT|SEG_G_BIT,			//8
		SEG_A_BIT|SEG_B_BIT|SEG_C_BIT|SEG_D_BIT|SEG_F_BIT|SEG_G_BIT,					//9
		SEG_A_BIT|SEG_B_BIT|SEG_C_BIT|SEG_E_BIT|SEG_F_BIT|SEG_G_BIT,					//A
		SEG_A_BIT|SEG_D_BIT|SEG_E_BIT|SEG_F_BIT,										//C
		SEG_B_BIT|SEG_C_BIT|SEG_D_BIT|SEG_E_BIT|SEG_F_BIT,								//U
		SEG_A_BIT|SEG_D_BIT|SEG_E_BIT|SEG_F_BIT|SEG_G_BIT,								//E
		SEG_D_BIT|SEG_E_BIT|SEG_G_BIT,													//c
		SEG_A_BIT|SEG_E_BIT|SEG_F_BIT|SEG_G_BIT,										//F
		SEG_E_BIT|SEG_G_BIT																//r
};

void Display_char(uint8_t char_number)
{
	uint8_t code = font[char_number];
	/*
	 * those shifts are required for proper bitband (SEG_x_REG) operation
	 */
	SEG_A_REG = code & SEG_A_BIT;
	SEG_B_REG = (code & SEG_B_BIT)>>1;
	SEG_C_REG = (code & SEG_C_BIT)>>2;
	SEG_D_REG = (code & SEG_D_BIT)>>3;
	SEG_E_REG = (code & SEG_E_BIT)>>4;
	SEG_F_REG = (code & SEG_F_BIT)>>5;
	SEG_G_REG = (code & SEG_G_BIT)>>6;
}

/*
 * BSRR bits 0:15 - reset place
 * BSRR bits 16:31 - set place
 *
 * Common cathode - low level (reset) switches segment ON
 */

void Disp_0_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
}
void Disp_1_()
{
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
}
void Disp_2_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_3_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_4_()
{
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_5_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
}
void Disp_6_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_7_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
}
void Disp_8_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_9_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_A_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;

}
void Disp_C_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
}
void Disp_U_()
{
	B_7SEG_GPIO_Port->BSRR = (uint32_t)B_7SEG_Pin<<16;
	C_7SEG_GPIO_Port->BSRR = (uint32_t)C_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
}
void Disp_E_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_c_()
{
	D_7SEG_GPIO_Port->BSRR = (uint32_t)D_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_F_()
{
	A_7SEG_GPIO_Port->BSRR = (uint32_t)A_7SEG_Pin<<16;
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	F_7SEG_GPIO_Port->BSRR = (uint32_t)F_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
void Disp_r_()
{
	E_7SEG_GPIO_Port->BSRR = (uint32_t)E_7SEG_Pin<<16;
	G_7SEG_GPIO_Port->BSRR = (uint32_t)G_7SEG_Pin<<16;
}
