/*
 * BCD_display_driver.c
 *
 *  Created on: 17 gru 2016
 *      Author: Tymoteusz
 */
#include "Basic_global_structures/global_structures.h"
#include "My_code/BCD_display_driver.h"

void Display_char()
{
	uint8_t character = Disp_BCD_Data.displayed_character;

	switch (character)
	{
	case 0:
		Disp_0_();
		// BLAH BLAH BLAH COMMIT TEST
		//aha
		break;
	case 1:
		Disp_1_();
		break;
	case 2:
		Disp_2_();
		break;
	case 3:
		Disp_3_();
		break;
	case 4:
		Disp_4_();
		break;
	case 5:
		Disp_5_();
		break;
	case 6:
		Disp_6_();
		break;
	case 7:
		Disp_7_();
		break;
	case 8:
		Disp_8_();
		break;
	case 9:
		Disp_9_();
		break;
	case 10:
		Disp_A_();
		break;
	case 11:
		Disp_C_();
		break;
	case 12:
		Disp_E_();
		break;
	case 13:
		Disp_F_();
		break;
	case 14:
		Disp_U_();
		break;
	case 15:
		Disp_c_();
		break;
	case 16:
		Disp_r_();
		break;
	default:
		break;
	}
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
