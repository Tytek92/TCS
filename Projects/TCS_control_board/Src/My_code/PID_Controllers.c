/*
 * PID_controllers.c
 *
 *  Created on: Jul 30, 2017
 *      Author: tymoteusz
 */

#include "My_code/PID_Controllers.h"

uint16_t PID_loop_right_wh()
{
	uint8_t iMax = 100;			// Used to prevent integral wind-up
	int8_t iMin = -100;		// Used to prevent integral wind-up
	int Err_Value;			// Holds the calculated Error value
	int P_Term;			// Holds the calculated Proportional value
	int I_Term;			// Holds the calculated Integral value
	int D_Term;			// Holds the calculated Differential value
	int PWM_Duty;			// Holds the new PWM value

	Err_Value = (System_State.TargetAngularVelocityRearRightWh - System_State.AngularVelocityRearRightWh);

	// This calculates Proportional value, Kp is multiplied with Err_Value and the result is assigned to P_Term
	P_Term = Kp1 * Err_Value;

	// Prepare Integral value, add the current error value to the integral value and assign the total to i_Temp
	i_Temp1 += Err_Value;

	// Prevents integral wind-up, limits i_Temp from getting too positive or negative
	if (i_Temp1 > iMax)
	{i_Temp1 = iMax;}
	else if (i_Temp1 < iMin)
	{i_Temp1 = iMin;}

	// Calculates the Integral value, Ki is multiplied with i_Temp and the result is assigned to I_Term
	I_Term = Ki1 * i_Temp1;

	// Calculates Differential value, Kd is multiplied with (d_Temp minus new_ADC_value) and the result is assigned to D_Term
	// The new_ADC_value will become the old ADC value on the next function call, this is assigned to d_Temp so it can be used
	D_Term = Kd1 * (d_Temp1 - Err_Value);
	d_Temp1 = Err_Value;

	/****** Now we have the P_Term, I_Term and D_Term *****/
	PWM_Duty = PWM_Temp1 - (P_Term + I_Term + D_Term);

	// PWM overflow prevention
	if (PWM_Duty > 300)
	{PWM_Duty = 300;}
	else if (PWM_Duty < 30)
	{PWM_Duty = 30;}

	// Adjusts the PWM duty cycle
	adjust_PWM_right_wh(PWM_Duty);
	// Assigns the current PWM duty cycle value to PWM_Temp
	PWM_Temp1 = PWM_Duty;
	return 0;
}

uint16_t PID_loop_left_wh()
{
	uint8_t iMax = 100;			// Used to prevent integral wind-up
	int8_t iMin = -100;		// Used to prevent integral wind-up
	int Err_Value;			// Holds the calculated Error value
	int P_Term;			// Holds the calculated Proportional value
	int I_Term;			// Holds the calculated Integral value
	int D_Term;			// Holds the calculated Differential value
	int PWM_Duty;			// Holds the new PWM value

	Err_Value = (System_State.TargetAngularVelocityRearLeftWh - System_State.AngularVelocityRearLeftWh);

	// This calculates Proportional value, Kp is multiplied with Err_Value and the result is assigned to P_Term
	P_Term = Kp2 * Err_Value;

	// Prepare Integral value, add the current error value to the integral value and assign the total to i_Temp
	i_Temp2 += Err_Value;

	// Prevents integral wind-up, limits i_Temp from getting too positive or negative
	if (i_Temp2 > iMax)
	{i_Temp2 = iMax;}
	else if (i_Temp2 < iMin)
	{i_Temp2 = iMin;}

	// Calculates the Integral value, Ki is multiplied with i_Temp and the result is assigned to I_Term
	I_Term = Ki2 * i_Temp2;

	// Calculates Differential value, Kd is multiplied with (d_Temp minus new_ADC_value) and the result is assigned to D_Term
	// The new_ADC_value will become the old ADC value on the next function call, this is assigned to d_Temp so it can be used
	D_Term = Kd2 * (d_Temp2 - Err_Value);
	d_Temp2 = Err_Value;

	/****** Now we have the P_Term, I_Term and D_Term *****/
	PWM_Duty = PWM_Temp2 - (P_Term + I_Term + D_Term);

	// PWM overflow prevention
	if (PWM_Duty > 300)
	{PWM_Duty = 300;}
	else if (PWM_Duty < 30)
	{PWM_Duty = 30;}

	// Adjusts the PWM duty cycle
	adjust_PWM_left_wh(PWM_Duty);
	// Assigns the current PWM duty cycle value to PWM_Temp
	PWM_Temp1 = PWM_Duty;
	return 0;
}
/*
 * smooth PWM change via timer 5 ch1
 */
void adjust_PWM_right_wh(int PWM_Duty)
{
	RightWhChangedDuty=1;

}
/*
 * smooth PWM change via timer 5 ch1
 */
void adjust_PWM_left_wh(int PWM_Duty)
{
	LeftWhChangedDuty=1;
}


