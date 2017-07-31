/*
 * PID_Controllers.h
 *
 *  Created on: Jul 30, 2017
 *      Author: tymoteusz
 */

#ifndef MY_CODE_PID_CONTROLLERS_H_
#define MY_CODE_PID_CONTROLLERS_H_

/*
 * includes
 */

#include "Basic_global_structures/global_structures.h"

/*
 * PID controller settings
 * Right rear wheel
 */
const float Kp1 = 0.01;		// The value for Proportional gain
const float Ki1 = 0.01;		// The value for Integral gain
const float Kd1 = 0.0001;	// The value for Differential gain
/*
/	Global Variables for PID
*/
int d_Temp1 = 0;				// This stores the old ADC value
int i_Temp1 = 0;				// This stores the accumulated Integral value
int PWM_Temp1 = 166;			// Given an initial value, after that just stores the old value for calculation
/*
* PID controller settings
* Right rear wheel
*/
const float Kp2 = 0.01;		// The value for Proportional gain
const float Ki2 = 0.01;		// The value for Integral gain
const float Kd2 = 0.0001;	// The value for Differential gain
/*
/	Global Variables for PID
*/
int d_Temp2 = 0;				// This stores the old ADC value
int i_Temp2 = 0;				// This stores the accumulated Integral value
int PWM_Temp2 = 166;			// Given an initial value, after that just stores the old value for calculation


/*
 * PID function prototypes
 */
uint16_t PID_loop_right_wh();
uint16_t PID_loop_right_wh();
void adjust_PWM_right_wh(int PWM_Duty);
void adjust_PWM_left_wh(int PWM_Duty);
#endif /* MY_CODE_PID_CONTROLLERS_H_ */
