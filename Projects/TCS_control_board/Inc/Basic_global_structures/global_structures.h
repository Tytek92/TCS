/*
 * global_structures.h
 *
 *  Created on: 11 gru 2016
 *      Author: Tymoteusz
 */

#ifndef BASIC_GLOBAL_STRUCTURES_GLOBAL_STRUCTURES_H_
#define BASIC_GLOBAL_STRUCTURES_GLOBAL_STRUCTURES_H_
#include <stdint.h>
/*
 * struct Disp_BCD_Data
 * access to this structure is guarded by osMutexId Disp_BCD_StateHandle
 * Tasks accessing this structure:
 * 									* Display_Task
 */

struct System_State {
	int16_t AngularVelocityRearRightWh;
	int16_t AngularVelocityRearLeftWh;
	int16_t TargetAngularVelocityRearRightWh;
	int16_t TargetAngularVelocityRearLeftWh;
	int16_t SystemLinearVelocity; // meters per second
	int16_t CurrentDrawRightWh;
	int16_t CurrentDrawLeftWh;
}System_State;

struct Disp_BCD_Data {
    uint8_t blink_status;
    uint8_t blink_frequency;
    uint8_t dp1_status;
    uint8_t dp2_status;
    /*
     *   uint8_t displayed_character
     *
     *   	field value:				sign:
     *   			0						0
     *   			1						1
     *   			2						2
     *   			3						3
     *   			4						4
     *   			5						5
     *   			6						6
     *   			7						7
     *   			8						8
     *   			9						9
     *   			10						A
     *   			11						C
     *   			12						U
     *   			13						E
     *   			14						c
     *   			15						F
     *   			16						r
     *
     *
     */
    uint8_t displayed_character;
} Disp_BCD_Data;

union FrameBuffer{
	char byte[76];
	uint32_t word[19];
};




/*
 * union TCS_input_data
 * Contains data received from host controller
 * access to this structure is guarded by osMutexId ##############
 * Tasks accessing this structure:
 * 									* #############
 */
union TCS_input_data{
	struct Single_Fields{
		uint32_t System_Mode;				/*System operation mode field
		*/
		uint32_t Linear_Velocity;			/*Linear target velocity
			*/
		uint32_t Turn_Angle;				/*Front axis turn angle
			*/
		uint32_t Velocity_Limit;			/*Linear speed boundary
		 	*/
		uint32_t Field0;					/*System operation mode field
		 	*/
		uint32_t Field1;					/*Linear target velocity
			*/
		uint32_t Field2;					/*Front axis turn angle
			*/
		uint32_t Field3;					/*System operation mode field
			*/
		uint32_t Field4;					/*System operation mode field
			*/
		uint32_t Field5;					/*System operation mode field
			*/
		uint32_t Field6;					/*System operation mode field
			*/
		uint32_t Field7;					/*System operation mode field
			*/
		uint32_t Field8;					/*System operation mode field
			*/
		uint32_t Field9;					/*System operation mode field
			*/
		uint32_t Field10;					/*System operation mode field
			*/
		uint32_t Field11;					/*System operation mode field
			*/
		uint32_t Field12;					/*System operation mode field
			*/
		uint32_t Field13;					/*System operation mode field
			*/
		uint32_t Field14;					/*System operation mode field
			*/
		uint32_t Field15;					/*System operation mode field
			*/
		uint32_t Field16;					/*System operation mode field
			*/
		uint32_t Field17;					/*System operation mode field
			*/
	}Fields;
	uint32_t Concatenated_Fields[18];
};


#endif /* BASIC_GLOBAL_STRUCTURES_GLOBAL_STRUCTURES_H_ */
