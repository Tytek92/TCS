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


#endif /* BASIC_GLOBAL_STRUCTURES_GLOBAL_STRUCTURES_H_ */
