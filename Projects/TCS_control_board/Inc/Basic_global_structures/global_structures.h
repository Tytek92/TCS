/*
 * global_structures.h
 *
 *  Created on: 11 gru 2016
 *      Author: Tymoteusz
 */

#ifndef BASIC_GLOBAL_STRUCTURES_GLOBAL_STRUCTURES_H_
#define BASIC_GLOBAL_STRUCTURES_GLOBAL_STRUCTURES_H_
/*
 * struct Disp_BCD_Data
 * access to this structure is guarded by osMutexId Disp_BCD_StateHandle
 * Tasks accessing this structure:
 * 									* Display_Task
 */
struct Disp_BCD_Data {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} Disp_BCD_Data;


#endif /* BASIC_GLOBAL_STRUCTURES_GLOBAL_STRUCTURES_H_ */
