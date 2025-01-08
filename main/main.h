/*
 * main.h
 *
 *  Created on: 29-Dec-2024
 *      Author: ilang
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

#define TEMP_BUFFER_SIZE 5
#define TEMP_FRAC_BUFFER_SIZE 5
#define DIST_BUFFER_SIZE 16 
#define DIST_STATUS_BUFFER_SIZE 16

#define BLINDSPOT 2.0  // 2 cm Blind spot
#define MAX_DIST_MESUR 100.0  //100cm


// Shared Display Data Structure
extern  char temp_display[TEMP_BUFFER_SIZE];
extern  char temp_fractional[TEMP_FRAC_BUFFER_SIZE];
extern  char dist_display[DIST_BUFFER_SIZE];
extern  char dist_display_status[DIST_STATUS_BUFFER_SIZE];


#endif /* MAIN_MAIN_H_ */
