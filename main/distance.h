/*
 * distance.h
 *
 *  Created on: 09-Jan-2025
 *      Author: ilang
 */

#ifndef MAIN_DISTANCE_H_
#define MAIN_DISTANCE_H_
#include <stdbool.h>

#define DIST_BUFFER_SIZE 16 
#define DIST_STATUS_BUFFER_SIZE 16

#define BLINDSPOT 2  // 2 cm Blind spot
#define MAX_DIST_MESUR 100 //100cm

extern int nvm_base_distance ;
extern volatile bool distance_alert_active;
extern volatile int average_distance;

 extern char dist_display[DIST_BUFFER_SIZE] ;
 extern char dist_display_status[DIST_STATUS_BUFFER_SIZE];

void init_ultrasonic_gpio();
void distance_task(void *pvParameters);

#endif /* MAIN_DISTANCE_H_ */
