/*
 * motor_control.h
 *
 *  Created on: 10-Jan-2025
 *      Author: ilang
 */

#ifndef MAIN_MOTOR_CONTROL_H_
#define MAIN_MOTOR_CONTROL_H_
#include <stdbool.h>


bool get_motor_running() ;
void motor_control_task(void *pvParameters);
void init_motor_control();

#endif /* MAIN_MOTOR_CONTROL_H_ */
