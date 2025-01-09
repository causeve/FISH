/*
 * main.h
 *
 *  Created on: 29-Dec-2024
 *      Author: ilang
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_






//#define LONG_PRESS_THRESHOLD_MS 4000 // Long press: 4 seconds

#define DEBOUNCE_DELAY_MS 50 // 50 ms debounce time


typedef enum {
    NORMAL_MODE,
    CALIBRATION_MODE,
    NONE
} SystemState;

extern SystemState current_state;





#endif /* MAIN_MAIN_H_ */
