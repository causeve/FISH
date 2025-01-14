/*
 * flashnvm.h
 *
 *  Created on: 09-Jan-2025
 *      Author: ilang
 */

#ifndef MAIN_FLASHNVM_H_
#define MAIN_FLASHNVM_H_
#include <stdint.h>
#include <stdbool.h>


void init_nvs() ;
void save_base_distance(float );
void save_base_distance(float);
int load_base_distance();
void save_calibration_status(bool);
bool load_calibration_status();
void check_first_time_calibration();
void clear_calibration_data();

#endif /* MAIN_FLASHNVM_H_ */
