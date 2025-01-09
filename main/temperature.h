/*
 * temperature.h
 *
 *  Created on: 09-Jan-2025
 *      Author: ilang
 */

#ifndef MAIN_TEMPERATURE_H_
#define MAIN_TEMPERATURE_H_
#include <stdbool.h>

#define TEMP_BUFFER_SIZE 5
#define TEMP_FRAC_BUFFER_SIZE 5

extern  char temp_display[TEMP_BUFFER_SIZE];
extern  char temp_fractional[TEMP_FRAC_BUFFER_SIZE];

extern volatile bool temperature_alert_active;

void temperature_task(void *pvParameters);
void init_ds18b20();


#endif /* MAIN_TEMPERATURE_H_ */
