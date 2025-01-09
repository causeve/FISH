/*
 * buzzer.c
 *
 *  Created on: 09-Jan-2025
 *      Author: ilang
 */


#include "buzzer.h"
#include "driver/gpio.h"
#include "main.h"
#include "esp_timer.h"
#include "distance.h"
#include "temperature.h"
#include "arch/sys_arch.h"





// Buzzer Configuration
#define BUZZER_PIN GPIO_NUM_19






// Initialize Buzzer
void init_buzzer() {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_PIN, 0);
}

// Toggle Buzzer State
void toggle_buzzer(bool state) {
    gpio_set_level(BUZZER_PIN, state ? 1 : 0);
}

// Buzzer Task
void buzzer_task(void *pvParameters) {
    bool buzzer_state = false;
    uint64_t last_buzzer_toggle_time = 0;

    while (1) {
		
		if (current_state == NORMAL_MODE) {
		        uint64_t current_time = esp_timer_get_time();
		
		        // Temperature Alert
		        if (temperature_alert_active && !distance_alert_active) {
		            // Toggle buzzer every 1 second for temperature alert
		            if ((current_time - last_buzzer_toggle_time) > 1000000) {
		                buzzer_state = !buzzer_state;
		                toggle_buzzer(buzzer_state);
		                last_buzzer_toggle_time = current_time;
		            }
		        }
		        // Water Level Alert
		        else if (distance_alert_active && !temperature_alert_active) {
		            // Toggle buzzer every 3 seconds for water level alert
		            if ((current_time - last_buzzer_toggle_time) > 3000000) {
		                buzzer_state = !buzzer_state;
		                toggle_buzzer(buzzer_state);
		                last_buzzer_toggle_time = current_time;
		            }
		        }
		        // Both Alerts Active
		        else if (temperature_alert_active && distance_alert_active) {
		            // Prioritize temperature alert (shorter interval)
		            if ((current_time - last_buzzer_toggle_time) > 1000000) {
		                buzzer_state = !buzzer_state;
		                toggle_buzzer(buzzer_state);
		                last_buzzer_toggle_time = current_time;
		            }
		        }
		        // No Alerts
		        else {
		            // Turn off the buzzer
		            toggle_buzzer(false);
		            buzzer_state = false;
		        }
		}
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to reduce CPU usage
    }
}