/*
 * distance.c
 *
 *  Created on: 09-Jan-2025
 *      Author: ilang
 */
#include "distance.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "arch/sys_arch.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include <main.h>

//ultrasonic Configuration
#define TRIG_PIN GPIO_NUM_5
#define ECHO_PIN GPIO_NUM_18
#define DISTANCE_BUFFER_SIZE 20

static const char *TAG = "distance";


float distance_buffer[DISTANCE_BUFFER_SIZE];
int buffer_index = 0;


float nvm_base_distance = 0.0;
static volatile uint64_t echo_start_time = 0;
static volatile uint64_t echo_end_time = 0;
static volatile bool distance_ready  = false;

volatile float average_distance =0.0;




volatile bool distance_alert_active = false;


// Shared Display Data Structure

 char dist_display[DIST_BUFFER_SIZE] = "--";
 char dist_display_status[DIST_STATUS_BUFFER_SIZE] = "--";

// Calculate Distance from Ultrasonic Sensor
float calculate_distance() {
    if (distance_ready) {
        distance_ready = false;
        uint64_t duration = echo_end_time - echo_start_time;
        return (duration * 0.034) / 2.0;
    }
    return -1.0;
}


// GPIO Interrupt Handler for Ultrasonic
static void IRAM_ATTR echo_isr_handler(void *arg) {
    if (gpio_get_level(ECHO_PIN) == 1) {
        echo_start_time = esp_timer_get_time();
    } else {
        echo_end_time = esp_timer_get_time();
        distance_ready = true;
    }
}

// Initialize GPIO for Ultrasonic Sensor
void init_ultrasonic_gpio() {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_PIN, echo_isr_handler, NULL);
}

// Trigger Ultrasonic Sensor
void trigger_ultrasonic() {
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);
}







void add_to_buffer(float distance) {
    distance_buffer[buffer_index] = distance;
    buffer_index = (buffer_index + 1) % DISTANCE_BUFFER_SIZE;
}

float get_average_distance() {
    float sum = 0.0;
    for (int i = 0; i < DISTANCE_BUFFER_SIZE; i++) {
        sum += distance_buffer[i];
    }
    return sum / DISTANCE_BUFFER_SIZE;
}

float validate_distance(float current_distance, float previous_distance) {
    if (current_distance < 2.0 || current_distance >400.0) {
        return -1.0; // Invalid range
    }
    
    /*
    if (fabs(current_distance - previous_distance) > 20.0) {
        return previous_distance; // Ignore sudden spikes
    }
    */
    return current_distance;
}


// Distance Task
void distance_task(void *pvParameters) {
	
	
    float distance = 0.0, previous_distance = 0.0;

    // Initialize buffer for averaging
    for (int i = 0; i < DISTANCE_BUFFER_SIZE; i++) {
        distance_buffer[i] = 0.0;
    }

    while (1) {
        // Trigger the ultrasonic sensor
        trigger_ultrasonic();
        vTaskDelay(pdMS_TO_TICKS(60)); // Allow time for the echo to complete

        // Calculate the distance
        distance = calculate_distance();
        if (distance == -1.0) {
            ESP_LOGW(TAG, "Invalid distance reading!");
            continue; // Skip if invalid
        }

        // Validate and filter the distance
        distance = validate_distance(distance, previous_distance);
        if (distance == -1.0) {
			strncpy(dist_display_status, "FULL", sizeof(dist_display_status));
            strncpy(dist_display, "0", sizeof(dist_display));
            ESP_LOGW(TAG, "Filtered out spurious distance reading!");
            continue; // Skip spurious readings
        }

        previous_distance = distance;

        // Add to buffer and calculate average
        add_to_buffer(distance);
        float  loc_average_distance = average_distance= get_average_distance();
        
        // Log and display the distance
        ESP_LOGI(TAG, "Average Distance: %.2f cm", loc_average_distance);
        
		if (current_state == NORMAL_MODE) {
	        // Update display and alert logic
	        if (loc_average_distance > 0 && loc_average_distance < (nvm_base_distance-1)) {
	            strncpy(dist_display_status, "FULL!!", sizeof(dist_display_status));
	            strncpy(dist_display, "0", sizeof(dist_display));
	            distance_alert_active = false;
	        } else if (loc_average_distance >= BLINDSPOT && loc_average_distance <= MAX_DIST_MESUR) {
	            snprintf(dist_display, sizeof(dist_display), "%d", (int)(loc_average_distance - nvm_base_distance));
	            
	            if ((int)loc_average_distance > (nvm_base_distance + 5)) {
	                distance_alert_active = true;
	                strncpy(dist_display_status, "LOW!!", sizeof(dist_display_status));
	            } else {
	                distance_alert_active = false;
	                strncpy(dist_display_status, "GOOD", sizeof(dist_display_status));
	            }
	        } else {
	            distance_alert_active = false; // Out of range
	        }
         }else if (current_state == CALIBRATION_MODE) {
			 
			 snprintf(dist_display, sizeof(dist_display), "%d", (int)(loc_average_distance));

		}
        vTaskDelay(pdMS_TO_TICKS(100)); // Control update rate
    }
}