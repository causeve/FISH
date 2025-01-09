/*
 * temperature.c
 *
 *  Created on: 09-Jan-2025
 *      Author: ilang
 */
#include "temperature.h"
#include <ds18x20.h>// For esp-idf-lib
#include <main.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include "arch/sys_arch.h"


// DS18B20 Configuration
#define DS18B20_GPIO GPIO_NUM_4           // DS18B20 data pin
#define MAX_SENSORS 1

static const char *TAG = "Temperature";


volatile bool temperature_alert_active = false;

 char temp_display[TEMP_BUFFER_SIZE] = "--";
 char temp_fractional[TEMP_FRAC_BUFFER_SIZE]=".--";

static ds18x20_addr_t ds18b20_addrs[MAX_SENSORS];
static int ds18b20_sensor_count = 0;

// Initialize DS18B20 Sensor
void init_ds18b20() {
    size_t found_devices = 0;
    gpio_set_pull_mode(DS18B20_GPIO, GPIO_PULLUP_ONLY);
    esp_err_t err = ds18x20_scan_devices(DS18B20_GPIO, ds18b20_addrs, MAX_SENSORS, &found_devices);
    ds18b20_sensor_count = (int)found_devices;

    if (err == ESP_OK && ds18b20_sensor_count > 0) {
        ESP_LOGI(TAG, "Found %d DS18B20 sensor(s)", ds18b20_sensor_count);
    } else {
        ESP_LOGE(TAG, "No DS18B20 sensors found! Error: %d", err);
        ds18b20_sensor_count = 0;
    }
}

// Read Temperature from DS18B20
float read_temperature() {
    if (ds18b20_sensor_count > 0) {
        float temp = 0.0;
        esp_err_t err = ds18b20_measure_and_read(DS18B20_GPIO, ds18b20_addrs[0], &temp);
        if (err == ESP_OK) {
            return temp;
        }
    }
    return -1000.0; // Invalid reading
}



// Temperature Task
void temperature_task(void *pvParameters) {
	 int retry_count = 0;
    const int max_retries = 5; // Limit the number of retries
    while (1) {
		
		if (current_state == NORMAL_MODE) {
	        float temp = read_temperature();
	
	        if (temp > -1000.0) { // Valid temperature
	            ESP_LOGI(TAG, "Temperature: %.1f", temp);
	            int integer_part = (int)temp;
	            float fractional_part = temp - integer_part;
	            snprintf(temp_fractional, sizeof(temp_fractional), ".%02d", (int)(fractional_part * 100));
	            snprintf(temp_display, sizeof(temp_display), "%d", integer_part);
	
	            temperature_alert_active = (temp >= 32.0);
	        } else {
	            ESP_LOGE(TAG, "Invalid temperature reading!");
	            strncpy(temp_display, "--", sizeof(temp_display));
	            strncpy(temp_fractional, ".00", sizeof(temp_fractional));
	            // Reinitialize DS18B20 if reading fails repeatedly
	            if (retry_count < max_retries) {
                    ESP_LOGE(TAG, "Reinitializing DS18B20 sensor. Retry %d of %d.", retry_count + 1, max_retries);
                    init_ds18b20();
                    retry_count++;
                } else {
                    ESP_LOGW(TAG, "Max retries reached. Skipping sensor initialization.");
                }
	        }
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Read every 2 seconds
    }
}