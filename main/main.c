#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "arch/sys_arch.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "string.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include <ds18x20.h>// For esp-idf-lib
#include "oled.h"
#include "main.h"
#define ENABLE_LOGGING 1 // Set to 0 to disable logging






// UART Configuration
#define UART_PORT       UART_NUM_0
#define UART_BAUD_RATE  115200
#define UART_TX_PIN     UART_PIN_NO_CHANGE
#define UART_RX_PIN     UART_PIN_NO_CHANGE

//ultrasonic Configuration
#define TRIG_PIN GPIO_NUM_5
#define ECHO_PIN GPIO_NUM_18
#define DISTANCE_BUFFER_SIZE 20

// DS18B20 Configuration
#define DS18B20_GPIO GPIO_NUM_4           // DS18B20 data pin
#define MAX_SENSORS 1


// Buzzer Configuration
#define BUZZER_PIN GPIO_NUM_19

static const char *TAG = "main";


// Shared Alert Structure
typedef struct {
    float distance;
    bool temp_alert; // Flag for temperature exceeding threshold
} AlertData;

static volatile uint64_t echo_start_time = 0;
static volatile uint64_t echo_end_time = 0;
static volatile bool distance_ready  = false;

static ds18x20_addr_t ds18b20_addrs[MAX_SENSORS];
static int ds18b20_sensor_count = 0;

volatile bool distance_alert_active = false;
volatile bool temperature_alert_active = false;

// Shared Display Data Structure
 char temp_display[TEMP_BUFFER_SIZE] = "--";
 char temp_fractional[TEMP_FRAC_BUFFER_SIZE]=".--";
 char dist_display[DIST_BUFFER_SIZE] = "--";
 char dist_display_status[DIST_STATUS_BUFFER_SIZE] = "--";


// Prepare UART buffer
char uart_buffer[32] ={0};












// Function to initialize UART
void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "UART initialized at %d baud", UART_BAUD_RATE);
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

// Calculate Distance from Ultrasonic Sensor
float calculate_distance() {
    if (distance_ready) {
        distance_ready = false;
        uint64_t duration = echo_end_time - echo_start_time;
        return (duration * 0.034) / 2.0;
    }
    return -1.0;
}

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

float distance_buffer[DISTANCE_BUFFER_SIZE];
int buffer_index = 0;


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
        float average_distance = get_average_distance();
        
        // Log and display the distance
        ESP_LOGI(TAG, "Average Distance: %.2f cm", average_distance);

        // Update display and alert logic
        if (average_distance > 0 && average_distance < BLINDSPOT) {
            strncpy(dist_display_status, "FULL!!", sizeof(dist_display_status));
            strncpy(dist_display, "0", sizeof(dist_display));
            distance_alert_active = false;
        } else if (average_distance >= BLINDSPOT && average_distance <= MAX_DIST_MESUR) {
            snprintf(dist_display, sizeof(dist_display), "-%d", (int)(average_distance));
            
            if ((int)average_distance > (BLINDSPOT + 5)) {
                distance_alert_active = true;
                strncpy(dist_display_status, "LOW!!", sizeof(dist_display_status));
            } else {
                distance_alert_active = false;
                strncpy(dist_display_status, "GOOD", sizeof(dist_display_status));
            }
        } else {
            distance_alert_active = false; // Out of range
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Control update rate
    }
}




// Temperature Task
void temperature_task(void *pvParameters) {
    while (1) {
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
            init_ds18b20();
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Read every 2 seconds
    }
}





// Buzzer Task
void buzzer_task(void *pvParameters) {
    bool buzzer_state = false;
    uint64_t last_buzzer_toggle_time = 0;

    while (1) {
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

        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to reduce CPU usage
    }
}






void oled_refresh_task(void *pvParameters) {
    static signed char count = 0;

    while (1) {
        if (count >= 0 && count < 2) {
            if (temperature_alert_active) {
                toggle_invert_display(true);
            } else {
                toggle_invert_display(false);
            }
            print_temp_oled();
        } else if (count >= 2 && count < 4) {
            if (distance_alert_active) {
                toggle_invert_display(true);
            } else {
                toggle_invert_display(false);
            }
            print_Water_level_oled();
        }

        count++;
        if (count == 4) {
            count = 0; // Reset the counter
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Refresh display every 1 second
    }
}





// Main Application
void app_main() {
    // Initialize Peripherals
    ssd1306_init();
    ssd1306_clear();
    init_ultrasonic_gpio();
    init_ds18b20();
    init_buzzer();
  /*
    oled_draw_string(0, 0, "ENV MONITOR");
    oled_draw_string(0, 16, "TEMP: N/A");
    oled_draw_string(0, 32, "DIST: ");
    oled_draw_string(0, 48, "ALERT: ");
    
     */
     
    #if ENABLE_LOGGING
    esp_log_level_set("main", ESP_LOG_INFO);
	#else
	esp_log_level_set("main", ESP_LOG_NONE);
	#endif
       
     Print_logo_diplay();
     sys_delay_ms(5000);
       
    // Create FreeRTOS Tasks
    xTaskCreate(temperature_task, "Temperature Task", 2048, NULL, 1, NULL);
    xTaskCreate(distance_task, "Distance Task", 2048, NULL, 2, NULL);
    xTaskCreate(buzzer_task, "Buzzer Task", 2048, NULL, 1, NULL);
    xTaskCreate(oled_refresh_task, "OLED Refresh Task", 2048, NULL, 1, NULL);
}