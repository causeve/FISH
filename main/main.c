
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
#include "main.h"
#include "oled.h"
#include "flashnvm.h"
#include "temperature.h"
#include "distance.h"
#include "buzzer.h"
#include "motor_control.h"
#define ENABLE_LOGGING 0 // Set to 0 to disable logging






// UART Configuration
#define UART_PORT       UART_NUM_0
#define UART_BAUD_RATE  115200
#define UART_TX_PIN     UART_PIN_NO_CHANGE
#define UART_RX_PIN     UART_PIN_NO_CHANGE








// GPIO pins for calibration mode
#define PUSH_BUTTON GPIO_NUM_33


static const char *TAG = "main";


typedef enum {
    STATE_NO_CHANGE,    // No state change
    STATE_ENTER_WATER_FILL,
    STATE_EXIT_WATER_FILL
} StateChangeRequest;

volatile StateChangeRequest state_change_request = STATE_NO_CHANGE;

SystemState current_state ; // Default state


 


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




void init_gpio_for_state_machine() {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PUSH_BUTTON),
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}


void water_level_low() {
    if (state_change_request == STATE_NO_CHANGE && current_state != WATER_FILL_MODE) {
        state_change_request = STATE_ENTER_WATER_FILL;
        ESP_LOGI("State Machine", "State change requested: Enter WATER_FILL_MODE.");
    }
}

void water_level_restored() {
    if (state_change_request == STATE_NO_CHANGE && current_state == WATER_FILL_MODE) {
        state_change_request = STATE_EXIT_WATER_FILL;
        ESP_LOGI("State Machine", "State change requested: Exit WATER_FILL_MODE.");
    }
}
void state_machine_task(void *pvParameters) {
    const uint32_t DEBOUNCE_TIME_MS = 50;       // Debounce time in milliseconds
    const uint32_t LONG_PRESS_THRESHOLD_MS = 2000; // Long press threshold (2 seconds)
    const uint32_t CLEAR_CALIBRATION_THRESHOLD_MS = 7000; // Clear calibration threshold (7 seconds)

    
    bool last_button_state = true;      // Previous stable state of the button (active-high)
    uint64_t press_start_tick = 0;      // Tick when the button press started
    uint64_t press_duration_ms = 0;     // Press duration in milliseconds

    while (1) {
		
		        // Handle state change requests (e.g., from distance_task)
        if (state_change_request != STATE_NO_CHANGE) {
            if (state_change_request == STATE_ENTER_WATER_FILL) {
                ESP_LOGI("State Machine", "Processing state change: Enter WATER_FILL_MODE.");
                current_state = WATER_FILL_MODE;
            } else if (state_change_request == STATE_EXIT_WATER_FILL) {
                ESP_LOGI("State Machine", "Processing state change: Exit WATER_FILL_MODE.");
                current_state = NORMAL_MODE;
            }
            state_change_request = STATE_NO_CHANGE; // Reset request
        }
		
		if (current_state == NORMAL_MODE || current_state == CALIBRATION_MODE) {
		        // Read the raw button state (active-low: 0 when pressed)
		        bool raw_button_state = gpio_get_level(PUSH_BUTTON);
		
		        // Debounce Logic: Only process state change after stability
		        if (raw_button_state != last_button_state) {
		            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS)); // Wait for debounce time
		
		            // Re-check the button state after debounce delay
		            if (gpio_get_level(PUSH_BUTTON) == raw_button_state) {
		                last_button_state = raw_button_state; // Update stable state
		
		                if (!last_button_state) { // Button Pressed (active-low: 0)
		                    press_start_tick = esp_timer_get_time(); // Record the press start time
		                } else { // Button Released (active-high: 1)
		                    // Calculate press duration
		                    press_duration_ms = (esp_timer_get_time() - press_start_tick) / 1000; // Convert to ms
		
		                    if (press_duration_ms >= CLEAR_CALIBRATION_THRESHOLD_MS) {
		                        // Clear Calibration Data
		                        clear_calibration_data();
		                        Clear_NVM_Display_oled();
		                         
		                        ESP_LOGI("State", "Button held for 7+ seconds: Calibration data cleared.");
		                        esp_restart(); // Trigger system restart
		                        // Provide feedback via OLED or other mechanism
		                    } else if (press_duration_ms >= LONG_PRESS_THRESHOLD_MS) {
		                        // Long Press: Enter Calibration Mode
		                        if (current_state != CALIBRATION_MODE) {
		                            current_state = CALIBRATION_MODE;
		                            ESP_LOGI("State", "Long press detected: Enter Calibration Mode.");
		                        }
		                    } else {
		                        // Short Press: Confirm Calibration
		                        if (current_state == CALIBRATION_MODE) {
		                            
		                            if (average_distance > BLINDSPOT) {
		                                save_base_distance(average_distance);
		                                save_calibration_status(true);
		                                ESP_LOGI("Calibration", "Calibration complete. Base distance set: %d cm", average_distance);
		                                nvm_base_distance = average_distance;
		                                current_state = NORMAL_MODE; // Transition to Normal Mode
		                            } else {
		                                ESP_LOGW("Calibration", "Invalid distance. Retry calibration.");
		                            }
		                        } else {
		                            ESP_LOGW("State", "Short press ignored in Normal Mode.");
		                        }
		                    }
		                }
		            }
		        }
        }else if (current_state == WATER_FILL_MODE) {
          // Add water filling-related logic if necessary
            ESP_LOGI("State Machine", "In WATER_FILL_MODE. Skipping button operations.");
		}

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay for efficient polling
    }
}









// Main Application
void app_main() {
    // Initialize Peripherals
    init_nvs();
    ssd1306_init();
    ssd1306_clear();
    init_gpio_for_state_machine();
    init_ultrasonic_gpio();
    init_ds18b20();
    init_buzzer();
    init_motor_control();
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
     
     // Check calibration status and initialize state
    check_first_time_calibration();
     sys_delay_ms(3000);
       
    // Create FreeRTOS Tasks
    xTaskCreate(temperature_task, "Temperature Task", 2048, NULL, 1, NULL);
    xTaskCreate(distance_task, "Distance Task", 2048, NULL, 2, NULL);
    xTaskCreate(buzzer_task, "Buzzer Task", 2048, NULL, 1, NULL);
    xTaskCreate(oled_refresh_task, "OLED Refresh Task", 2048, NULL, 1, NULL);
     xTaskCreate(state_machine_task, "State Machine Task", 2048, NULL, 2, NULL);
     xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 2, NULL);
    
}