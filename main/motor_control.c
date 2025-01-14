#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "motor_control.h"
#include "main.h"
#include <distance.h>
// GPIO and PWM Configuration
// Choose Mapping Method
//#define USE_QUADRATIC_MAPPING
 #define USE_LINEAR_MAPPING

#define MOTOR_PWM_PIN GPIO_NUM_25         // GPIO for PWM signal
#define MOTOR_IN1_PIN GPIO_NUM_26         // GPIO for IN1
#define MOTOR_IN2_PIN GPIO_NUM_27         // GPIO for IN2
#define PWM_FREQUENCY 1000                // 1 kHz PWM frequency
#define MAX_DUTY_CYCLE 192                // 75% of 255
#define MIN_DUTY_CYCLE 126                 // 30% of 255
#define MOTOR_CONTROL_TASK_DELAY_MS 100   // Control task polling rate

// Motor State
static bool motor_running = false;       // Motor running status


static const char *TAG = "Motor Control";


// Initialize the PWM and Motor GPIOs
void init_motor_control() {
    // Configure PWM Timer
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    // Configure PWM Channel
    ledc_channel_config_t pwm_channel = {
        .gpio_num = MOTOR_PWM_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // Start with motor OFF
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));

    // Configure IN1 and IN2 as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1_PIN) | (1ULL << MOTOR_IN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Set initial motor state
    gpio_set_level(MOTOR_IN1_PIN, 0);
    gpio_set_level(MOTOR_IN2_PIN, 0);

    ESP_LOGI(TAG, "Motor control initialized.");
}

// Set the motor's PWM duty cycle
void set_motor_duty_cycle(uint8_t duty_cycle) {
    ESP_LOGI(TAG, "Setting motor duty cycle: %d", duty_cycle);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

// Start the motor in forward direction
void motor_start_forward(uint8_t duty_cycle) {
    motor_running = true; // Set motor running state
    gpio_set_level(MOTOR_IN1_PIN, 1);    // IN1 HIGH
    gpio_set_level(MOTOR_IN2_PIN, 0);    // IN2 LOW
    set_motor_duty_cycle(duty_cycle);    // Set PWM duty cycle
    ESP_LOGI(TAG, "Motor started in forward direction.");
}

// Stop the motor
void motor_stop() {
    motor_running = false; // Reset motor running state
    gpio_set_level(MOTOR_IN1_PIN, 0);   // IN1 LOW
    gpio_set_level(MOTOR_IN2_PIN, 0);   // IN2 LOW
    set_motor_duty_cycle(0);            // Turn OFF PWM
    ESP_LOGI(TAG, "Motor stopped.");
}

// Get the motor running status
bool get_motor_running() {
    return motor_running;
}


// Motor Control Task
#include <math.h> // For powf() in quadratic mapping
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"

// Choose Mapping Method
//#define USE_QUADRATIC_MAPPING
 #define USE_LINEAR_MAPPING

void motor_control_task(void *pvParameters) {
    while (1) {
        if (current_state == WATER_FILL_MODE) {
            if (!motor_running) {
                motor_start_forward(MIN_DUTY_CYCLE); // Start motor at minimum speed
            }

            // Adjust motor speed dynamically based on selected mapping method
            float current_level = average_distance; // Current water level
            float base_level = nvm_base_distance;   // Base water level
            float max_diff = 5.0;                  // Maximum difference (base - 5 cm)
            uint8_t duty_cycle;

            if (current_level > (base_level + 5)) {
                float level_diff = current_level - (base_level + 5);

#ifdef USE_QUADRATIC_MAPPING
                // Quadratic Mapping
                duty_cycle = MIN_DUTY_CYCLE +
                             (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) * powf(level_diff / max_diff, 2);
                ESP_LOGI("Motor Control", "Quadratic Mapping: Level Diff: %.2f, Duty Cycle: %d", level_diff, duty_cycle);

#elif defined(USE_LINEAR_MAPPING)
                // Linear Mapping
                duty_cycle = MIN_DUTY_CYCLE +
                             ((level_diff / max_diff) * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE));
                ESP_LOGI("Motor Control", "Linear Mapping: Level Diff: %.2f, Duty Cycle: %d", level_diff, duty_cycle);
#endif

                // Clamp the duty cycle
                if (duty_cycle > MAX_DUTY_CYCLE) duty_cycle = MAX_DUTY_CYCLE;

                set_motor_duty_cycle(duty_cycle); // Update motor speed
            } else if (current_level <= base_level) {
                set_motor_duty_cycle(MIN_DUTY_CYCLE); // Set to minimum speed at base level
            }

        } else {
            // Stop motor when not in WATER_FILL_MODE
            if (motor_running) {
                motor_stop();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_TASK_DELAY_MS)); // Polling delay
    }
}


