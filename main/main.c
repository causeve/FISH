#include <stdio.h>
#include "arch/sys_arch.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "string.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include <ds18x20.h>// For esp-idf-lib

// I2c  Configuration
#define I2C_MASTER_SCL_IO 22              // SCL pin
#define I2C_MASTER_SDA_IO 21              // SDA pin
#define I2C_MASTER_NUM I2C_NUM_0          // I2C port number
#define I2C_MASTER_FREQ_HZ 100000         // I2C frequency

// OLED Configuration
#define OLED_I2C_ADDRESS 0x3C             // OLED I2C address
#define OLED_WIDTH 128                    // OLED display width
#define OLED_HEIGHT 64                    // OLED display height


// UART Configuration
#define UART_PORT       UART_NUM_0
#define UART_BAUD_RATE  115200
#define UART_TX_PIN     UART_PIN_NO_CHANGE
#define UART_RX_PIN     UART_PIN_NO_CHANGE

//ultrasonic Configuration
#define TRIG_PIN GPIO_NUM_5
#define ECHO_PIN GPIO_NUM_18

// DS18B20 Configuration
#define DS18B20_GPIO GPIO_NUM_4           // DS18B20 data pin
#define MAX_SENSORS 1


// Buzzer Configuration
#define BUZZER_PIN GPIO_NUM_19

static const char *TAG = "SSD1306";

static volatile uint64_t echo_start_time = 0;
static volatile uint64_t echo_end_time = 0;
static volatile bool distance_ready  = false;

static ds18x20_addr_t ds18b20_addrs[MAX_SENSORS];
static int ds18b20_sensor_count = 0;

// Font data for 8x8 ASCII characters
static const uint8_t font8x8_basic[96][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00, 0x00}, // !
    {0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 0x00, 0x00}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 0x00, 0x00}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x00, 0x00}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x00, 0x00}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14, 0x00, 0x00, 0x00}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00, 0x00}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00, 0x00}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00}, // 1
    {0x72, 0x49, 0x49, 0x49, 0x46, 0x00, 0x00, 0x00}, // 2
    {0x21, 0x41, 0x49, 0x4D, 0x33, 0x00, 0x00, 0x00}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00, 0x00}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00, 0x00}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00, 0x00}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00, 0x00}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00, 0x00}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00, 0x00}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00}, // >
    {0x02, 0x01, 0x59, 0x09, 0x06, 0x00, 0x00, 0x00}, // ?
    {0x3E, 0x41, 0x5D, 0x55, 0x1E, 0x00, 0x00, 0x00}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00, 0x00, 0x00}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00, 0x00}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00, 0x00}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00, 0x00}, // F
    {0x3E, 0x41, 0x41, 0x51, 0x71, 0x00, 0x00, 0x00}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00, 0x00}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00, 0x00}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00}, // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F, 0x00, 0x00, 0x00}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00, 0x00}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00, 0x00}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00, 0x00}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00, 0x00}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31, 0x00, 0x00, 0x00}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00, 0x00}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00, 0x00}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00, 0x00}, // V
    {0x3F, 0x40, 0x30, 0x40, 0x3F, 0x00, 0x00, 0x00}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00, 0x00}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07, 0x00, 0x00, 0x00}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00, 0x00}, // Z
};

// Prepare UART buffer
char uart_buffer[32] ={0};


// Function to initialize DS18B20
void init_ds18b20() {
    size_t found_devices = 0;
    
    gpio_set_pull_mode(DS18B20_GPIO, GPIO_PULLUP_ONLY);
    esp_err_t err = ds18x20_scan_devices(DS18B20_GPIO, ds18b20_addrs, MAX_SENSORS, &found_devices);
    ds18b20_sensor_count = (int)found_devices;

    if (err == ESP_OK && ds18b20_sensor_count > 0) {
        ESP_LOGI(TAG, "Found %d DS18B20 sensor(s)", ds18b20_sensor_count);
    } else {
        ESP_LOGE(TAG, "No DS18B20 sensors found! Error: %d", err);
        ds18b20_sensor_count = 0; // Ensure no attempts to read invalid sensors
    }
}

// Function to read temperature from DS18B20
float read_temperature() {
	 
    if (ds18b20_sensor_count > 0) {
        float temp = 0.0;
        
        #if 0
        // Issue a measurement command and wait for the conversion to complete
        esp_err_t err = ds18x20_measure(DS18B20_GPIO, ds18b20_addrs[0], true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start temperature measurement. Error: %d", err);
            return -1000.0; // Invalid reading
        }

        // Wait for measurement to complete (750ms for 12-bit resolution)
        vTaskDelay(pdMS_TO_TICKS(750));
#endif
        // Read the temperature
       esp_err_t err = ds18b20_measure_and_read(DS18B20_GPIO, ds18b20_addrs[0], &temp);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f", temp);
            return temp; // Valid reading
        } else {
            ESP_LOGE(TAG, "Failed to read temperature. Error: %d", err);
        }
    }
    return -1000.0; // Invalid reading
}



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

// I2C initialization
void i2c_master_init() {
    esp_err_t ret;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
    }
}

// Function to send commands to OLED
void oled_send_command(uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Control byte for commands
    i2c_master_write_byte(cmd, command, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// Function to send data to OLED
void oled_send_data(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true);  // Control byte for data
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// OLED initialization
void oled_init() {
    oled_send_command(0xAE);  // Display OFF
    oled_send_command(0xD5);  // Set Display Clock Divide Ratio
    oled_send_command(0x80);
    oled_send_command(0xA8);  // Set Multiplex Ratio
    oled_send_command(0x3F);  // For 128x64
    oled_send_command(0xD3);  // Set Display Offset
    oled_send_command(0x00);
    oled_send_command(0x40);  // Set Display Start Line
    oled_send_command(0x8D);  // Enable Charge Pump
    oled_send_command(0x14);
    oled_send_command(0x20);  // Set Memory Addressing Mode
    oled_send_command(0x00);  // Horizontal Addressing Mode
    oled_send_command(0xA1);  // Set Segment Re-map
    oled_send_command(0xC8);  // Set COM Output Scan Direction
    oled_send_command(0xDA);  // Set COM Pins Hardware Configuration
    oled_send_command(0x12);
    oled_send_command(0x81);  // Set Contrast Control
    oled_send_command(0x7F);
    oled_send_command(0xD9);  // Set Pre-charge Period
    oled_send_command(0xF1);
    oled_send_command(0xDB);  // Set VCOMH Deselect Level
    oled_send_command(0x40);
    oled_send_command(0xA4);  // Disable Entire Display ON
    oled_send_command(0xA6);  // Normal Display
    oled_send_command(0xAF);  // Display ON
}

// Clear OLED screen
void oled_clear() {
    for (uint16_t i = 0; i < OLED_WIDTH * OLED_HEIGHT / 8; i++) {
        oled_send_data(0x00);
    }
}

// Draw a single character
void oled_draw_char(uint8_t x, uint8_t y, char c) {
    if (c < 32 || c > 127) return;  // Ignore non-printable characters
    uint8_t *bitmap = font8x8_basic[c - 32];
    oled_send_command(0xB0 + (y / 8));        // Set page address
    oled_send_command(0x00 + (x & 0x0F));     // Lower column address
    oled_send_command(0x10 + (x >> 4));       // Higher column address
    for (int i = 0; i < 8; i++) {
        oled_send_data(bitmap[i]);
    }
}

// Draw a string
void oled_draw_string(uint8_t x, uint8_t y, const char *str) {
    while (*str) {
        oled_draw_char(x, y, *str);
        x += 8;  // Move to the next character position
        if (x + 8 > OLED_WIDTH) {  // Wrap to the next line if necessary
            x = 0;
            y += 8;
        }
        str++;
    }
}





// Function to initialize the buzzer
void init_buzzer() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(BUZZER_PIN, 0); // Ensure buzzer is off initially
}


// Function to control the buzzer
void toggle_buzzer(bool state) {
    gpio_set_level(BUZZER_PIN, state ? 1 : 0);
}

// GPIO interrupt handler for ECHO pin
static void IRAM_ATTR echo_isr_handler(void *arg) {
    if (gpio_get_level(ECHO_PIN) == 1) {
        // Rising edge: Start time
        echo_start_time = esp_timer_get_time();
    } else {
        // Falling edge: End time
        echo_end_time = esp_timer_get_time();
        distance_ready = true;
    }
}


// Function to initialize GPIO for ultrasonic sensor
void init_ultrasonic_gpio() {
    // Configure TRIG_PIN as output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    // Configure ECHO_PIN as input with interrupt on both edges
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);

    // Install GPIO interrupt service and attach the handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_PIN, echo_isr_handler, NULL);
}

// Function to trigger the ultrasonic sensor
void trigger_ultrasonic() {
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);  // Ensure a clean LOW signal
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10); // 10 µs HIGH pulse
    gpio_set_level(TRIG_PIN, 0);
}

// Function to calculate distance
float calculate_distance() {
    if (distance_ready) {
        distance_ready = false;
        uint64_t duration = echo_end_time - echo_start_time;
        return (duration * 0.034) / 2.0; // Distance in cm
    }
    return -1.0; // Distance not ready
}

// Main application
void app_main() {
	 // Initialize ultrasonic GPIO
	    // Initialize ultrasonic sensor
	       init_buzzer();
    init_ultrasonic_gpio();
 
    init_uart();
    i2c_master_init();
    oled_init();
    oled_clear();
    init_ds18b20();
   
    //logo
    
    oled_draw_string(0, 48, "CAUSEVE.....");
    sys_delay_ms(5000);
    // Initialize DS18B20
   // ds18b20_init();
     oled_clear();
    // Display static text on OLED
    oled_draw_string(0, 0, "ENV MONITOR");
    oled_draw_string(0, 16, "TEMP: N/A");
    oled_draw_string(0, 32, "DIST: ");
    oled_draw_string(0, 48, "ALERT: ");
    
       bool buzzer_state = false;
    uint64_t last_buzzer_toggle_time = 0;
    
while (1) {
	
		        // Read temperature and update OLED
        float temp = read_temperature();
        ESP_LOGI("Test", "Temperature: %.2f", temp);
        if (temp > -1000.0) {
            char temp_str[16];
            snprintf(temp_str, sizeof(temp_str), "%.2f C", temp);
            oled_draw_string(50, 16, temp_str);
        } else {
            oled_draw_string(50, 16, "123");
        }

	
        // Trigger ultrasonic sensor
        trigger_ultrasonic();
        //vTaskDelay(pdMS_TO_TICKS(60)); // Allow enough time for measurement

        // Calculate distance
        float distance = calculate_distance();

        // Handle distance display and buzzer logic
        if (distance > 0 && distance < 18.0) {
            oled_draw_string(50, 32, "BELOW MIN");
            toggle_buzzer(false);
        } else if (distance > 18.0) {
            char dist_str[16];
            snprintf(dist_str, sizeof(dist_str), "%.2f cm", distance);
            oled_draw_string(50, 32, dist_str);

            if (distance > 27.0) {
                uint64_t current_time = esp_timer_get_time();
                if ((current_time - last_buzzer_toggle_time) > 3000000) { // 3 seconds in µs
                    buzzer_state = !buzzer_state;
                    toggle_buzzer(buzzer_state);
                    last_buzzer_toggle_time = current_time;
                }
                oled_draw_string(50, 48, "ON ");
            } else {
                buzzer_state = false;
                toggle_buzzer(buzzer_state);
                oled_draw_string(50, 48, "OFF");
            }
        } else {
            //oled_draw_string(50, 32, "ERROR");
            toggle_buzzer(false);
        }



        vTaskDelay(pdMS_TO_TICKS(100)); // Main loop delay
    }
}

