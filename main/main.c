#include <stdbool.h>
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

#define ENABLE_LOGGING 0 // Set to 0 to disable logging

// I2c  Configuration
#define I2C_MASTER_SCL_IO 22              // SCL pin
#define I2C_MASTER_SDA_IO 21              // SDA pin
#define I2C_MASTER_NUM I2C_NUM_0          // I2C port number
#define I2C_MASTER_FREQ_HZ 400000         // I2C frequency

// OLED Configuration
#define OLED_ADDR 0x3C             // OLED I2C address
// Define display width and height
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64


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

static const char *TAG = "ENV_MONITOR";


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
static char temp_display[5] = "--";
static char temp_fractional[5]=".--";
static char dist_display[16] = "--";
static char dist_display_status[16] = "GOOD -_-";


// Prepare UART buffer
char uart_buffer[32] ={0};

const uint8_t degree_symbol[] = {
    0x06, //  ▓▓▓▓▓▓░░
    0x09, //  ▓▓▓▓░░▓░
    0x09, //  ▓▓▓▓░░▓░
    0x06, //  ▓▓▓▓▓▓░░
    0x00, //  ▓▓▓▓▓▓▓▓
};




// Frame buffer for the display

static uint8_t ssd1306_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

const uint8_t font_5x7[] = {
    // Space ' ' (ASCII 32)
    0x00, 0x00, 0x00, 0x00, 0x00, 
    // '!' (ASCII 33)
    0x00, 0x00, 0x5F, 0x00, 0x00, 
    // '"' (ASCII 34)
    0x00, 0x07, 0x00, 0x07, 0x00, 
    // '#' (ASCII 35)
    0x14, 0x7F, 0x14, 0x7F, 0x14, 
    // '$' (ASCII 36)
    0x24, 0x2A, 0x7F, 0x2A, 0x12, 
    // '%' (ASCII 37)
    0x23, 0x13, 0x08, 0x64, 0x62, 
    // '&' (ASCII 38)
    0x36, 0x49, 0x55, 0x22, 0x50, 
    // ''' (ASCII 39)
    0x00, 0x05, 0x03, 0x00, 0x00, 
    // '(' (ASCII 40)
    0x00, 0x1C, 0x22, 0x41, 0x00, 
    // ')' (ASCII 41)
    0x00, 0x41, 0x22, 0x1C, 0x00, 
    // '*' (ASCII 42)
    0x14, 0x08, 0x3E, 0x08, 0x14, 
    // '+' (ASCII 43)
    0x08, 0x08, 0x3E, 0x08, 0x08, 
    // ',' (ASCII 44)
    0x00, 0x50, 0x30, 0x00, 0x00, 
    // '-' (ASCII 45)
    0x08, 0x08, 0x08, 0x08, 0x08, 
    // '.' (ASCII 46)
    0x00, 0x60, 0x60, 0x00, 0x00, 
    // '/' (ASCII 47)
    0x20, 0x10, 0x08, 0x04, 0x02, 
    // '0' (ASCII 48)
    0x3E, 0x51, 0x49, 0x45, 0x3E, 
    // '1' (ASCII 49)
    0x00, 0x42, 0x7F, 0x40, 0x00, 
    // '2' (ASCII 50)
    0x42, 0x61, 0x51, 0x49, 0x46, 
    // '3' (ASCII 51)
    0x21, 0x41, 0x45, 0x4B, 0x31, 
    // '4' (ASCII 52)
    0x18, 0x14, 0x12, 0x7F, 0x10, 
    // '5' (ASCII 53)
    0x27, 0x45, 0x45, 0x45, 0x39, 
    // '6' (ASCII 54)
    0x3C, 0x4A, 0x49, 0x49, 0x30, 
    // '7' (ASCII 55)
    0x01, 0x71, 0x09, 0x05, 0x03, 
    // '8' (ASCII 56)
    0x36, 0x49, 0x49, 0x49, 0x36, 
    // '9' (ASCII 57)
    0x06, 0x49, 0x49, 0x29, 0x1E, 
    // ':' (ASCII 58)
    0x00, 0x36, 0x36, 0x00, 0x00, 
    // ';' (ASCII 59)
    0x00, 0x56, 0x36, 0x00, 0x00, 
    // '<' (ASCII 60)
    0x08, 0x14, 0x22, 0x41, 0x00, 
    // '=' (ASCII 61)
    0x14, 0x14, 0x14, 0x14, 0x14, 
    // '>' (ASCII 62)
    0x00, 0x41, 0x22, 0x14, 0x08, 
    // '?' (ASCII 63)
    0x02, 0x01, 0x51, 0x09, 0x06, 
    // '@' (ASCII 64)
    0x32, 0x49, 0x79, 0x41, 0x3E, 
    // 'A' (ASCII 65)
    0x7E, 0x11, 0x11, 0x11, 0x7E, 
    // 'B' (ASCII 66)
    0x7F, 0x49, 0x49, 0x49, 0x36, 
    // 'C' (ASCII 67)
    0x3E, 0x41, 0x41, 0x41, 0x22, 
    // 'D' (ASCII 68)
    0x7F, 0x41, 0x41, 0x22, 0x1C, 
    // 'E' (ASCII 69)
    0x7F, 0x49, 0x49, 0x49, 0x41, 
    // 'F' (ASCII 70)
    0x7F, 0x09, 0x09, 0x09, 0x01, 
    // 'G' (ASCII 71)
    0x3E, 0x41, 0x49, 0x49, 0x7A, 
    // 'H' (ASCII 72)
    0x7F, 0x08, 0x08, 0x08, 0x7F, 
    // 'I' (ASCII 73)
    0x00, 0x41, 0x7F, 0x41, 0x00, 
    // 'J' (ASCII 74)
    0x20, 0x40, 0x41, 0x3F, 0x01, 
    // 'K' (ASCII 75)
    0x7F, 0x08, 0x14, 0x22, 0x41, 
    // 'L' (ASCII 76)
    0x7F, 0x40, 0x40, 0x40, 0x40, 
    // 'M' (ASCII 77)
    0x7F, 0x02, 0x04, 0x02, 0x7F, 
    // 'N' (ASCII 78)
    0x7F, 0x04, 0x08, 0x10, 0x7F, 
    // 'O' (ASCII 79)
    0x3E, 0x41, 0x41, 0x41, 0x3E, 
    // 'P' (ASCII 80)
    0x7F, 0x09, 0x09, 0x09, 0x06, 
    // 'Q' (ASCII 81)
    0x3E, 0x41, 0x51, 0x21, 0x5E, 
    // 'R' (ASCII 82)
    0x7F, 0x09, 0x19, 0x29, 0x46, 
    // 'S' (ASCII 83)
    0x46, 0x49, 0x49, 0x49, 0x31, 
    // 'T' (ASCII 84)
    0x01, 0x01, 0x7F, 0x01, 0x01, 
    // 'U' (ASCII 85)
    0x3F, 0x40, 0x40, 0x40, 0x3F, 
    // 'V' (ASCII 86)
    0x1F, 0x20, 0x40, 0x20, 0x1F, 
    // 'W' (ASCII 87)
    0x7F, 0x20, 0x18, 0x20, 0x7F, 
    // 'X' (ASCII 88)
    0x63, 0x14, 0x08, 0x14, 0x63, 
    // 'Y' (ASCII 89)
    0x03, 0x04, 0x78, 0x04, 0x03, 
    // 'Z' (ASCII 90)
    0x61, 0x51, 0x49, 0x45, 0x43, 
    // '[' (ASCII 91)
    0x00, 0x7F, 0x41, 0x41, 0x00, 
    // '\' (ASCII 92)
    0x02, 0x04, 0x08, 0x10, 0x20, 
    // ']' (ASCII 93)
    0x00, 0x41, 0x41, 0x7F, 0x00, 
    // '^' (ASCII 94)
    0x04, 0x02, 0x01, 0x02, 0x04, 
    // '_' (ASCII 95)
    0x40, 0x40, 0x40, 0x40, 0x40, 
    // '`' (ASCII 96)
    0x00, 0x01, 0x02, 0x04, 0x00, 
    // 'a' (ASCII 97)
    0x20, 0x54, 0x54, 0x54, 0x78, 
    // 'b' (ASCII 98)
    0x7F, 0x48, 0x44, 0x44, 0x38, 
    // 'c' (ASCII 99)
    0x38, 0x44, 0x44, 0x44, 0x20, 
    // 'd' (ASCII 100)
    0x38, 0x44, 0x44, 0x48, 0x7F, 
    // 'e' (ASCII 101)
    0x38, 0x54, 0x54, 0x54, 0x18, 
    // 'f' (ASCII 102)
    0x08, 0x7E, 0x09, 0x01, 0x02, 
    // 'g' (ASCII 103)
    0x08, 0x14, 0x54, 0x54, 0x3C, 
    // 'h' (ASCII 104)
    0x7F, 0x08, 0x04, 0x04, 0x78, 
    // 'i' (ASCII 105)
    0x00, 0x44, 0x7D, 0x40, 0x00, 
    // 'j' (ASCII 106)
    0x20, 0x40, 0x44, 0x3D, 0x00, 
    // 'k' (ASCII 107)
    0x7F, 0x10, 0x28, 0x44, 0x00, 
    // 'l' (ASCII 108)
    0x00, 0x41, 0x7F, 0x40, 0x00, 
    // 'm' (ASCII 109)
    0x7C, 0x04, 0x18, 0x04, 0x78, 
    // 'n' (ASCII 110)
    0x7C, 0x08, 0x04, 0x04, 0x78, 
    // 'o' (ASCII 111)
    0x38, 0x44, 0x44, 0x44, 0x38, 
    // 'p' (ASCII 112)
    0x7C, 0x14, 0x14, 0x14, 0x08, 
    // 'q' (ASCII 113)
    0x08, 0x14, 0x14, 0x18, 0x7C, 
    // 'r' (ASCII 114)
    0x7C, 0x08, 0x04, 0x04, 0x08, 
    // 's' (ASCII 115)
    0x48, 0x54, 0x54, 0x54, 0x20, 
    // 't' (ASCII 116)
    0x04, 0x3F, 0x44, 0x40, 0x20, 
    // 'u' (ASCII 117)
    0x3C, 0x40, 0x40, 0x20, 0x7C, 
    // 'v' (ASCII 118)
    0x1C, 0x20, 0x40, 0x20, 0x1C, 
    // 'w' (ASCII 119)
    0x3C, 0x40, 0x30, 0x40, 0x3C, 
    // 'x' (ASCII 120)
    0x44, 0x28, 0x10, 0x28, 0x44, 
    // 'y' (ASCII 121)
    0x0C, 0x50, 0x50, 0x50, 0x3C, 
    // 'z' (ASCII 122)
    0x44, 0x64, 0x54, 0x4C, 0x44, 
    // '{' (ASCII 123)
    0x00, 0x08, 0x36, 0x41, 0x00, 
    // '|' (ASCII 124)
    0x00, 0x00, 0x7F, 0x00, 0x00, 
    // '}' (ASCII 125)
    0x00, 0x41, 0x36, 0x08, 0x00, 
    // '~' (ASCII 126)
    0x08, 0x04, 0x08, 0x10, 0x08

};


// Send a command to the SSD1306
static void ssd1306_send_command(uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Command mode
    i2c_master_write_byte(cmd, command, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
}


// Send data to the SSD1306
static void ssd1306_send_data(uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true); // Data mode
    i2c_master_write(cmd, data, len, true); // Pass the `cmd` handle here
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
}


// Initialize the SSD1306 display
void ssd1306_init(void) {
	 esp_err_t ret;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
  ret =  i2c_param_config(I2C_MASTER_NUM, &conf);
    
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return;
    }
  ret =  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
    }

    ssd1306_send_command(0xAE); // Display OFF
    ssd1306_send_command(0x20); // Set Memory Addressing Mode
    ssd1306_send_command(0x00); // Horizontal addressing mode
    ssd1306_send_command(0xB0); // Set page start address
    ssd1306_send_command(0xC8); // Set COM output scan direction
    ssd1306_send_command(0x00); // Set low column address
    ssd1306_send_command(0x10); // Set high column address
    ssd1306_send_command(0x40); // Set start line address
    ssd1306_send_command(0x81); // Set contrast control
    ssd1306_send_command(0xFF); // Max contrast
    ssd1306_send_command(0xA1); // Set segment re-map
    ssd1306_send_command(0xA6); // Normal display
    ssd1306_send_command(0xA8); // Set multiplex ratio
    ssd1306_send_command(0x3F); // 1/64 duty
    ssd1306_send_command(0xD3); // Set display offset
    ssd1306_send_command(0x00);
    ssd1306_send_command(0xD5); // Set display clock divide ratio/oscillator frequency
    ssd1306_send_command(0x80);
    ssd1306_send_command(0xD9); // Set pre-charge period
    ssd1306_send_command(0xF1);
    ssd1306_send_command(0xDA); // Set COM pins hardware configuration
    ssd1306_send_command(0x12);
    ssd1306_send_command(0xDB); // Set VCOMH deselect level
    ssd1306_send_command(0x20);
    ssd1306_send_command(0x8D); // Enable charge pump
    ssd1306_send_command(0x14);
    ssd1306_send_command(0xAF); // Display ON
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
}

// Clear the display
void ssd1306_clear(void) {
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
}

// Refresh the display with the buffer content
void ssd1306_display(void) {
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_send_command(0xB0 + page); // Set page address
        ssd1306_send_command(0x00);       // Set low column address
        ssd1306_send_command(0x10);       // Set high column address
        ssd1306_send_data(&ssd1306_buffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
    }
}

// Draw a pixel at (x, y)
void ssd1306_draw_pixel(int x, int y, bool color) {
    // Check boundaries to avoid memory corruption
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }

    // Calculate the byte in the frame buffer
    uint16_t byte_index = x + (y / 8) * SSD1306_WIDTH;

    // Set or clear the bit corresponding to the pixel
    if (color) {
        ssd1306_buffer[byte_index] |= (1 << (y % 8)); // Set bit
    } else {
        ssd1306_buffer[byte_index] &= ~(1 << (y % 8)); // Clear bit
    }
}


// Draw a character (simple 5x7 font)
void ssd1306_draw_char(int x, int y, char c) {
    if (c < 32 || c > 126) return; // Ignore non-printable characters

    int font_width = 5;  // Width of each character
    int font_height = 7; // Height of each character
    int font_offset = (c - 32) * font_width; // Calculate font array offset

    for (int i = 0; i < font_width; i++) { // Iterate over columns
        uint8_t column = font_5x7[font_offset + i]; // Get column data
        for (int j = 0; j < font_height; j++) {    // Iterate over rows
            if (column & (1 << j)) {               // Check if pixel is set
                ssd1306_draw_pixel(x + i, y + j, true);
            }
        }
    }
}


// Draw a string
void ssd1306_draw_string(int x, int y, const char *str) {
    while (*str) {
        ssd1306_draw_char(x, y, *str);
        x += 6; // Advance to the next character (5 pixels + 1 for spacing)
        str++;
    }
}

void ssd1306_draw_scaled_char(int x, int y, char c, int scale) {
    if (c < 32 || c > 126) return; // Ignore non-printable characters

    int font_width = 5;  // Width of each character
    int font_height = 7; // Height of each character
    
    int font_offset = (c - 32) * font_width; // Calculate font array offset
       

    for (int i = 0; i < font_width; i++) { // Iterate over columns
        uint8_t column = font_5x7[font_offset + i]; // Get column data
        for (int j = 0; j < font_height; j++) {    // Iterate over rows
            if (column & (1 << j)) {               // Check if pixel is set
                for (int dx = 0; dx < scale; dx++) {
                    for (int dy = 0; dy < scale; dy++) {
                        ssd1306_draw_pixel(x + i * scale + dx, y + j * scale + dy, true);
                    }
                }
            }
        }
    }
}


void ssd1306_draw_scaled_string(int x, int y, const char *str, int scale) {
    while (*str) {
        ssd1306_draw_scaled_char(x, y, *str, scale);
        x += (5 * scale) + scale; // Advance to the next character
        str++;
    }
}

void ssd1306_draw_bitmap(int x, int y, const uint8_t *bitmap, int width, int height) {
    for (int i = 0; i < width; i++) {
        uint8_t column = bitmap[i];
        for (int j = 0; j < height; j++) {
            if (column & (1 << j)) {
                ssd1306_draw_pixel(x + i, y + j, true);
            }
        }
    }
}

void ssd1306_draw_scaled_bitmap(int x, int y, const uint8_t *bitmap, int width, int height, int scale) {
    for (int i = 0; i < width; i++) { // Iterate over columns
        uint8_t column = bitmap[i]; // Extract column data
        for (int j = 0; j < height; j++) { // Iterate over rows
            if (column & (1 << j)) { // Check if the pixel is set
                for (int dx = 0; dx < scale; dx++) {
                    for (int dy = 0; dy < scale; dy++) {
                        ssd1306_draw_pixel(x + i * scale + dx, y + j * scale + dy, true);
                    }
                }
            }
        }
    }
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

void toggle_invert_display(bool invert) {
    ssd1306_send_command(invert ? 0xA7 : 0xA6); // 0xA7: Inverted, 0xA6: Normal
}


// Distance Task
void distance_task(void *pvParameters) {
    float distance = 0.0;

    while (1) {
        // Trigger the ultrasonic sensor
        trigger_ultrasonic();
        vTaskDelay(pdMS_TO_TICKS(60)); // Allow time for the echo to complete

        // Calculate the distance
        distance = calculate_distance();
        if (distance == -1.0) {
            continue; // Skip if invalid
        }

        ESP_LOGI(TAG, "Raw Distance: %.2f cm", distance);

        if (distance > 0 && distance < 18.0) {
            strncpy(dist_display_status, "FULL!!", sizeof(dist_display_status));
            strncpy(dist_display, "0", sizeof(dist_display));
            distance_alert_active = false; // No alert for below min
        } else if (distance >= 18.0 && distance <= 100.0) {
			
			snprintf(dist_display, sizeof(dist_display), "-%d", (int)(distance- 18));
			
            //snprintf(dist_display, sizeof(dist_display), "DIST: %.2f CM", distance);
            
           // strncpy(dist_display_status, "GOOD", sizeof(dist_display_status));
            // Activate alert if distance > 27.0
            
            if((int)distance > 23)
            {
			distance_alert_active = true;	
			strncpy(dist_display_status, "LOW!!", sizeof(dist_display_status));
			}else 
			{
				distance_alert_active = false;
			strncpy(dist_display_status, "GOOD", sizeof(dist_display_status));
			}
        } else {
            //strncpy(dist_display_status, "TOO LOW", sizeof(dist_display_status));
            distance_alert_active = false; // No alert for out-of-range
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Prevent excessive updates
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

            temperature_alert_active = (temp >= 30.0);
        } else {
            ESP_LOGE(TAG, "Invalid temperature reading!");
            strncpy(temp_display, "N/A", sizeof(temp_display));
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



void print_temp_oled(void)
{
	 ssd1306_clear();
	 ssd1306_draw_scaled_string(30, 0, &"Temperature", 1);
    ssd1306_draw_scaled_string(0, 20, &temp_display, 5);
    ssd1306_draw_scaled_bitmap(80, 20, degree_symbol, 5, 7,2); // Degree symbol at (30, 2)
    ssd1306_draw_scaled_string(60, 50, &temp_fractional, 2);
    ssd1306_draw_scaled_string(100, 20, "C", 5);
    ssd1306_display();
	
}


void print_Water_level_oled(void)
{
	 ssd1306_clear();
	ssd1306_draw_scaled_string(30, 0, &"Water Level", 1);
	ssd1306_draw_scaled_string(0, 20, &dist_display_status, 4);
    ssd1306_draw_scaled_string(60, 50, &dist_display, 2);
    ssd1306_draw_scaled_string(90, 50, "cm", 2);
    ssd1306_display();
	
}

void Print_logo_diplay()
{
	 ssd1306_clear();
	 ssd1306_draw_scaled_string(30, 0, &"ENV MONITOR", 1);
	 ssd1306_draw_scaled_string(10, 15, &"Yoogi", 4);
	     ssd1306_draw_scaled_string(70, 50, &"By Cuaseve", 1);
	 ssd1306_display();
	
	
}


void oled_refresh_task(void *pvParameters) {
	static char count= 0;
    while (1) {
				
          if(count >=0 && count  <2)
          {
			  if (temperature_alert_active)
			  {
				  toggle_invert_display(true);
			  }
			  else 
			  {
				  toggle_invert_display(false);
				  
			}
			  print_temp_oled();
			  
			  count ++;
			  
		  }
		  else if(count >=2 && count  <4)
		  {
			  
			   if (distance_alert_active)
			  {
				  toggle_invert_display(true);
			  }
			  else 
			  {
				  toggle_invert_display(false);
				  
			}
		  print_Water_level_oled();
		  
		  count ++;
		  if(count == 4){
		  count = 0;}
		  
		  
		  }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Refresh display every 20 ms
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
    esp_log_level_set("ENV_MONITOR", ESP_LOG_INFO);
	#else
	esp_log_level_set("ENV_MONITOR", ESP_LOG_NONE);
	#endif
       
     Print_logo_diplay();
     sys_delay_ms(5000);
       
    // Create FreeRTOS Tasks
    xTaskCreate(temperature_task, "Temperature Task", 2048, NULL, 1, NULL);
    xTaskCreate(distance_task, "Distance Task", 2048, NULL, 2, NULL);
    xTaskCreate(buzzer_task, "Buzzer Task", 2048, NULL, 1, NULL);
    xTaskCreate(oled_refresh_task, "OLED Refresh Task", 2048, NULL, 1, NULL);
}