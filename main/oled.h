#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>



// OLED Configuration
#define OLED_ADDR 0x3C             // OLED I2C address
// Define display width and height
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64


// I2c  Configuration
#define I2C_MASTER_SCL_IO 22              // SCL pin
#define I2C_MASTER_SDA_IO 21              // SDA pin
#define I2C_MASTER_NUM I2C_NUM_0          // I2C port number
#define I2C_MASTER_FREQ_HZ 400000         // I2C frequency


// Function Prototypes
void ssd1306_init(void);
void Print_logo_diplay(void);
void ssd1306_clear(void);
void print_Water_level_oled(void);
void print_temp_oled(void);
void toggle_invert_display(bool invert);
void CalibMode_Water_level_oled(void);
void base_distance_diplay_oled(float base_distance);

void Clear_NVM_Display_oled(void);
void oled_refresh_task(void *pvParameters);
#endif // OLED_DISPLAY_H
