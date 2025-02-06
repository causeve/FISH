/*
 * main.h
 *
 *  Created on: 29-Dec-2024
 *      Author: ilang
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

#define LOGGING_ENABLED 1  // 0-disable 1-enable

#if LOGGING_ENABLED == 1
    // Logging is enabled
    #define ESP_LOGE(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format, ##__VA_ARGS__)
    #define ESP_LOGW(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format, ##__VA_ARGS__)
    #define ESP_LOGI(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format, ##__VA_ARGS__)
    #define ESP_LOGD(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   tag, format, ##__VA_ARGS__)
    #define ESP_LOGV(tag, format, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)
#else
    // Logging is disabled
    #define ESP_LOGE(tag, format, ...) ((void)0)
    #define ESP_LOGW(tag, format, ...) ((void)0)
    #define ESP_LOGI(tag, format, ...) ((void)0)
    #define ESP_LOGD(tag, format, ...) ((void)0)
    #define ESP_LOGV(tag, format, ...) ((void)0)
#endif

//#define LONG_PRESS_THRESHOLD_MS 4000 // Long press: 4 seconds

#define DEBOUNCE_DELAY_MS 50 // 50 ms debounce time


typedef enum {
    NORMAL_MODE,
    CALIBRATION_MODE,
    WATER_FILL_MODE
} SystemState;

extern SystemState current_state;

void water_level_low();
void water_level_restored();


#endif /* MAIN_MAIN_H_ */
