#include "flashnvm.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "oled.h"
#include <main.h>
#include "esp_log.h"
#include "distance.h"







void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}



void save_base_distance(float base_distance) {
    int base_distance_int = (int)base_distance; // Convert to integer
    nvs_handle_t my_handle;

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        err = nvs_set_i32(my_handle, "base_dist", base_distance_int);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Base distance saved: %d cm", base_distance_int);
            nvs_commit(my_handle); // Commit changes
        } else {
            ESP_LOGE("NVS", "Failed to save base distance!");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("NVS", "Error opening NVS!");
    }
}


int load_base_distance() {
    int base_distance_int = -1; // Default value
    nvs_handle_t my_handle;

    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        err = nvs_get_i32(my_handle, "base_dist", &base_distance_int);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Base distance loaded: %d cm", base_distance_int);
        } else {
            ESP_LOGW("NVS", "No base distance found in NVS.");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("NVS", "Error opening NVS!");
    }

    return base_distance_int; // Convert to float if needed
}

void save_calibration_status(bool is_calibrated) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        err = nvs_set_u8(my_handle, "is_calibrated", is_calibrated ? 1 : 0);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Calibration status saved: %s", is_calibrated ? "true" : "false");
            nvs_commit(my_handle);
        } else {
            ESP_LOGE("NVS", "Failed to save calibration status!");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("NVS", "Error opening NVS!");
    }
}

bool load_calibration_status() {
    nvs_handle_t my_handle;
    uint8_t is_calibrated = 0; // Default: not calibrated
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        err = nvs_get_u8(my_handle, "is_calibrated", &is_calibrated);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Calibration status loaded: %s", is_calibrated ? "true" : "false");
        } else {
            ESP_LOGW("NVS", "Calibration status not found.");
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE("NVS", "Error opening NVS!");
    }
    return is_calibrated == 1;
}

void check_first_time_calibration() {
    if (load_calibration_status()) {
        ESP_LOGI("System", "Calibration found. Starting in Normal Mode.");
        
        nvm_base_distance =load_base_distance();
        
         base_distance_diplay_oled( nvm_base_distance);       
          current_state = NORMAL_MODE; 
       
    } else {
        ESP_LOGI("System", "No calibration found. Entering Calibration Mode.");
        current_state = CALIBRATION_MODE;
    }
}

void clear_calibration_data() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        // Erase calibration keys
        err = nvs_erase_key(my_handle, "is_calibrated");
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Cleared is_calibrated key.");
        } else {
            ESP_LOGW("NVS", "Failed to clear is_calibrated key.");
        }

        err = nvs_erase_key(my_handle, "base_dist");
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Cleared base_dist key.");
        } else {
            ESP_LOGW("NVS", "Failed to clear base_dist key.");
        }

        // Commit changes
        err = nvs_commit(my_handle);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Calibration data cleared successfully.");
        } else {
            ESP_LOGE("NVS", "Failed to commit changes.");
        }

        nvs_close(my_handle);
    } else {
        ESP_LOGE("NVS", "Error opening NVS!");
    }
}
