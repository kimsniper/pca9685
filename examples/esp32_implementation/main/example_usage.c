#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//PCA9685 components
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

static const char *TAG = "example_usage";

void app_main(void)
{
    esp_err_t err = ESP_OK;
    uint8_t id = 0;

    pca9685_i2c_hal_init();

    err = pca9685_i2c_reset();
    if(err != ESP_OK) ESP_LOGE(TAG, "Error resetting the device!");

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "PCA9685 initialization successful");
        while(1)
        {
            
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    else{
        ESP_LOGE(TAG, "PCA9685 initialization failed!");
    }
}
