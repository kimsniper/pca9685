#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*  PCA9685 components  */
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

static const char *TAG = "example_usage";

void app_main(void)
{
    esp_err_t err = ESP_OK;
    uint8_t id = 0;

    pca9685_i2c_hal_init();

    err += pca9685_i2c_reset();
    ESP_LOGI(TAG, "Device reset: %s", err == PCA9685_OK ? "Successful" : "Failed");

    uint8_t mode;
    pca9685_i2c_read_mode_1(&mode);
    ESP_LOGW(TAG, "Mode 1 Setting: %d", mode);

    err += pca9685_i2c_write_pre_scale(50, 25000000);
    ESP_LOGI(TAG, "Frequency Setting: %s", err == PCA9685_OK ? "Successful" : "Failed");

    //pca9685_i2c_reset();

    uint16_t frequency;
    pca9685_i2c_read_pre_scale(&frequency, 25000000);
    ESP_LOGI(TAG, "Frequency Reading: %d", frequency);

    pca9685_i2c_sleep_mode(PCA9685_MODE_NORMAL);
    pca9685_i2c_autoincrement(PCA9685_AUTOINCR_ON);

    pca9685_i2c_read_mode_1(&mode);
    ESP_LOGW(TAG, "Mode 1 Setting: %d", mode);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "PCA9685 initialization successful");
        while(1)
        {
            pca9685_i2c_all_led_pwm_set(5, 0);
            vTaskDelay(pdMS_TO_TICKS(5000));
            pca9685_i2c_all_led_pwm_set(10, 0);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    else{
        ESP_LOGE(TAG, "PCA9685 initialization failed!");
    }
}