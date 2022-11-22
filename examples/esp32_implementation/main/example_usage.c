#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//PCA9685 components
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

static const char *TAG = "example_usage";

#include "driver/i2c.h"

static uint8_t i2c_slave_knock(uint8_t i2c_port, uint8_t slave_addr) {
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return err == ESP_OK;
}

static void i2c_scan(void) {
    uint8_t slave_count = 0;
    vTaskDelay(500 / portTICK_PERIOD_MS);
    for (int slave_addr = 0; slave_addr < 127; slave_addr++) {
        if (i2c_slave_knock(0, slave_addr)) {
            ESP_LOGI(TAG, "master %d slave_%d_addr %02X", 0, slave_count,
                     slave_addr);
            slave_count++;
        }
    }

    ESP_LOGI(TAG, "Slave Count %d", slave_count);
}

void app_main(void)
{
    esp_err_t err = ESP_OK;
    uint8_t id = 0;

    pca9685_i2c_hal_init();

    i2c_scan();

    err += pca9685_i2c_reset();
    ESP_LOGI(TAG, "Device reset: %s", err == PCA9685_OK ? "Successful" : "Failed");

    err += pca9685_i2c_write_pre_scale(100);
    ESP_LOGI(TAG, "Frequency Setting: %s", err == PCA9685_OK ? "Successful" : "Failed");

    uint8_t frequency;
    pca9685_i2c_read_pre_scale(&frequency);
    ESP_LOGI(TAG, "Frequency: %d", frequency);

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