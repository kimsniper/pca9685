#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*  PCA9685 components  */
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

static const char *TAG = "example_usage";

#define LED_OUTPUT_PIN_0      0
#define LED_OUTPUT_PIN_1      1
#define LED_OUTPUT_PIN_2      2
#define LED_OUTPUT_PIN_3      3
#define LED_OUTPUT_PIN_4      4
#define LED_OUTPUT_PIN_5      5
#define LED_OUTPUT_PIN_6      6
#define LED_OUTPUT_PIN_7      7
#define LED_OUTPUT_PIN_8      8
#define LED_OUTPUT_PIN_9      9
#define LED_OUTPUT_PIN_10     10
#define LED_OUTPUT_PIN_11     11
#define LED_OUTPUT_PIN_12     12
#define LED_OUTPUT_PIN_13     13
#define LED_OUTPUT_PIN_14     14
#define LED_OUTPUT_PIN_15     15

void servo_drive()
{
    ESP_LOGI(TAG, "Servo drive: 1 ms duty cycle");
    pca9685_i2c_led_pwm_set(LED_OUTPUT_PIN_1, 5, 0);
}

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

    err += pca9685_i2c_write_pre_scale(50, 25000000); /* Setting frequency to 50 Hz (200ms) */
    ESP_LOGI(TAG, "Frequency Setting: %s", err == PCA9685_OK ? "Successful" : "Failed");

    //pca9685_i2c_reset();

    uint16_t frequency;
    pca9685_i2c_read_pre_scale(&frequency, 25000000);
    ESP_LOGI(TAG, "Frequency Reading: %d", frequency);

    pca9685_i2c_sleep_mode(PCA9685_MODE_NORMAL);
    pca9685_i2c_autoincrement(PCA9685_AUTOINCR_ON);

    pca9685_i2c_read_mode_1(&mode);
    ESP_LOGW(TAG, "Mode 1 Setting: %d", mode);

    pca9685_i2c_read_mode_2(&mode);
    ESP_LOGW(TAG, "Mode 2 Setting: %d", mode);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "PCA9685 initialization successful");
        while(1)
        {
            
            pca9685_i2c_led_pwm_set(LED_OUTPUT_PIN_1, 5, 0);
            vTaskDelay(pdMS_TO_TICKS(5000));
            ESP_LOGI(TAG, "Servo drive: 2 ms duty cycle");
            pca9685_i2c_led_pwm_set(LED_OUTPUT_PIN_1, 10, 0);
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
    else{
        ESP_LOGE(TAG, "PCA9685 initialization failed!");
    }
}