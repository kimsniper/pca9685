#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*  PCA9685 components  */
#include "pca9685_i2c.h"
#include "pca9685_i2c_hal.h"

static const char *TAG = "example_usage";

#define PCA9685_OSC_CLK         25000000
#define SERVO_PWM_FREQ          50

#define SERVO_OUTPUT_PIN_0      0
#define SERVO_OUTPUT_PIN_1      1
#define SERVO_OUTPUT_PIN_2      2
#define SERVO_OUTPUT_PIN_3      3
#define SERVO_OUTPUT_PIN_4      4
#define SERVO_OUTPUT_PIN_5      5
#define SERVO_OUTPUT_PIN_6      6
#define SERVO_OUTPUT_PIN_7      7
#define SERVO_OUTPUT_PIN_8      8
#define SERVO_OUTPUT_PIN_9      9
#define SERVO_OUTPUT_PIN_10     10
#define SERVO_OUTPUT_PIN_11     11
#define SERVO_OUTPUT_PIN_12     12
#define SERVO_OUTPUT_PIN_13     13
#define SERVO_OUTPUT_PIN_14     14
#define SERVO_OUTPUT_PIN_15     15

void servo_pwm_drive(uint8_t servo_no, float d_cycle_ms)
{
    if(pca9685_i2c_led_pwm_set(servo_no, (d_cycle_ms/(1000/(float)SERVO_PWM_FREQ))*100, 0) == PCA9685_OK)
        ESP_LOGI(TAG, "Servo drive: Duty cycle -> %.01fms, Duration -> %.01fms", d_cycle_ms, (1000/(float)SERVO_PWM_FREQ));
    else
        ESP_LOGE(TAG, "Servo drive: ERROR!");
}

void app_main(void)
{
    esp_err_t err = ESP_OK;

    pca9685_i2c_hal_init();

    err += pca9685_i2c_reset();
    ESP_LOGI(TAG, "Device reset: %s", err == PCA9685_OK ? "Successful" : "Failed");

    err += pca9685_i2c_write_pre_scale(SERVO_PWM_FREQ, PCA9685_OSC_CLK); /* Setting frequency to 50 Hz (200ms) */
    ESP_LOGI(TAG, "Frequency Setting: %s", err == PCA9685_OK ? "Successful" : "Failed");

    pca9685_i2c_sleep_mode(PCA9685_MODE_NORMAL);
    pca9685_i2c_autoincrement(PCA9685_AUTOINCR_ON);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "PCA9685 initialization successful");
        while(1)
        {
            servo_pwm_drive(SERVO_OUTPUT_PIN_1, 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
            servo_pwm_drive(SERVO_OUTPUT_PIN_1, 2);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    else{
        ESP_LOGE(TAG, "PCA9685 initialization failed!");
    }
}