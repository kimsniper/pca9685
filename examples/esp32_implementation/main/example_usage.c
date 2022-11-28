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

/*  Select I2C address to be used. ()
    I2C Address options:
    - Default address - 0x40
    - Sub addresses 1-3
    - All call address
*/
#define I2C_DEFAULT_ADDRESS     0
#define I2C_SUB_ADDRESS_1       1
#define I2C_SUB_ADDRESS_2       2
#define I2C_SUB_ADDRESS_3       3
#define I2C_ALLCALL_ADDRESS     4

#define USE_I2C_ADDRESS         I2C_DEFAULT_ADDRESS 

pca9685_dev_t pca9685_1;

void servo_pwm_drive(pca9685_dev_t dev, uint8_t servo_no, float d_cycle_ms)
{
    if(pca9685_i2c_led_pwm_set(dev, servo_no, (d_cycle_ms/(1000/(float)SERVO_PWM_FREQ))*100, 0) == PCA9685_OK)
        ESP_LOGI(TAG, "Servo drive: Duty cycle -> %.01fms, Duration -> %.01fms", d_cycle_ms, (1000/(float)SERVO_PWM_FREQ));
    else
        ESP_LOGE(TAG, "Servo drive: ERROR!");
}

void app_main(void)
{
    esp_err_t err = ESP_OK;

    pca9685_i2c_hal_init();

    err += pca9685_i2c_reset(); /* Perform all device reset */
    ESP_LOGI(TAG, "Device reset: %s", err == PCA9685_OK ? "Successful" : "Failed");

    /* Register i2c device*/
    pca9685_i2c_register( &pca9685_1, 
                          I2C_ADDRESS_PCA9685,
                          I2C_ALL_CALL_ADDRESS_PCA9685,
                          I2C_SUB_ADDRESS_1_PCA9685,
                          I2C_SUB_ADDRESS_2_PCA9685,
                          I2C_SUB_ADDRESS_3_PCA9685); /* Use default values */

#if USE_I2C_ADDRESS == I2C_DEFAULT_ADDRESS

    //Already in default I2C address

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_1

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_1, PCA9685_ADDR_RESPOND); /* Enable subaddres 1 response*/
    ESP_LOGI(TAG, "Enable subaddress 1: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_1; /* Assign i2c_addr to subaddress 1 */

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_2

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_2, PCA9685_ADDR_RESPOND); /* Enable subaddres 2 response*/
    ESP_LOGI(TAG, "Enable subaddress 2: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_2; /* Assign i2c_addr to subaddress 2 */

#elif USE_I2C_ADDRESS == I2C_SUB_ADDRESS_3

    err += pca9685_i2c_sub_addr_resp(pca9685_1, PCA9685_SUB_ADDR_3, PCA9685_ADDR_RESPOND); /* Enable subaddres 3 response*/
    ESP_LOGI(TAG, "Enable subaddress 3: %s", err == PCA9685_OK ? "Successful" : "Failed");
    pca9685_1.i2c_addr =  pca9685_1.sub_addr_3; /* Assign i2c_addr to subaddress 3 */

#elif USE_I2C_ADDRESS == I2C_ALLCALL_ADDRESS

    /* All call address response is already enabled by default */
    pca9685_1.i2c_addr =  pca9685_1.allcall_addr;

#endif

    ESP_LOGI(TAG, "i2c_addr: 0x%02x", pca9685_1.i2c_addr);

    err += pca9685_i2c_write_pre_scale(pca9685_1, SERVO_PWM_FREQ, PCA9685_OSC_CLK); /* Setting frequency to 50 Hz (200ms) */
    ESP_LOGI(TAG, "Frequency Setting: %s", err == PCA9685_OK ? "Successful" : "Failed");

    pca9685_i2c_sleep_mode(pca9685_1, PCA9685_MODE_NORMAL);
    pca9685_i2c_autoincrement(pca9685_1, PCA9685_AUTOINCR_ON); /* Register increment every read/write */

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "PCA9685 initialization successful");
        while(1)
        {
            servo_pwm_drive(pca9685_1, SERVO_OUTPUT_PIN_1, 1); /* Drive servo at 1 ms */
            pca9685_i2c_hal_ms_delay(2000);
            servo_pwm_drive(pca9685_1, SERVO_OUTPUT_PIN_1, 2); /* Drive servo at 2 ms */
            pca9685_i2c_hal_ms_delay(2000);
        }
    }
    else{
        ESP_LOGE(TAG, "PCA9685 initialization failed!");
    }
}
