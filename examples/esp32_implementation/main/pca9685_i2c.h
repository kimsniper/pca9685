/*
 * Copyright (c) 2022, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MAIN_PCA9685_I2C
#define MAIN_PCA9685_I2C

#ifdef __cplusplus
extern "C" {
#endif

#include "pca9685_i2c_hal.h" 

typedef struct{
    uint8_t addr_no;
    uint8_t address;
} pca9685_subaddr_t;

typedef struct{
    uint8_t restart : 1;
    uint8_t extclk : 1;
    uint8_t auto_increment : 1;
    uint8_t sleep : 1;
    uint8_t sub_1 : 1;
    uint8_t sub_2 : 1;
    uint8_t sub_3 : 1;
    uint8_t all_call : 1;
} pca9685_mode1_t;

typedef struct{
    uint8_t invrt : 1;
    uint8_t och : 1;
    uint8_t outdrv : 1;
    uint8_t outne : 2;
} pca9685_mode2_t;

typedef struct{
    uint8_t led_no;
    uint16_t led_ON;
    uint16_t led_OFF;
} pca9685_led_pwm_t;

typedef enum{
    PCA9685_LED_OFF,
    PCA9685_LED_ON,
} pca9685_led_state_t;

typedef struct{
    uint8_t led_no;
    pca9685_led_state_t state;
} pca9685_led_t;

/**
 * @brief PCA9685 device address.
 * @details PCA9685 I2C slave address.
 */
#define I2C_ADDRESS_PCA9685             0x76

/**
 * @brief PCA9685 command code registers.
 * @details R/W Command registers
 */
#define REG_MODE_1                      0x00
#define REG_MODE_2                      0x01
#define REG_ALLCALLADR                  0x05
#define REG_ALL_LED_ON                  0xFA
#define REG_ALL_LED_OFF                 0xFC
#define REG_PRE_SCALE                   0xFE
#define REG_TEST_MODE                   0xFF

/**
 * @brief PCA9685 MODE_1 register bit description.
 * @details R/W Command registers
 */
#define MODE_1_RESTART_EN               0x01
#define MODE_1_EXT_CLOCK                0x01
#define MODE_1_INT_CLOCK                0x00
#define MODE_1_AUTO_INCREMENT_EN        0x01
#define MODE_1_AUTO_INCREMENT_DIS       0x00
#define MODE_1_SLEEP_NORMAL             0x01
#define MODE_1_SLEEP_LOW_POWER          0x00
#define MODE_1_SUB_1_RESPOND            0x01
#define MODE_1_SUB_2_RESPOND            0x01
#define MODE_1_SUB_3_RESPOND            0x01
#define MODE_1_ALLCALL_RESPOND          0x01

/**
 * @brief PCA9685 MODE_2 register bit description.
 * @details R/W Command registers
 */
#define MODE_2_INVRT                    0x01
#define MODE_2_NOT_INVRT                0x00
#define MODE_2_OCH_STOP                 0x01
#define MODE_2_OCH_ACK                  0x00
#define MODE_2_OUTDRV_TOTEM             0x01
#define MODE_2_OUTDRV_OPEN              0x00
#define MODE_2_OUTNE_LEDEN_LOW          0x00
#define MODE_2_OUTNE_LEDEN_HIGH         0x01
#define MODE_2_OUTNE_LEDEN_HIGHZ        0x02

/**
 * @brief PCA9685 macros
 * @details Other macros
 */
#define LED_OFFSET_ADR                  0x06
#define SUBADR_OFFSET_ADR               0x01
#define STAB_TIME                       1     //Stabilization time (ms)


/**
 * @brief PCA9685 calibration setting.
 * @details Set global parameter calibration values.
 */
pca9685_err_t pca9685_i2c_read_mode_1(uint8_t *mode);

/**
 * @brief PCA9685 calibration setting.
 * @details Set global parameter calibration values.
 */
pca9685_err_t pca9685_i2c_write_mode_1(pca9685_mode1_t cfg);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_PCA9685_I2C */
