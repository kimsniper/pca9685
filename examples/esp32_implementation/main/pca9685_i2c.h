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

typedef enum{
    PCA9685_AUTOINCR_OFF = 0x00,
    PCA9685_AUTOINCR_ON  = 0x01,
} pca9685_auto_incr_t;

typedef enum{
    PCA9685_MODE_NORMAL = 0x00,
    PCA9685_MODE_SLEEP  = 0x01,
} pca9685_sleep_mode_t;

typedef enum{
    PCA9685_ADDR_NORESPOND = 0x00,
    PCA9685_ADDR_RESPOND = 0x01,
} pca9685_addr_resp_t;

typedef enum{
    PCA9685_SUB_ADDR_1 = 0x03,
    PCA9685_SUB_ADDR_2 = 0x02,
    PCA9685_SUB_ADDR_3 = 0x01,
} pca9685_subaddr_no_t;

typedef struct{
    uint16_t delay;
    uint16_t duty_cycle;
} pca9685_pwm_cycle_t;

typedef struct{
    uint8_t led_no;
    pca9685_pwm_cycle_t cycle;
} pca9685_led_pwm_t;

typedef enum{
    PCA9685_LED_OFF = 0x00,
    PCA9685_LED_ON  = 0x01,
} pca9685_led_state_t;

typedef struct{
    uint8_t led_no;
    pca9685_led_state_t state;
} pca9685_led_t;

typedef enum{
    PCA9685_OUTPUT_NOTINVERT = 0x00,
    PCA9685_OUTPUT_INVERT = 0x01,
} pca9685_output_invert_t;

typedef enum{
    PCA9685_CH_ONSTOP = 0x00,
    PCA9685_CH_ONACK = 0x01,
} pca9685_output_change_t;

typedef enum{
    PCA9685_OUTPUT_LOW = 0x00,
    PCA9685_OUTPUT_HIGH = 0x01,
    PCA9685_OUTPUT_HIGH_IMPEDANCE = 0x02,
} pca9685_output_not_enable_t;

typedef enum{
    PCA9685_OUTPUT_OPEN_DRAIN = 0x00,
    PCA9685_OUTPUT_TOTEM_POLE = 0x01,
} pca9685_output_drive_t;

typedef struct{
    pca9685_output_drive_t outdrv;
    pca9685_output_not_enable_t outne;
    pca9685_output_change_t och;
    pca9685_output_invert_t invrt;
} pca9685_output_t;

/**
 * @brief PCA9685 I2C slave address
 */
#define I2C_ADDRESS_PCA9685             0x40

/**
 * @brief PCA9685 I2C geeneral call address
 */
#define I2C_GEN_CALL_ADDRESS_PCA9685    0x00

/**
 * @brief PCA9685 R/W Command registers
 */
#define REG_RESET                       0x00
#define REG_MODE_1                      0x00
#define REG_MODE_2                      0x01
#define REG_ALLCALLADR                  0x05
#define REG_ALL_LED                     0xFA
#define REG_PRE_SCALE                   0xFE
#define REG_TEST_MODE                   0xFF

/**
 * @brief PCA9685 software reset command
 */
#define SWRST                           0x06

/**
 * @brief PCA9685 MODE_1 register bit description
 */
#define MODE_1_RESTART_EN               0x01
#define MODE_1_EXT_CLOCK                0x01
#define MODE_1_INT_CLOCK                0x00
#define MODE_1_AUTO_INCREMENT_EN        0x01
#define MODE_1_AUTO_INCREMENT_DIS       0x00
#define MODE_1_SLEEP_NORMAL             0x01
#define MODE_1_SLEEP_LOW_POWER          0x00

/**
 * @brief PCA9685 MODE_2 register bit description
 */
#define MODE_2_OCH_STOP                 0x01
#define MODE_2_OCH_ACK                  0x00
#define MODE_2_OUTDRV_TOTEM             0x01
#define MODE_2_OUTDRV_OPEN              0x00
#define MODE_2_OUTNE_LEDEN_LOW          0x00
#define MODE_2_OUTNE_LEDEN_HIGH         0x01
#define MODE_2_OUTNE_LEDEN_HIGHZ        0x02

/**
 * @brief Other PCA9685 macros
 */
#define LED_OFFSET_ADR                  0x06
#define SUBADR_OFFSET_ADR               0x01
#define STAB_TIME                       1     //Stabilization time (ms)
#define PWM_OUTPUT_COUNTER_MAX          0x1000  //0000h to 0FFFh (12 bit) counter

/**
 * @brief Read PCA9685 mode 1 register
 */
pca9685_err_t pca9685_i2c_read_mode_1(uint8_t *mode);

/**
 * @brief Write PCA9685 mode 2 register
 */
pca9685_err_t pca9685_i2c_read_mode_2(uint8_t *mode);

/**
 * @brief Set PCA9685 register auto increment in mode 1 register
 */
pca9685_err_t pca9685_i2c_autoincrement(pca9685_auto_incr_t setting);

/**
 * @brief PCA9685 restart
 */
pca9685_err_t pca9685_i2c_restart();

/**
 * @brief PCA9685 sleep mode setting
 */
pca9685_err_t pca9685_i2c_sleep_mode(pca9685_sleep_mode_t sleep_mode);

/**
 * @brief PCA9685 reset
 */
pca9685_err_t pca9685_i2c_reset();

/**
 * @brief PCA9685 output initialization
 */
pca9685_err_t pca9685_i2c_output_init(pca9685_output_t setting);

/**
 * @brief Set PCA9685 LEDx HIGH/LOW output 
 */
pca9685_err_t pca9685_i2c_led_set(uint8_t led_no, pca9685_led_state_t state);

/**
 * @brief Set PCA9685 all LEDs HIGH/LOW output 
 */
pca9685_err_t pca9685_i2c_all_led_set(pca9685_led_state_t state);

/**
 * @brief Set PCA9685 LEDx PWM output
 */
pca9685_err_t pca9685_i2c_led_pwm_set(uint8_t led_no, float d_cycle, float delay);

/**
 * @brief Set PCA9685 all LEDs PWM output
 */
pca9685_err_t pca9685_i2c_all_led_pwm_set(float d_cycle, float delay);

/**
 * @brief Set PCA9685 pre scale settings
 */
pca9685_err_t pca9685_i2c_write_pre_scale(double frequency, double osc_clk_hz);

/**
 * @brief Read PCA9685 pre scale settings
 */
pca9685_err_t pca9685_i2c_read_pre_scale(double *frequency, double osc_clk_hz);

/**
 * @brief Set PCA9685 all call address
 */
pca9685_err_t pca9685_i2c_write_allcall_addr(uint8_t allcall_addr);

/**
 * @brief Read PCA9685 all call address
 */
pca9685_err_t pca9685_i2c_read_allcall_addr(uint8_t *allcall_addr);

/**
 * @brief Set PCA9685 sub address
 */
pca9685_err_t pca9685_i2c_write_sub_addr(pca9685_subaddr_no_t addr_no, uint8_t sub_addr);

/**
 * @brief Read PCA9685 sub address
 */
pca9685_err_t pca9685_i2c_read_sub_addr(pca9685_subaddr_no_t addr_no, uint8_t *sub_addr);

/**
 * @brief Set PCA9685 sub address response type
 */
pca9685_err_t pca9685_i2c_sub_addr_resp(pca9685_subaddr_no_t sub_addr, pca9685_addr_resp_t resp);

/**
 * @brief Set PCA9685 all call address response type
 */
pca9685_err_t pca9685_i2c_allcall_address_resp(pca9685_addr_resp_t resp);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_PCA9685_I2C */
