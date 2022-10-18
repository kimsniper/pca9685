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

#ifndef MAIN_PCA96858_I2C
#define MAIN_PCA96858_I2C

#ifdef __cplusplus
extern "C" {
#endif

#include "pca96858_i2c_hal.h" 

/*
typedef struct{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
} pca96858_calib_t;
*/

/**
 * @brief PCA9685 device address.
 * @details PCA9685 I2C slave address.
 */
#define I2C_ADDRESS_PCA9685             0x76

/**
 * @brief PCA9685 command code registers.
 * @details R/W Command registers
 */
#define REG_CALIB                       0x88
#define REG_ID_PARTNUMBER               0xD0
#define REG_RESET                       0xE0
#define REG_STATUS                      0xF3
#define REG_CTRL_MEAS                   0xF4
#define REG_CONFIG                      0xF5
#define REG_PRESS_READ                  0xF7
#define REG_TEMP_READ                   0xFA

/**
 * @brief PCA9685 calibration setting.
 * @details Set global parameter calibration values.
 */
//pca96858_err_t pca96858_i2c_write_ctrl_meas(pca96858_ctrl_meas_t cfg);

/**
 * @brief PCA9685 calibration setting.
 * @details Set global parameter calibration values.
 */
//pca96858_err_t pca96858_i2c_read_ctrl_meas(pca96858_ctrl_meas_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_PCA96858_I2C */
