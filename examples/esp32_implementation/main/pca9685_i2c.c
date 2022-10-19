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

#include "math.h"

#include "pca9685_i2c.h" 
#include "pca9685_i2c_hal.h" 

pca9685_err_t pca9685_i2c_read_mode_1(uint8_t *mode)
{
    uint8_t reg = REG_MODE_1;
    pca9685_err_t err = pca9685_i2c_hal_read(I2C_ADDRESS_PCA9685, &reg, mode, 1);
    return err;
}

pca9685_err_t pca9685_i2c_read_mode_2(uint8_t *mode)
{
    uint8_t reg = REG_MODE_2;
    pca9685_err_t err = pca9685_i2c_hal_read(I2C_ADDRESS_PCA9685, &reg, mode, 1);
    return err;
}

pca9685_err_t pca9685_i2c_restart()
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;
    if(pca9685_i2c_read_mode_1(&mode) != PCA9685_OK && !(mode & (1 << 7)))
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = mode | (mode & ~(1 << 4));
    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    if(pca9685_i2c_read_mode_1(&mode) != PCA9685_OK)
        return PCA9685_ERR;
    data[1] = mode | (mode & (1 << 7));
    err += pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_reset()
{
    uint8_t reg = REG_RESET;
    uint8_t data[2];
    data[0] = reg;
    data[1] = SWRST;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_output_invert(pca9685_output_invert_t val)
{
    uint8_t reg = REG_MODE_2;
    uint8_t data[2];
    uint8_t mode;
    
    pca9685_err_t err = pca9685_i2c_read_mode_2(&mode);
    if(err != PCA9685_OK)
        return err;

    data[0] = reg;
    data[1] = mode | (val << 4);

    err += pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_output_enable(pca9685_output_enable_t val)
{
    uint8_t reg = REG_MODE_2;
    uint8_t data[2];
    uint8_t mode;
    
    pca9685_err_t err = pca9685_i2c_read_mode_2(&mode);
    if(err != PCA9685_OK)
        return err;

    data[0] = reg;
    data[1] = mode | (val << 1);

    err += pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_output_drive(pca9685_output_drive_t val)
{
    uint8_t reg = REG_MODE_2;
    uint8_t data[2];
    uint8_t mode;
    
    pca9685_err_t err = pca9685_i2c_read_mode_2(&mode);
    if(err != PCA9685_OK)
        return err;

    data[0] = reg;
    data[1] = mode | (val << 2);

    err += pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_led_set(pca9685_led_t led)
{
    uint8_t reg = (led.led_no * 4) + LED_OFFSET_ADR;
    uint8_t data[5];
    data[0] = reg;
    data[2] = 1 << 4;
    if (led.state == PCA9685_LED_OFF)
        data[4] = 1 << 4;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_led_pwm_set(pca9685_led_pwm_t led)
{
    uint8_t reg = (led.led_no * 4) + LED_OFFSET_ADR;
    uint8_t data[5];
    data[0] = reg;
    data[1] = led.cycle.led_ON & 0xFF;
    data[2] = led.cycle.led_ON >> 8;
    data[3] = led.cycle.led_OFF & 0xFF;
    data[4] = led.cycle.led_OFF >> 8;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_all_led_set(pca9685_led_state_t state)
{
    uint8_t reg = REG_ALL_LED;
    uint8_t data[5];
    data[0] = reg;
    data[2] = 1 << 4;
    if (state == PCA9685_LED_OFF)
        data[4] = 1 << 4;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_all_led_pwm_set(pca9685_pwm_cycle_t cycle)
{
    uint8_t reg = REG_ALL_LED;
    uint8_t data[5];
    data[0] = reg;
    data[1] = cycle.led_ON & 0xFF;
    data[2] = cycle.led_ON >> 8;
    data[3] = cycle.led_OFF & 0xFF;
    data[4] = cycle.led_OFF >> 8;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_write_pre_scale(uint16_t frequency)
{
    uint8_t reg = REG_PRE_SCALE;
    uint8_t data[2];
    data[0] = reg;
    data[1] = round((25 * 1000 * 1000) / (4096 * frequency)) - 1;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_read_pre_scale(uint16_t *frequency)
{
    uint8_t reg = REG_PRE_SCALE;
    uint8_t data;
    pca9685_err_t err = pca9685_i2c_hal_read(I2C_ADDRESS_PCA9685, &reg, &data, 1);
    *frequency = round((25 * 1000 * 1000) / (4096 * (data + 1))); 

    return err;
}

pca9685_err_t pca9685_i2c_write_allcall_addr(uint8_t addr)
{
    uint8_t reg = REG_ALLCALLADR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = addr << 1;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_read_allcall_addr(uint8_t *addr)
{
    uint8_t reg = REG_ALLCALLADR;
    pca9685_err_t err = pca9685_i2c_hal_read(I2C_ADDRESS_PCA9685, &reg, addr, 1);

    return err;
}

pca9685_err_t pca9685_i2c_write_sub_addr(pca9685_subaddr_t subaddr)
{
    uint8_t reg = subaddr.addr_no + SUBADR_OFFSET_ADR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = subaddr.address << 1;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_read_sub_addr(pca9685_subaddr_t *subaddr)
{
    uint8_t reg = subaddr->addr_no + SUBADR_OFFSET_ADR;
    uint8_t data;
    pca9685_err_t err = pca9685_i2c_hal_read(I2C_ADDRESS_PCA9685, &reg, &data, 1);
    subaddr->address = subaddr->address >> 1;

    return err;
}