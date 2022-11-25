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

#include "stdio.h"

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

pca9685_err_t pca9685_i2c_autoincrement(uint8_t setting)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode_mask;
    if(pca9685_i2c_read_mode_1(&mode_mask) != PCA9685_OK)
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = mode_mask | (setting << 5);
    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_restart()
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode_mask;
    if(pca9685_i2c_read_mode_1(&mode_mask) != PCA9685_OK && !(mode_mask & (1 << 7)))
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = mode_mask | (mode_mask & ~(1 << 4));
    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    if(pca9685_i2c_read_mode_1(&mode_mask) != PCA9685_OK)
        return PCA9685_ERR;
    data[1] = mode_mask | (mode_mask & (1 << 7));
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

pca9685_err_t pca9685_i2c_output_init(pca9685_output_t output)
{
    uint8_t reg = REG_MODE_2;
    uint8_t data[2];
    uint8_t mode_mask;
    
    pca9685_err_t err = pca9685_i2c_read_mode_2(&mode_mask);
    if(err != PCA9685_OK)
        return err;

    data[0] = reg;
    data[1] = (mode_mask & 0xFC) | (output.invrt << 4) | (output.och << 3) | (output.outdrv << 2) | (output.outne << 1);

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

pca9685_err_t pca9685_i2c_led_pwm_set(uint8_t led_no, uint8_t d_cycle, uint8_t delay)
{
    uint8_t reg = (led_no * 4) + LED_OFFSET_ADR;
    uint8_t data[5];
    uint16_t delay_tm = round(delay * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_on_tm = round(d_cycle * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_off_tm = delay_tm + led_on_tm;

    data[0] = reg;
    data[1] = (delay_tm - 1) & 0xFF;
    data[2] = (delay_tm - 1) >> 8;
    data[3] = led_off_tm > 4096 ? (4096 - led_off_tm) & 0xFF : (led_off_tm - 1) & 0xFF;
    data[4] = led_off_tm > 4096 ? (4096 - led_off_tm) >> 8 : (led_off_tm - 1) >> 8;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_all_led_pwm_set(uint8_t d_cycle, uint8_t delay)
{
    uint8_t reg = REG_ALL_LED;
    uint8_t data[5];
    uint16_t delay_tm = round(delay * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_on_tm = round(d_cycle * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_off_tm = delay_tm + led_on_tm;

    if (led_on_tm == led_off_tm)    /* The LEDn_ON and LEDn_OFF count registers should never be programmed with the same values */
        return PCA9685_ERR;

    data[0] = reg;
    data[1] = (delay_tm - 1) & 0xFF;
    data[2] = (delay_tm - 1) >> 8;
    data[3] = led_off_tm > 4096 ? (4096 - led_off_tm) & 0xFF : (led_off_tm - 1) & 0xFF;
    data[4] = led_off_tm > 4096 ? (4096 - led_off_tm) >> 8 : (led_off_tm - 1) >> 8;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_write_pre_scale(uint16_t frequency)
{
    uint8_t reg = REG_PRE_SCALE;
    uint8_t data[2];
    data[0] = reg;
    data[1] = round((25 * 1000 * 1000) / (4096 * frequency)) - 1;
    printf("data[1]: %d", data[1]);
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

pca9685_err_t pca9685_i2c_allcall_address_resp(uint8_t resp)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode_mask;
    if(pca9685_i2c_read_mode_1(&mode_mask) != PCA9685_OK)
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = mode_mask | (resp << 0);
    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
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

pca9685_err_t pca9685_i2c_sub_address_resp(uint8_t sub_addr, uint8_t resp)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode_mask;
    if(pca9685_i2c_read_mode_1(&mode_mask) != PCA9685_OK)
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = mode_mask | (resp << sub_addr);
    pca9685_err_t err = pca9685_i2c_hal_write(I2C_ADDRESS_PCA9685, data, sizeof(data));
    return err;
}