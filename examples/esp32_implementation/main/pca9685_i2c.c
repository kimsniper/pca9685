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

pca9685_err_t pca9685_i2c_read_mode_1(pca9685_dev_t dev, uint8_t *mode)
{
    uint8_t reg = REG_MODE_1;
    pca9685_err_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, mode, 1);
    return err;
}

pca9685_err_t pca9685_i2c_read_mode_2(pca9685_dev_t dev, uint8_t *mode)
{
    uint8_t reg = REG_MODE_2;
    pca9685_err_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, mode, 1);
    return err;
}

pca9685_err_t pca9685_i2c_clock(pca9685_dev_t dev, pca9685_auto_extclk_t clk)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = (mode & 0xBF) | (clk << 6);
    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_autoincrement(pca9685_dev_t dev, pca9685_auto_incr_t setting)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = (mode & 0xDF) | (setting << 5);
    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_restart(pca9685_dev_t dev)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK && !(mode & (1 << 7)))
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = mode | (mode & ~(1 << 4));
    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;
    data[1] = mode | (mode & (1 << 7));
    err += pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_sleep_mode(pca9685_dev_t dev, pca9685_sleep_mode_t sleep_mode)
{
    uint8_t reg = REG_MODE_1;
    uint8_t data[2];
    uint8_t mode;
    
    pca9685_err_t err = pca9685_i2c_read_mode_1(dev, &mode);
    if(err != PCA9685_OK)
        return err;

    data[0] = reg;
    data[1] = (mode & 0xEF) | (sleep_mode << 4);

    err += pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    return err;
}

pca9685_err_t pca9685_i2c_reset()
{
    uint8_t data = SWRST;

    pca9685_err_t err = pca9685_i2c_hal_write(I2C_GEN_CALL_ADDRESS_PCA9685, &data, 1);
    pca9685_i2c_hal_ms_delay(STAB_TIME);
    return err;
}

pca9685_err_t pca9685_i2c_output_init(pca9685_dev_t dev, pca9685_output_t output)
{
    uint8_t reg = REG_MODE_2;
    uint8_t data[2];
    uint8_t mode;
    
    pca9685_err_t err = pca9685_i2c_read_mode_2(dev, &mode);
    if(err != PCA9685_OK)
        return err;

    data[0] = reg;
    data[1] = (mode & 0xE0) | (output.invrt << 4) | (output.och << 3) | (output.outdrv << 2) | (output.outne << 1);

    err += pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_led_set(pca9685_dev_t dev, uint8_t led_no, pca9685_led_state_t state)
{
    uint8_t reg = (led_no * 4) + LED_OFFSET_ADR;
    uint8_t data[5];
    data[0] = reg;
    data[2] = 1 << 4;
    if (state == PCA9685_LED_OFF)
        data[4] = 1 << 4;

    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_all_led_set(pca9685_dev_t dev, pca9685_led_state_t state)
{
    uint8_t reg = REG_ALL_LED;
    uint8_t data[5];
    data[0] = reg;
    data[2] = 1 << 4;
    if (state == PCA9685_LED_OFF)
        data[4] = 1 << 4;

    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_led_pwm_set(pca9685_dev_t dev, uint8_t led_no, float d_cycle, float delay)
{
    uint8_t reg = (led_no * 4) + LED_OFFSET_ADR;
    uint8_t data[5];
    uint16_t delay_tm = round(delay * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_on_tm = round(d_cycle * .01f * PWM_OUTPUT_COUNTER_MAX);
    uint16_t led_off_tm = delay_tm + led_on_tm;

    if(delay_tm == 0)
        delay_tm = 1;

    data[0] = reg;
    data[1] = (delay_tm - 1) & 0xFF;
    data[2] = (delay_tm - 1) >> 8;
    data[3] = led_off_tm > PWM_OUTPUT_COUNTER_MAX ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) & 0xFF : (led_off_tm - 1) & 0xFF;
    data[4] = led_off_tm > PWM_OUTPUT_COUNTER_MAX ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) >> 8 : (led_off_tm - 1) >> 8;

    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_all_led_pwm_set(pca9685_dev_t dev, float d_cycle, float delay)
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
    data[3] = led_off_tm > PWM_OUTPUT_COUNTER_MAX ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) & 0xFF : (led_off_tm - 1) & 0xFF;
    data[4] = led_off_tm > PWM_OUTPUT_COUNTER_MAX ? (PWM_OUTPUT_COUNTER_MAX - led_off_tm) >> 8 : (led_off_tm - 1) >> 8;

    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_write_pre_scale(pca9685_dev_t dev, double frequency, double osc_clk_hz)
{
    uint8_t reg = REG_PRE_SCALE;
    uint8_t data[2];
    data[0] = reg;
    data[1] = round(osc_clk_hz / (PWM_OUTPUT_COUNTER_MAX * frequency)) - 1;
    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_read_pre_scale(pca9685_dev_t dev, double *frequency, double osc_clk_hz)
{
    uint8_t reg = REG_PRE_SCALE;
    uint8_t data;
    pca9685_err_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, &data, 1);
    *frequency = (osc_clk_hz) / (PWM_OUTPUT_COUNTER_MAX * ((data) + 1)); 
    return err;
}

pca9685_err_t pca9685_i2c_write_allcall_addr(pca9685_dev_t dev, uint8_t allcall_addr)
{
    uint8_t reg = REG_ALLCALLADR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = allcall_addr << 1;

    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_read_allcall_addr(pca9685_dev_t dev, uint8_t *allcall_addr)
{
    uint8_t reg = REG_ALLCALLADR;
    uint8_t data;
    pca9685_err_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, &data, 1);
    *allcall_addr = data >> 1;

    return err;
}

pca9685_err_t pca9685_i2c_write_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t sub_addr)
{
    uint8_t reg = addr_no + SUBADR_OFFSET_ADR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = sub_addr << 1;

    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_read_sub_addr(pca9685_dev_t dev, pca9685_subaddr_no_t addr_no, uint8_t *sub_addr)
{
    uint8_t reg = addr_no + SUBADR_OFFSET_ADR;
    uint8_t data;
    pca9685_err_t err = pca9685_i2c_hal_read(dev.i2c_addr, &reg, &data, 1);
    *sub_addr = data >> 1;

    return err;
}

pca9685_err_t pca9685_i2c_sub_addr_resp(pca9685_dev_t dev, pca9685_subaddr_no_t sub_addr, pca9685_addr_resp_t resp)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;

    if(sub_addr == 1)
        sub_addr = 3;
    else if(sub_addr == 3) 
        sub_addr = 1;

    uint8_t data[2];
    data[0] = reg;
    data[1] = (mode & ~(1 << sub_addr)) | (resp << sub_addr);
    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

pca9685_err_t pca9685_i2c_allcall_address_resp(pca9685_dev_t dev, pca9685_addr_resp_t resp)
{
    uint8_t reg = REG_MODE_1;
    uint8_t mode;
    if(pca9685_i2c_read_mode_1(dev, &mode) != PCA9685_OK)
        return PCA9685_ERR;
    uint8_t data[2];
    data[0] = reg;
    data[1] = (mode & ~(1 << 0)) | resp;
    pca9685_err_t err = pca9685_i2c_hal_write(dev.i2c_addr, data, sizeof(data));
    return err;
}

void pca9685_i2c_register(pca9685_dev_t *dev, 
                          uint8_t _i2c_addr,
                          uint8_t _allcall_addr,
                          uint8_t _sub_addr_1,
                          uint8_t _sub_addr_2,
                          uint8_t _sub_addr_3){
    
    dev->i2c_addr = _i2c_addr;
    dev->allcall_addr = _allcall_addr;
    dev->sub_addr_1 = _sub_addr_1;
    dev->sub_addr_2 = _sub_addr_2;
    dev->sub_addr_3 = _sub_addr_3;

    pca9685_i2c_write_allcall_addr(*dev, dev->allcall_addr);
    pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_1, dev->sub_addr_1);
    pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_2, dev->sub_addr_2);
    pca9685_i2c_write_sub_addr(*dev, PCA9685_SUB_ADDR_3, dev->sub_addr_3);
}
