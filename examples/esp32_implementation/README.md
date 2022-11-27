# PCA9685 - ESP-IDF

PCA9685 i2c library for ESP-IDF.
ESP-IDF template used for this project: https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c/i2c_simple

## Overview

This example demonstrates usage of PCA9685 for servo PWM control.

### Hardware Required

To run this example, you should have one ESP32, ESP32-S or ESP32-C based development board as well as a PCA9685. The PCA9685 is an I2C-bus controlled 16-channel LED controller optimized for Red/Green/Blue/Amber (RGBA) color backlighting applications. Each LED output has its own 12-bit resolution (4096 steps) fixed frequency individual PWM controller that operates at a programmable frequency from a typical of 24 Hz to 1526 Hz with a duty cycle that is adjustable from 0 % to 100 % to allow the LED to be set to a specific brightness value. All outputs are set to the same PWM frequency. It is easy to operate via a simple I2C command, you can read the datasheet [here](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf).

#### Pin Assignment:

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                  | SDA             | SCL           |
| ---------------- | -------------- | -------------- |
| ESP I2C Master   | I2C_MASTER_SDA | I2C_MASTER_SCL |
| PCA9685          | SDA            | SCL            |


For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `Example Configuration` in `menuconfig`.

**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```bash
I (322) example_usage: Device reset: Successful
I (332) example_usage: i2c_addr: 0x40
I (332) example_usage: Frequency Setting: Successful
I (342) example_usage: PCA9685 initialization successful
I (342) example_usage: Servo drive: Duty cycle -> 1.0ms, Duration -> 20.0ms
I (2352) example_usage: Servo drive: Duty cycle -> 2.0ms, Duration -> 20.0ms
I (4352) example_usage: Servo drive: Duty cycle -> 1.0ms, Duration -> 20.0ms
I (6352) example_usage: Servo drive: Duty cycle -> 2.0ms, Duration -> 20.0ms
I (8352) example_usage: Servo drive: Duty cycle -> 1.0ms, Duration -> 20.0ms
I (10352) example_usage: Servo drive: Duty cycle -> 2.0ms, Duration -> 20.0ms
I (12352) example_usage: Servo drive: Duty cycle -> 1.0ms, Duration -> 20.0ms
I (14352) example_usage: Servo drive: Duty cycle -> 2.0ms, Duration -> 20.0ms
I (16352) example_usage: Servo drive: Duty cycle -> 1.0ms, Duration -> 20.0ms
I (18352) example_usage: Servo drive: Duty cycle -> 2.0ms, Duration -> 20.0ms
```
