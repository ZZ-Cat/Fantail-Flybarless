# Fantail Flybarless
 #Arduino #FantailFlybarless #KeepRCHelisAlive

![DRM Free](https://static.fsf.org/dbd/label/DRM-free%20label%20120.en.png)

[Keep it that way](https://www.defectivebydesign.org/what_is_drm_digital_restrictions_management)

### Written & developed by:
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/0)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/0)[![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/1)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/1)[![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/2)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/2)[![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/3)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/3)[![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/4)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/4)[![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/5)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/5)[![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/6)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/6)[![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/7)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/7)

## Description:
 Electronic stabilization for radio controlled helicopters.

 Fantail is an electronic flybarless stabilization unit, that's designed for use in radio controlled helicopters.
 For those that fly radio controlled helicopters, you already have a fair idea of what flybarless units are (&, by extension, what this is).
 For those that don't, a flybarless unit is a device that helps maintain flight stability, without sacrificing control inputs from the pilot.

 As far as flybarless units go, Fantail doesn't really bring anything new to the table. Other than the fact that it's entirely open source;
 it's programmed in the Arduino IDE; & its components can be sourced from places like Adafruit & Mouser.

## Features (IE What's been implemented thus far, & works):
 * Microchip SAM D21 ARM Cortex M0+ microcontroller.
   - This is the same microcontroller that's found on the Arduino Zero, Adafruit Metro M0 Express & Adafruit Feather M0 develpment boards.
   - Clocked at 48 MHz.
 * Firmware is updatable over USB.
 * FreeRTOS.
   - Fantail is driven by FreeRTOS, version 10.4.1. When Amazon Web Services releases updated versions of FreeRTOS, Fantail will be updated to reflect that.
 * Servo Front End.
   - A dedicated task, that manages all communications to the servos & ESC is called the "Servo Front End".
   - All servo channels are synchronized & get updated whenever the Servo Front End receives new control data.
   - The hardware, what does the PWM, is a PCA9685 I²C PWM driver. This can be found on Adafruit's 8-Channel PWM/Servo Driver Feather Wing development board.
 * 120° CCPM Swashplate mixing.
   - Other swashplate types have not been implemented, because all modern helicopters use 120° swashplates.
 * Built-in Range Limiter.
   - This prevents an individual servo from moving past its mechanical limits, what would otherwise damage the servo or whatever mechanical component that the
   servo is connected to.
 * Adjustable servo travel rates.
   - This controls the rotational rate of an individual servo's output shaft, by way of interpolating between set values. Currently, linear interpolation is used.
   - This allows compensation for the rising-&-settling behavior of 120° swashplates, that are connected to slow servos.

## What's Next (IE What will be implemented in future updates):
 * 9-DoF Orientation Sensor.
   - Sensor is a Bosch Sensortec BNO055. This is found on Adafruit's Absolute Orientation Sensor development board.
   - It has built-in Euler & Quaternion units, as well as being able to read raw gyroscope, accelerometer & magnetometer data.
 * PID Controller.
   - All control rates (IE How fast & how far the helicopter will rotate about its pitch, yaw & heading axes) will be brought out, to be adjusted by the user.
   - All Kp, Ki & Kd gains will be brought out, to be adjusted by the user.
 * Spektrum SRXL2 receiver interface.
   - This is the only open source protocol that I know of, that is used in the RC hobby.

## Errata:
 * TBD.

## Requirements:
 * TBD.

## How to obtain:
 * TBD.

## Quick Start Guide:
 * TBD.

## Contributing:
 * Read the Contributing Guidelines for details; & make sure you're thick-skinned, because it's a no holds barred approach.

## Software License:
![GNU GPL v3](https://www.gnu.org/graphics/gplv3-with-text-136x68.png)

Fantail Flybarless © 2020. Cassandra "ZZ Cat" Robinson. All rights reserved.
