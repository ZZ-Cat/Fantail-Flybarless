# Fantail Flybarless
 [#Arduino](https://www.facebook.com/hashtag/arduino) [#FantailFlybarless](https://www.facebook.com/hashtag/fantailflybarless) [#KeepRCHelisAlive](https://www.facebook.com/hashtag/keeprchelisalive)

![DRM Free](https://static.fsf.org/dbd/label/DRM-free%20label%20120.en.png)

[Keep it that way](https://www.defectivebydesign.org/what_is_drm_digital_restrictions_management)

### Written & developed by:
 [Cassandra "ZZ Cat" Robinson](https://bit.ly/ZZCatOnFacebook)
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/0)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/0)
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/1)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/1)
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/2)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/2)
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/3)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/3)
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/4)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/4)
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/5)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/5)
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/6)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/6)
 [![](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/images/7)](https://sourcerer.io/fame/ZZ-Cat/ZZ-Cat/Fantail-Flybarless/links/7)

## Description:
 Electronic stabilization for radio controlled helicopters.

 **Development of Fantail Flybarless has been discontinued.**

 Fantail was an open source electronic flybarless stabilization unit & flight controller, that's designed for use in radio controlled helicopters.
 For those that fly radio controlled helicopters, you already have a fair idea of what flybarless units are, & (by extension) what Fantail is.
 For those that don't, a flybarless unit is an electronic device that helps maintain flight stability, without sacrificing control inputs from the pilot.
 How Fantail does this, is by its use of a multi-axis PID controller, which reads orientation data off of a nine degrees-of-freedom fusion sensor & combines that with the pilot's control data from their transmitter & then, sends it out to the helicopter's servos & ESC. Thus, controlling the helicopter.

## Features (IE What's been implemented thus far, & works):
 * Microchip SAM D21 ARM Cortex M0+ microcontroller.
   - This is the same microcontroller that's found on the Arduino Zero, Adafruit Metro M0 Express & Adafruit Feather M0 develpment boards.
   - Clocked at 48 MHz.
 * Firmware is updatable over USB.
 * FreeRTOS.
   - Fantail's beating heart & soul.
   - Multi-tasking is efficiently managed by the OS, which has an advantage over the traditional single thread loop.
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

## What's Next (IE What was planned in future updates, but was never implemented):
 * 9-DoF Orientation Sensor.
   - Sensor is a Bosch Sensortec BNO055. This is found on Adafruit's Absolute Orientation Sensor development board.
   - It has built-in Euler & Quaternion units, as well as being able to read raw gyroscope, accelerometer & magnetometer data.
 * PID Controller.
   - All tunable parameters will be brought out to the end user.
 * Spektrum SRXL2 receiver interface.
   - This is the only open source protocol that I know of, that is used in the RC hobby.

## Errata:
 * TBD.

## Requirements:
 * Hardware (For prototyping purposes ONLY):
   * [Adafruit BNO055 Orientation Sensor Breakout.](https://www.adafruit.com/product/2472)
   * [Adafruit Feather M0.](https://www.adafruit.com/product/2772)
   * [Adafruit PWM/Servo Feather Wing.](https://www.adafruit.com/product/2928)
   * [Solderless "breadboard".](https://www.adafruit.com/product/239)
   * [Spektrum 4651T SRXL2 Receiver with Telemetry.](https://www.spektrumrc.com/Products/Default.aspx?ProdID=SPM4651T)
   * Your helicopter.
 * Software:
   * [Arduino IDE](https://www.arduino.cc/en/software)
   * Adafruit SAMD Board Support Package
   * Arduino SAMD Board Support Package
   * [FreeRTOS](https://github.com/FreeRTOS/FreeRTOS-Kernel/releases)
   * [Servo Easing](https://github.com/ArminJo/ServoEasing/releases)

## How to obtain:
 * TBD.

## Quick Start Guide:
 * TBD.

## Contributing:
 * Contributions to this project have been discontinued.

## Software License:
![GNU GPL v3](https://www.gnu.org/graphics/gplv3-with-text-136x68.png)
Fantail Flybarless © 2020. Cassandra "ZZ Cat" Robinson. All rights reserved.

# Developer's Footnotes
###### Written by Cassandra "ZZ Cat" Robinson.

## Discontinuation of Fantail:
I have chosen to discontinue Fantail in favor of helping out with testing & developing [RotorFlight](https://github.com/rotorflight).
It makes no sense to me to create a project that competes with another project that I am already supporting. In my view, that is a conflict of interest & I view that as unethical.
To avoid that conflict of interest, I have chosen to discontinue Fantail & focus more on helping out the folks that are working on RotorFlight.
 
