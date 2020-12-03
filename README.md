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

 **Fantail is in its early stages of development & a lot of the core functionality is yet to be implemented.**

 Fantail is an open source electronic flybarless stabilization unit & flight controller, that's designed for use in radio controlled helicopters.
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

## What's Next (IE What will be implemented in future updates):
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
 * Read through the software license file & the Terms & Conditions file.
 * Read the Contribution Guidelines; & make sure you're thick-skinned, because it's a no holds barred approach - Even by my own standards.
   Because, apparently, I have to spell out for some people what they should already know, to begin with.
 * I have recently opened up a new branch called "Canary". Submit your Pull Requests there & I will review them, in accordance with my Contribution Guidelines.

## Software License:
![GNU GPL v3](https://www.gnu.org/graphics/gplv3-with-text-136x68.png)
Fantail Flybarless © 2020. Cassandra "ZZ Cat" Robinson. All rights reserved.

# Developer's Footnotes
###### Written by Cassandra "ZZ Cat" Robinson.

## Why I have decided against the use of UAVCAN:
 When I did the initial release of Fantail here, on GitHub, I hinted at a possible implementation of CAN (Control Area Network) in a future update.
 I have back-tracked on this idea & have decided not to do it.
 From what my research tells me, I would need to have a CAN Bus going from Fantail out to one (or more) CAN-to-PWM adapter(s), where the adapters are hooked up to the helicopter's servos & ESC (motor controller) - Because these still rely on PWM to do whatever-it-is that they need to do.
 The RC receiver (what plugs into Fantail's SRXL2 port) uses a half-duplex UART protocol. So, Fantail would almost double as an SRXL2-to-UAVCAN adapter; & for every adapter, to me, that's a bottle-neck - IE another device that can complicate a process that is quite simple. Not only that, there is also the power consumption that I need to think of as well. Each CAN-to-PWM adapter would need to have another microcontroller, what translates the UAVCAN data to PWM Servo Data (this includes the ESC too, because it relies on the same PWM signal that the servos use).

 So, in order to properly implement CAN (more specifically, UAVCAN), the entire helicopter's avionic ecosystem would need to be on the CAN Bus.
 This would mean each servo's controller board (the electronics what's inside the servo itself) would need to have a UAVCAN interface; the ESC would need to have its own integrated UAVCAN interface; the battery management system would need its own UAVCAN interface; the receiver, rather than using SRXL2, would also need to have its own UAVCAN interface; & only then, can Fantail make a sensible use of UAVCAN. But, if I do that, I would lose compatibility with off-the-shelf avionics. Because nobody manufactures servos, ESCs, receivers & battery management systems for RC helicopters that use UAVCAN, period. I would be the only one doing it; & I see that as a major disadvantage, what will forever outweigh any possible benefits that UAVCAN may bring to the Fantail Flybarless project.

## Why I have decided against the use of GPS:
 Because I am constantly reminded of the KISS Theory throughout this (& other) project(s) - **K**eep **I**t **S**imple, **S**tupid. This is also the other reason why I went against UAVCAN too. The very second I bring either of these two things into the picture, the software's complexity increases. Also, I'll start falling down the rabbit hole, that is "Feature Creep", where I'll wind up promising gold & everyone will get nickel-plated fecal matter. That's not how I roll.
 I want Fantail to be as skookum as what's commercially available (EG BeastX Micro Beast, MSH Brain, Spirit etc), if not better than Fantail's commercial counterparts.

## Why does Fantail only support SRXL2?
 Short answer: It's the only open source protocol in existence, that has public documentation.

 As far as I know, Spektrum's SRXL2 protocol is the only open source protocol in existence, period.
 Other popular protocols (such as Futaba's S.Bus, Team Black Sheep's CRSF... crikey, even combined PPM) are all closed source. For those that have open source firmware what use these protocols - either the developers work for these manufacturers, they have had to sign non-disclosure agreements (among other hoops that they would have had to have jumped through) to include these protocols in their own OEM firmware, or (worse yet) they would have reverse-engineered the protocols. The very latter of which, goes against one of the core principles of this project & (by extension) one of my own core values as a developer. Because, at that point, that's basically intellectual property theft; & theft of any kind is abhorrent.

 So, I couldn't do it, even if I wanted to.

 This is one of the reasons why I was looking at UAVCAN as an alternative protocol in the first place. But, because of the reasons that I have written above, that is why I can't use UAVCAN either. That would otherwise effectively turn Fantail into a monopoly on what you can & cannot use with the unit. IE If I implemented UAVCAN, you would __only__ be able to use __my avionics__ with __my flybarless unit.__ That is __not__ what Fantail is all about. Fantail is for everyone, period.
