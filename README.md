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
 * Update the code.
   - FreeRTOS 10.4.2 has been released & Fantail needs to be updated to use this.
   - The code needs to have exception (try-throw-catch) blocks, with proper error handling. Currently, I'm not happy with the way Fantail handles errors.
   - At some point down the track, I want to restructure Fantail's code to be a lot cleaner than what it is, currently.
   - I²C Bus Scanning. Both the PCA9685 & BNO055 chips run on the I²C Bus & doing a clean sweep of the I²C Bus on startup would provide a better way of detecting connected devices, than what's already been implemented.
 * Fork the libraries that Fantail uses?
   - I'm in two minds of doing this.
   If I do it, that removes Fantail's current dependencies on third party libraries. Though, it would be up to me, to ensure that those same libraries are canonically identical to their originals - Including updating to their latest versions, when their respective developers release updates.
   In a way, I am already doing this with FreeRTOS, as I'm having to adapt it to compile in the Arduino IDE.
   If I don't do it, that puts the burden of chasing down Fantail's dependencies on anyone that were to obtain the source code. In the case of FreeRTOS, that means YOU would have to create your own Arduino adaptation of it; & I know full well how much of a headache creating an Arduino port of ANY common-place API can be (& FreeRTOS is no exception to this).
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

## Why I have decided against the use of UAVCAN:
 When I did the initial release of Fantail here, on GitHub, I hinted at a possible implementation of CAN (Control Area Network) in a future update.
 I have back-tracked on this idea & have decided not to do it.
 From what my research tells me, I would need to have a CAN Bus going from Fantail out to one (or more) CAN-to-PWM adapter(s), where the adapters are hooked up to the helicopter's servos & ESC (motor controller) - Because these still rely on PWM to do whatever-it-is that they need to do.
 The RC receiver (what plugs into Fantail's SRXL2 port) uses a half-duplex UART protocol. So, Fantail would almost double as an SRXL2-to-UAVCAN adapter; & for every adapter, to me, that's a bottle-neck - IE another device that can complicate a process that is quite simple. Not only that, there is also the power consumption that I need to think of as well. Each CAN-to-PWM adapter would need to have another microcontroller, what translates the UAVCAN data to PWM Servo Data (this includes the ESC too, because it relies on the same PWM signal that the servos use).

 So, in order to properly implement CAN (more specifically, UAVCAN), the entire helicopter's avionic ecosystem would need to be on the CAN Bus.
 This would mean each servo's controller board (the electronics what's inside the servo itself) would need to have a UAVCAN interface; the ESC would need to have its own integrated UAVCAN interface; the battery management system would need its own UAVCAN interface; the receiver, rather than using SRXL2, would also need to have its own UAVCAN interface; & only then, can Fantail make a sensible use of UAVCAN. But, if I do that, I would lose compatibility with off-the-shelf avionics. Because nobody manufactures servos, ESCs, receivers & battery management systems for RC helicopters that use UAVCAN, period. I would be the only one doing it; & I see that as a major disadvantage, what will forever outweigh any possible benefits that UAVCAN may bring to the Fantail Flybarless project.

## Why I have decided against the use of GPS:
 Because I am constantly reminded of the KISS Theory throughout this (& other) project(s) - Keep It Simple, Stupid. This is also the other reason why I went against UAVCAN too. The very second I bring either of these two things into the picture, the software's complexity increases. Also, I'll start falling down the rabbit hole, that is "Creeping Feature-itis", where I'll wind up promising gold & you guys will get nickel-plated fecal matter. That's not how I roll.
 I want Fantail to be as skookum as what's commercially available (EG BeastX Micro Beast, MSH Brain, Spirit etc), if not better than Fantail's commercial counterparts.

Fantail Flybarless © 2020. Cassandra "ZZ Cat" Robinson. All rights reserved.
