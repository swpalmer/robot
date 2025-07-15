Robot
=====
-----
Zig code for a self-balancing robot. Uses the `pigpio` library for GPIO control, including PWM for motor control and I2C for sensor communication.
The robot has two main drive motors and a gyroscope sensor.

BOM
---

| Qty. | Part Description | Cost Per Unit |
|---|---|---|
|   2 | ZS-X11H V2 motor driver for DC motors with Hall effect sensors. | $25 |
|   1 | BMI 160 IMU | $3 |
|   1 | Raspberry Pi 3 Model B+ | $35 |
|   1 | DC-DC Buck Converter 36V to 5V (adjustable, CV/CC)| |


Wiring
------
Short the jumper on each motor driver to enable PWM control.
Establish a common ground by connecting the GND pins of the motor drivers and the Raspberry Pi.
Wire the drive motors to the motor drivers. Ma, Mb, Mc, including the Hall effect sensors Ha, Hb, Hc, and +5v/Gnd
(Usually the colours of the sensor wires match the colour of the motor wires for the same phase.  Keep that in mind when wiring the motors.)

Wire the drive motor power input to a suitable battery or power supply.
I used hoverboard motors and the hoverboard battery.
Wire the same power source to the input of the buck v=converter and the output of the buck converter to the Raspberry Pi via the GPIO header.  Use both +5V pins and multiple ground pins.  Keep the power wires short. Consider adding electrolytic capacitors at the power connections to help absorb power spikes.


Motor 1:
- PWM pin goes to GPIO 12
- Direction pin goes to GPIO 26

Motor 2:
- PWM pin goes to GPIO 13
- Direction pin goes to GPIO 27

BMI 160:
 - Wire to Pi I2C

Building
--------
The build is configured to cross-compile for Raspbery Pi by default.
It expects to find the Raspberry Pi `include` and `lib` directories.
```bash
zig build
# When building RPi 5 use this to create a binary that will run on Rpi 4
zig build -Dcpu=cortex_a72
```

Raspberry Pi Configuration
--------------------------
`/boot/firmware/config.txt` must be edited to enable SPI and I2C.
The following lines must be added to the file:
```
dtparam=spi=on
dtparam=i2c_arm=on
```

The PWM device tree overlay must be enabled and configured
for PWM on GPIO 12 and 13.:
```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

On Raspberry Pi 3 you may need to disable the onboard audio driver
as it can interfere with the PWM.
```
dtparam=audio=off
```
