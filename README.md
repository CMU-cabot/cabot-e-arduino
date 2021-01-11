[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Cabot-E-Arduino

This repository is related to the top handle and its components (push buttons, touch sensor, and vibrators) of Cabot. The handle is usually 3D printed. All of the components are assembled on the handle and connected to an Arduino Mega 2560 board using various connections (shown beow in the connection diagram). The source code found in this repository, will help you to integrate all of these components with the Cabot body. 

## Pre-requisites

### Hardware

One example of hardware components to build the handle.

- [3D print parts](https://github.com/CMU-cabot/cabot_design/tree/master/cabot2_e2/handle)
- [Arduino Mega 2560](https://store.arduino.cc/usa/mega-2560-r3)
- [MPR121](https://www.adafruit.com/product/1982) capacitive touch sensor
- [PCB shield example](https://github.com/RealCabot/simplePCB.git)
  - This Arduino shield is derived from an earlier project. It also has IMU and motor controllers, but here we will use it for the capacitive touch sensor
- 4 [push buttons](https://www.adafruit.com/product/4183)
- 3 [mini disc vibrators](https://www.adafruit.com/product/1201)
- [Copper foil tape](https://www.adafruit.com/product/3483)
- Wires and headers

### Software

In order to run ROS on Arduino Mega 2560, you will need both the Arduino IDE as well as ROS Serial Arduino Library. Please follow the instructions on the following page for installing these requirements:
[Installation instructions](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

<font color = "blue">After installing Arduino IDE, please make sure to open the IDE at least once before proceeding to the next steps.</font>

The following packages are essential for letting the touch sensor, push buttons and vibrator communicate with Arduino Mega 2560. Please open a new terminal and copy-paste the following instructions:
```
cd ~/Arduino/libraries
git clone https://github.com/adafruit/Adafruit_BNO055.git
git clone https://github.com/adafruit/Adafruit_Sensor.git
git clone https://github.com/adafruit/Adafruit_ADXL343.git
git clone https://github.com/adafruit/Adafruit_MPR121.git
git clone https://github.com/JChristensen/Timer.git
git clone https://github.com/frankjoshua/rosserial_arduino_lib.git
rosrun rosserial_arduino make_libraries.py ~/Arduino
```

## Assembly instructions

The following figure explains the manner in which the touch sensor, push buttons and vibrators are supposed to be connected with Arduino Mega 2560.

<p align="center">
  <img src="figures/Arduino_shield.svg">
</p>

## Components description

- The Cabot's handle has three main features namely, touch sensor, push buttons (four of them), and vibrators (three of them).
- The touch sensor is used for giving user feedback to the robot while it is moving, to ensure that the robot moves along with the user without getting lost.
- The three vibrators are useful for giving alerts to the user in response to obstacles encountered in the front and on the sides of the robot while it is moving. They also provide alerts/ warnings to the user about upcoming left turns and right turns.
- The four push butons are useful for giving manual input signals to the robot, to command it to move in preferred directions (forward, backward, left turn, and right turn).
- The touch sensor is usually connected to a conductive material like copper film, or copper plate.

