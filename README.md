# Autonomous Car - Low Level Control

## Project Overview

This project showcases the low-level control system of an autonomous car, utilizing the ESP32 microcontroller. The core functionality revolves around receiving movement commands (Twist messages) via serial communication and accurately following these commands. Additionally, the system computes and sends back odometry information, essential for autonomous navigation and control.

## Key Features

- **Serial Communication**: Interprets Twist commands sent over serial communication to control the vehicle's movements.
- **Odometry Calculation**: Accurately calculates and reports the vehicle's odometry, crucial for understanding its current position and orientation.
- **PID Control for Motors**: Utilizes both velocity and position PID controllers to manage the precise movement of four motors.
- **FreeRTOS for Task Management**: Leverages the dual-core architecture of the ESP32 with FreeRTOS, ensuring efficient real-time task handling.


## Code Overview

- **Motor Control Task**: Manages the PID control of each motor - **Core 0**
- **Vehicle Control Task**: Handles the computation of odometry data - **Core 1**
- **Communication Task**: Manages incoming serial commands for vehicle movement and outgoing serial data for odometry - **Core 1**

## Operation

The vehicle operates by receiving Twist commands via serial communication, which dictate its movement. The system processes these commands, controls the motors accordingly, and sends back odometry data through serial communication for external monitoring and control.

## improvements:
- **improve velocity calculation**: implement moving average , maybe increase encoder count from half quad to quad
- **add slip detection**: use current sensing and velocity calculation to detect wheel slipage
- **vehicle pos pid**: implement position pid in the vehicle level

![cute_car](https://github.com/ofekreches/RTmcu/assets/104078036/dec9b97e-dc59-4e71-b4dd-ce8ee4d0f429)
