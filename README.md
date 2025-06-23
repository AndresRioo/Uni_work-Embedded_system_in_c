# Robotics Control System in C

## Project Description

This project implements embedded control algorithms for robotics applications using the C programming language on the MSP432 microcontroller platform. The focus is on communication with sensors, motor control via UART in half-duplex mode, and implementing basic movement states such as straight driving, wall following, and rotation.

The code includes UART initialization, half-duplex communication handling, and state management to enable the robot to navigate in different scenarios.

## Features

- UART communication setup and handling for data transmission and reception.
- State-based control system with states such as moving straight, wall on the right, wall on the left, and rotation.
- Motor control commands sent over UART in half-duplex mode.
- Timeout management with TimerA2 for communication reliability.
- Modular code structure for easy extension and integration.

## Hardware Platform

- MSP432P401R microcontroller.
- Custom robotics hardware with motor drivers controlled via UART.
- Sensors for wall detection.

## Usage

1. Compile the code using your preferred ARM compiler (in this project we used CCS).
2. Flash the binary to the MSP432 microcontroller.
3. Connect the robot hardware and power it on.
4. The robot will execute basic navigation based on sensor inputs and state logic.


