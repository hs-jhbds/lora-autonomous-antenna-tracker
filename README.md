# LoRa-Based Autonomous Antenna Tracking System

## Project Description

An ESP32-based autonomous antenna tracking system designed to detect and optimize the orientation of a receiving antenna using LoRa communication and Received Signal Strength Indicator (RSSI) measurements.

The system uses an XL1278-SMT (SX1278) LoRa module operating at 433.92 MHz to receive wireless beacon packets from a transmitter. It analyzes the received signal strength and automatically adjusts the antenna's azimuth and elevation using a 28BYJ-48 stepper motor and an MG996R servo motor.

During operation, the receiver performs an initial scanning process across predefined azimuth and elevation angles. At each position, it measures RSSI and identifies the orientation with the strongest received signal. Once the optimal position is selected, the system continuously monitors signal strength. If the RSSI drops below a configured threshold for consecutive monitoring cycles, the system initiates another scan to find a stronger signal.

A real-time dashboard provides system information, including RSSI, baseline signal strength, antenna orientation, tracking status, and signal history through serial communication with the ESP32.

## Key Features

- Autonomous azimuth and elevation scanning.
- LoRa-based wireless beacon reception at 433.92 MHz.
- RSSI-based antenna orientation optimization.
- Automatic signal degradation detection.
- Re-scanning based on configurable RSSI thresholds.
- ESP32-based embedded control.
- Real-time dashboard telemetry.
- Live monitoring of antenna position and signal strength.

## Hardware Components

- ESP32 development board
- XL1278-SMT LoRa transceiver (SX1278)
- 28BYJ-48 stepper motor
- ULN2003 stepper motor driver
- MG996R servo motor
- Transmitting LoRa module
- Directional or experimental receiving antenna

## Software and Technologies

- Arduino IDE
- Embedded C++
- RadioLib
- SPI communication
- LoRa modulation
- RSSI measurement
- Serial JSON telemetry
- HTML, CSS and JavaScript dashboard

## Objective

To develop an automated antenna orientation system that uses wireless signal-strength measurements to identify and maintain an effective receiving direction, reducing the need for manual antenna adjustment.

yt video-
https://youtu.be/_83Oc3CgYgQ?si=ilq8DlQc8vbfAHxD
