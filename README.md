Smart Navigation Shoes for the Visually Impaired

An IoT-based assistive system designed to help visually impaired individuals navigate safely using sensor-based obstacle detection, stair detection, indoor navigation, and GPS-based emergency alerts.

The system consists of two ESP32-powered smart shoes, each responsible for different sensing tasks and providing audio feedback through buzzer patterns.

System Overview

The project uses multiple sensors integrated with ESP32 microcontrollers to detect environmental hazards and provide guidance to the user.

Main capabilities include:

Obstacle detection

Stair detection (up and down)

Indoor location identification using RFID

GPS-based emergency location sharing

Audio feedback for navigation cues

Shoe Modules
Shoe 1 – Navigation & Safety

This shoe performs environment sensing and emergency communication.

Components

ESP32

HC-SR04 Ultrasonic Sensor

VL53L0X ToF Sensor

MPU6050 IMU

NEO-6M GPS Module

Push Button (SOS)

Passive Buzzer

Functions

Detect obstacles in front of the user

Detect staircases using ToF distance measurements

Use IMU to determine foot motion and avoid false detections

Send emergency location via Telegram when SOS button is pressed

Provide buzzer alerts for navigation feedback

Shoe 2 – Indoor Navigation

This shoe assists with location identification in indoor environments.

Components

ESP32

HC-SR04 Ultrasonic Sensor

MPU6050 IMU

MFRC522 RFID Reader

Passive Buzzer

Functions

Detect nearby obstacles

Read RFID tags placed at locations (rooms, corridors, etc.)

Provide unique buzzer patterns to indicate specific locations

Feedback System

Different buzzer patterns inform the user about the surroundings.

Event	Audio Pattern
Normal ground	1 long beep
Stair up	2 beeps
Stair down	3 beeps
Obstacle detected	5 rapid beeps
RFID location markers	Unique patterns
Key Technologies

ESP32 – Main microcontroller for both shoes

Sensor Fusion – Combines ultrasonic, ToF, and IMU data

RFID – Indoor location identification

GPS + WiFi – Emergency location transmission

Audio Feedback – Real-time navigation assistance
