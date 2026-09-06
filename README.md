# Smart Autonomous Hospital Delivery & Safety Monitor Robot

## Overview

This repository contains the Arduino C++ firmware and documentation for an autonomous delivery robot designed to operate within a controlled hospital grid. The system automates the distribution of lightweight clinical materials while ensuring cargo security, tracking structural barriers, and actively monitoring the environment for fire or smoke hazards.

## Core Features

* **PID Line Tracking:** Uses a 5-channel infrared array to calculate a weighted error for smooth, proportional-integral-derivative (PID) differential steering.
* **RFID Junction Routing:** Scans under-floor coordinate identifier tags using SPI communication to execute precise 90-degree intersection pivots based on user-selected destinations.


* **Safety-Lock Payload Guarding:** A servo-controlled locking mechanism that allows manual toggling while idle, secures automatically during transit, and unlocks only upon arrival at the verified RFID coordinate.


* **Intelligent Collision Avoidance:** Continuously computes runtime echo gaps via an ultrasonic sensor, halting motor execution if obstacles appear within 20 cm.


* **Multi-Hazard Alarm System:** Integrates smoke and flame sensors to trigger emergency software overrides, immediately cutting motor power and pulsing an I2C-driven alarm.



## Hardware Components

| Component | Function |
| --- | --- |
| **Arduino Uno R3** | Primary microcontroller

 |
| **L298N Motor Driver** | Dual H-Bridge for 5V DC geared motors

 |
| **TCRT5000L Array** | 5-channel IR line tracking

 |
| **MFRC522** | RFID module for coordinate reading

 |
| **HC-SR04** | Ultrasonic proximity sensor

 |
| **SG90 Micro Servo** | Cargo compartment locking mechanism

 |
| **PCF8574** | I2C I/O expansion module for buttons/alarms

 |
| **16x2 I2C LCD** | User interface and telemetry display

 |

## Pin Mapping

| Arduino Pin | Connected Component |
| --- | --- |
| **Digital 0 (RX)** | Smoke Sensor |
| **Digital 1 (TX)** | IR Sensor 1 (Far Left) |
| **Digital 2** | SG90 Servo (PWM handled by Timer 1) |
| **Digital 3** | L298N IN1 (Left Reverse PWM) |
| **Digital 4** | HC-SR04 TRIG |
| **Digital 5** | L298N IN2 (Left Forward PWM) |
| **Digital 6** | L298N IN3 (Right Forward PWM) |
| **Digital 7** | MFRC522 RST |
| **Digital 8** | HC-SR04 ECHO |
| **Digital 9** | L298N IN4 (Right Reverse - PWM bypassed due to Timer 1 conflict) |
| **Digital 10** | MFRC522 SS (SDA) |
| **Digital 11-13** | MFRC522 SPI (MOSI, MISO, SCK) |
| **Analog 0-3** | IR Sensors 2, 3, 4, 5 |
| **Analog 4-5** | I2C SDA/SCL (LCD and PCF8574) |

## Dependencies

Ensure the following libraries are installed in your Arduino IDE via the Library Manager:

* `Wire.h` (Built-in I2C)
* `SPI.h` (Built-in SPI)
* `LiquidCrystal_I2C`
* `MFRC522`
* `Servo`
