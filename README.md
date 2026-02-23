# BLE Sensor Node

Wireless 6-axis IMU sensor with real-time BLE streaming.

## Features
- ESP32-C3 + BMI160 sensor
- 6 BLE characteristics (Accel X/Y/Z, Gyro X/Y/Z)
- Real-time notifications at 10Hz
- SPI communication with BMI160
- NimBLE stack (ESP-IDF)

## Hardware
- ESP32-C3 Super Mini
- BMI160 6-axis IMU
- SPI: MOSI=GPIO6, MISO=GPIO5, CLK=GPIO4, CS=GPIO7

## Status
Sensor reading functional  
BLE GATT server with 6 characteristics  
Live notifications working  
Central receiver (next)  
PCB design ready for fabrication

## Part of
6-node wireless motion capture system for AI-driven movement analysis.