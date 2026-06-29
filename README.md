# Gyro-Mouse-
A wireless mouse that translates hand orientation into cursor movement using BLE, built entirely without high-level motion libraries
---
## Demo
![cursor movement gif](
https://github.com/user-attachments/assets/ff9c8221-ca1e-4097-84d8-d4ecea60acd1
)
> please open the above video for demo ( video is at 4x speed ) 
---

## Overview
An ESP32-based wireless mouse that uses an MPU6050 gyroscope to translate wrist rotation into cursor movement, sent over Bluetooth HID to any paired device.

---

## Hardware
| Component     | Purpose                          |
|---------------|----------------------------------|
| ESP32         | Main MCU, BLE stack              |
| MPU6050       | 6-axis IMU (gyro + accelerometer)|
| USB-C cable   | Power only                       |

---

## How it works
1. MPU6050 reads gyro data at 100Hz via I2C
2. Kalman filter smooths raw sensor noise
3. Filtered angles map to delta X/Y cursor values
4. ESP32 sends HID mouse report over BLE to host PC

---

## Key technical challenges
- **Register-level interfacing**: Configured MPU6050 directly via I2C registers
  without library abstraction for full control over sampling rate and DLPF
- **Offset calibration**: Implemented startup calibration routine
  to zero out sensor bias on each boot
- **Latency** : Improved latency by using low weight signal procesing
- **Constant Drift caused by Gyrometer** : Eliminated Constant drift by Kalman Filter
- **Power Loss due to Heat in Esp32**: Eliminated the constant power loss by introducing idlMode which makes the loop sleep for 1000ms instead of 10ms

---

## Getting started
### Prerequisites
- ESP-IDF v5.x or Arduino IDE with ESP32 board package
- MPU6050 library (only for reference — not used in main build)

### Flash instructions
```bash
git clone https://github.com/yourusername/gyro-mouse-esp32
cd gyro-mouse-esp32
idf.py build flash monitor
```

---

## Project structure
```
src/
  BLE_mouse.cpp          — trial mpu configuration
  sigprcs.py             — testing output of signal processing
  mpu_config.cpp         - mpu configuration, signal processing
  main.cpp               - BLE 
```

---
## Signal processing 
Raw gyro (131 LSB/°/s)
- subtract calibration offset
- Kalman filter  (Q=0.001, R=0.03)
- 5-sample moving average
- deadzone gate  (1.5 – 4.0 deg/s, set from noise floor)
- scale × SENSITIVITY (0.15)
- int8 dx/dy  →  BLE HID move()

## What I learned
- Bare-metal I2C communication on ESP32
- BLE HID profile implementation
- Sensor fusion and noise filtering in real-time systems

---

## Author
Akula Sai Preetham · IIT Kharagpur · [LinkedIn](your-link)
