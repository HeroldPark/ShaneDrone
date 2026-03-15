# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ShaneDrone is a DIY micro-drone flight controller firmware for the **Arduino Nano ESP32** board, built with PlatformIO and the Arduino framework. It includes a real-time web dashboard served via LittleFS over WiFi.

## Build & Development Commands

All commands use **PlatformIO CLI** (`pio`).

```bash
# Compile firmware (빌드만)
pio run -e arduino_nano_esp32

# Compile and upload firmware ✅ JTAG 사용 (COM4 불필요)
pio run -e arduino_nano_esp32-jtag -t upload

# Upload LittleFS filesystem (web dashboard files in /data/)
pio run -e arduino_nano_esp32-jtag -t uploadfs

# Open serial monitor (115200 baud) — COM4 필요
pio device monitor -p COM4 -b 115200

# Clean build
pio run -e arduino_nano_esp32 -t clean
```

### Upload Method: JTAG (esp-builtin)
- `arduino_nano_esp32-jtag` 환경은 USB JTAG 인터페이스로 업로드 (COM4 불필요)
- Windows에서 **Zadig**로 `USB JTAG/serial debug unit (Interface 0)`에 **WinUSB** 드라이버 설치 필요 (최초 1회)
- 업로드 후 RESET 버튼 누르지 말고 **USB 재연결**로 부팅 (RESET 누르면 DFU 모드 진입)
- 시리얼 모니터가 필요하면 Zadig에서 `NanoESP32` 드라이버를 **CDC (serial)** 로 복원

## Architecture

### Control Flow
```
main.cpp (setup/loop)
  ├── sensors.cpp     — MPU9250 I2C driver, complementary filter attitude estimation
  ├── control.cpp     — PID controllers, motor mixing, LEDC PWM output
  ├── communication.cpp — SBUS RC input, WiFi AP, WebSocket telemetry, MSP/VTX
  └── web_async.cpp   — AsyncWebServer routes, LittleFS static file serving
```

### Loop Timing
- **Main loop**: 1000Hz — sensor reads
- **Control update**: 500Hz — PID calculation
- **Telemetry broadcast**: 50Hz — WebSocket JSON push

### Global Data Structures (defined in `include/config.h`)
- `SensorData` — attitude (roll/pitch/yaw), raw IMU values
- `ControllerInput` — RC channels normalized -1.0 to +1.0
- `MotorOutputs` — 4 motor PWM values (1000–2000µs)
- `DroneState` — armed flag, calibrated flag, battery voltage

### Web Dashboard
- Runs as WiFi AP: SSID `Shane_Drone`, password `drone123`, IP `192.168.4.1`
- Static files (HTML/CSS/JS) served from LittleFS (`/data/` directory)
- WebSocket at `/ws` for real-time bidirectional comms
- Must run `uploadfs` separately whenever `/data/` files change

### Flight Modes
- `STABILIZE (0)` — self-leveling
- `ANGLE (1)` — fixed angle
- `ACRO (2)` — pure rate control

## Critical Hardware Notes

### I2C (MPU9250 Sensor)
- Call `Wire.begin()` with **no arguments** — Arduino Nano ESP32 defaults to GPIO11 (SDA) / GPIO12 (SCL)
- Passing explicit `Wire.begin(SDA, SCL)` arguments causes pin mapping failures
- MPU9250 `NCS` pin **must** be tied to 3.3V to force I2C mode; floating NCS activates SPI mode and breaks I2C communication
- Firmware tries both I2C addresses 0x68 and 0x69 on startup
- If sensor initialization fails, firmware automatically falls back to **simulation mode** so the rest of the system (web dashboard, telemetry) remains functional

### Motor PWM
- Uses ESP32 LEDC peripheral (not analogWrite)
- Pins 5–8 for motors in quadcopter X-configuration
- PWM range: 1000–2000µs

### Serial Ports
- SBUS RC receiver: Serial2, pin 4, 100kHz inverted
- VTX (Walksnail Avatar MSP): Serial1, pins 16–17

## Key Configuration (`include/config.h`)
All PID gains, pin assignments, timing constants, safety thresholds, and data structures live here. Edit this file to tune the drone or change hardware mapping.

PID defaults (tuned for BETAFPV 1102 14000KV motors):
- Roll/Pitch: P=1.2, I=0.8, D=0.15
- Yaw: P=1.5, I=0.5, D=0.05

Battery safety:
- Critical: 3.4V → immediate disarm
- Low: 3.5V → warning

## Current Known Issue
The MPU9250 I2C communication is not working with the physical hardware (tested on two separate modules). The firmware compiles and the web dashboard functions correctly, but real sensor data is unavailable. Simulation mode is active as fallback. Suspected causes: NCS pin wiring or I2C pull-up resistors.
