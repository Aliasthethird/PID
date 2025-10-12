# PID Depth Control (ESP32)

**Author:** Gero Nootz
**Board:** Lolin Lite (ESP32)
**Version:** 1.1.1
**Date:** October 12, 2025

---

## Overview

This PlatformIO project controls the **depth of a propeller-driven system** using a **PID controller** and an **MS5837 pressure sensor**.
The PID output drives a servo (ESC) to maintain or oscillate around a target depth, displayed on a **SSD1306 OLED**.

The PID implementation resides in `lib/PID/`, and **generic serial functionality** has been modularized into the new `lib/SerialLineReader/` library (introduced in v1.1.1).

---

## Requirements

* **PlatformIO / Arduino framework**
* **ESP32Servo library version 3.0.6**
  (Newer versions require the latest Arduino core and code rewrite.)
* **Libraries:**

  * `Adafruit_GFX`
  * `Adafruit_SSD1306`
  * `MS5837`
  * `ESP32Servo@3.0.6`
  * `Wire`

---

## Serial Commands

| Command | Example   | Description                                    |
| ------- | --------- | ---------------------------------------------- |
| `KP:x`  | `KP:0.5`  | Set proportional gain                          |
| `KI:x`  | `KI:0.02` | Set integral gain                              |
| `KD:x`  | `KD:0.1`  | Set derivative gain                            |
| `SP:x`  | `SP:0.3`  | Set target depth (m)                           |
| `LPF:x` | `LPF:0.9` | Set low-pass filter gain                       |
| `ALT:x` | `ALT:1`   | Enable/disable alternating setpoint mode       |
| `dSP:x` | `dSP:0.1` | Set the step size (m) for alternating setpoint |

---

## Features

* Zero-depth calibration on startup
* OLED display for calibration, arming, and live setpoint display
* ESC arming countdown
* **Alternating setpoint option** for automatic step-response testing — ideal for **demonstration and tuning purposes**
* Serial output for live PID diagnostics
* **New in v1.1.1:**

  * Refactored serial read logic for cleaner structure
  * Introduced **SerialLineReader** library to handle generic serial input
  * Minor refactoring across modules for clarity and consistency

---

## Notes

* `motor1Pin` changed from GPIO **22 → 12**
* ADC input removed (no longer used)
* New `alterSP()` routine alternates the setpoint every 10 seconds when `ALT` is enabled

---

## Project Structure

```
PID-Depth-Control/
├── src/
│   └── main.cpp
├── lib/
│   ├── PID/
│   │   ├── PID.cpp
│   │   └── PID.h
│   └── SerialLineReader/
│       ├── SerialLineReader.cpp
│       └── SerialLineReader.h
├── platformio.ini
└── README.md
```

---

## Changelog

| Version   | Date       | Changes                                                                                                                                                            |
| --------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **1.1.1** | 2025-10-12 | Refactored serial read logic; introduced `SerialLineReader` library for generic serial handling; performed minor refactoring for clarity and consistency.          |
| **1.1.0** | 2025-10-10 | Added alternating setpoint feature (`ALT`, `dSP`) for demonstration purposes; removed ADC input; changed motor pin to GPIO 12; OLED now displays current setpoint. |
| **1.0.0** | 2025-07-30 | Initial release — PID depth control with MS5837 sensor, OLED feedback, and motor arming sequence.                                                                  |