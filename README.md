# PID Depth Control (ESP32)

**Author:** Gero Nootz
**Board:** Lolin Lite (ESP32)
**Version:** 1.2.0
**Date:** October 13, 2025

---

## Overview

This PlatformIO project controls the **depth of a propeller-driven system** using a **PID controller** and an **MS5837 pressure sensor**.
The PID output drives a servo (ESC) to maintain or oscillate around a target depth, displayed on an **SSD1306 OLED**.

The PID logic is implemented in `lib/PID/`, and **serial input handling** is encapsulated in the modular `lib/SerialLineReader/` library (introduced in v1.1.1).

---

## Requirements

* **PlatformIO** with the **Arduino framework**
* **ESP32Servo library v3.0.6**
  (Note: newer versions require the latest Arduino core and may need code adjustments.)
* **Additional libraries:**

  * `Adafruit_GFX`
  * `Adafruit_SSD1306`
  * `MS5837`
  * `ESP32Servo@3.0.6`
  * `Wire`

---

## Serial Commands

| Command | Example    | Description                                            |
| ------- | ---------- | ------------------------------------------------------ |
| `KP`    | `KP:0.5`   | Set proportional gain                                 |
| `KI`    | `KI:0.02`  | Set integral gain                                     |
| `KD`    | `KD:0.1`   | Set derivative gain                                   |
| `SP`    | `SP:0.3`   | Set target depth (m)                                  |
| `LPF`   | `LPF:0.9`  | Set low-pass filter gain                              |
| `FGM`   | `FGM:1`    | Set function generator mode<br><em>Modes:</em> 0=DC, 1=Sine, 2=Rectangle, 3=Sawtooth, 4=Triangle |
| `AMP`   | `AMP:0.01` | Set function generator amplitude                      |
| `T`     | `T:10000`  | Set function generator period (ms)                    |

## Features

* Zero-depth calibration on startup
* OLED display for calibration, arming, and live setpoint visualization
* ESC arming countdown
* **Alternating setpoint option** for step-response testing — ideal for **PID tuning and demonstration**
* Serial output for real-time PID diagnostics
* **New in v1.2.0:**

  * Added **Function Generator**, allowing automatic target depth variation with configurable amplitude, period, and waveform type
  * Improved serial parsing reliability and feedback messages
  * General code cleanup and documentation updates

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
| **1.3.0** | 2025-10-18 | Use FunctionGenerator lib.     |
| **1.2.0** | 2025-10-13 | Added Function Generator for variable target depth; enhanced serial command handling and feedback; refined code comments and documentation for better clarity.     |
| **1.1.1** | 2025-10-12 | Refactored serial read logic; introduced `SerialLineReader` library for generic serial handling; performed minor refactoring for clarity and consistency.          |
| **1.1.0** | 2025-10-10 | Added alternating set point feature (`ALT`, `dSP`) for demonstration purposes; removed ADC input; changed motor pin to GPIO 12; OLED now displays current set point. |
| **1.0.0** | 2025-07-30 | Initial release — PID depth control with MS5837 sensor, OLED feedback, and motor arming sequence.                                                                  |