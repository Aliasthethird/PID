# PID Depth Control (ESP32)

**Author:** Gero Nootz
**Board:** Lolin Lite (ESP32)
**Version:** 1.0.0
**Date:** July 30, 2025

---

## Overview

This project uses a **PID controller** to maintain a target depth using an **MS5837 pressure sensor** on an **ESP32 (Lolin Lite)**.
The PID output controls a **servo-driven propeller (ESC)** to stabilize depth.
A small **SSD1306 OLED** provides calibration and arming feedback.

The PID implementation resides in `lib/PID/`.

---

## Requirements

* **PlatformIO / Arduino framework**
* **ESP32Servo library version 3.0.6**
  (Newer versions require the latest Arduino core and a program rewrite.)
* **Libraries:**

  * `Adafruit_GFX`
  * `Adafruit_SSD1306`
  * `MS5837`
  * `ESP32Servo@3.0.6`
  * `Wire`

---

## Serial Commands

| Command | Example   | Description              |
| ------- | --------- | ------------------------ |
| `KP:x`  | `KP:0.5`  | Set proportional gain    |
| `KI:x`  | `KI:0.02` | Set integral gain        |
| `KD:x`  | `KD:0.1`  | Set derivative gain      |
| `SP:x`  | `SP:0.3`  | Set target depth (m)     |
| `LPF:x` | `LPF:0.9` | Set low-pass filter gain |

---

## Notes

* Zero-depth calibration performed at startup.
* OLED countdown during ESC arming.
* Live depth and PID data printed over serial.

---

## Project Structure

```
PID-Depth-Control/
├── src/
│   └── main.cpp
├── lib/
│   └── PID/
│       ├── PID.cpp
│       └── PID.h
├── include/
│   └── SerialLineReader.h
├── platformio.ini
└── README.md
```

---

## Changelog

| Version   | Date       | Changes                                                                         |
| --------- | ---------- | ------------------------------------------------------------------------------- |
| **1.0.0** | 2025-07-30 | Initial release — depth control with MS5837 sensor and PID library integration. |

> To add new entries: increment the version number, update the date, and briefly describe changes (e.g., bug fixes, tuning updates, new features).

---

Would you like me to include a short example `platformio.ini` block (with the exact library version locking for `ESP32Servo`)? It would make version tracking reproducible across setups.
