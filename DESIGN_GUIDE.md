# 📘 Technical Design Guide: StrideControl

This guide contains the verified technical specifications for the Sportsmaster T610 and Runfit 99 treadmill hardware.

## 📊 Sensor Calibration & Logic
The following constants must be used for accurate data representation:

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Speed Factor** | 1.1148 | Formula: `km/h = Hz * 1.1148` |
| **Distance Factor** | 3.2292 | Pulses per meter measured on Pin 7 |
| **Incline Frequency** | 394 Hz | Constant frequency during motor movement (Pin 11) |
| **Start-Up Delay** | 3.5s | Ignoring data during the machine's internal "3-2-1" countdown |

## ⚙️ Architecture & Core Allocation
To ensure high-precision timing, the ESP32-S3 utilizes both cores:
- **Core 0 (System):** Manages WiFi, WebSockets, and the web server for the tablet UI.
- **Core 1 (Hardware):** Handles time-critical interrupts for pulse counting, MitM command injection, and safety watchdogs.

## 🛡️ Safety Watchdogs
- **Ghost Running (C7):** If the CSAFE state is `InUse` but the speed sensor detects 0 Hz for > 2 seconds, a safety reset is triggered.
- **Homing Logic:** The system monitors Pin 11 during the `QUICK START` sequence. Calibration to 0.0% incline is locked once the 394 Hz signal ceases.

## 🔌 Hardware Interfacing
- **Speed (Pin 7):** 12V Pulse -> PC817 Optocoupler -> ESP32 GPIO.
- **Incline (Pin 11):** 5V Pulse -> BSS138 Level Shifter -> ESP32 GPIO.
- **Keyboard Bus:** 16-pin flat cable intercepted via SN74HC4066 analog switches.
- **CSAFE Status:** RS232 (9600 Baud, 8-N-1) via MAX3232 to the treadmill's RJ45 port.
