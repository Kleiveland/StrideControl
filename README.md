# 🏃‍♂️ StrideControl
**Smart control and modern dashboard for professional treadmills.**

StrideControl is a "restomod" project designed to modernize heavy-duty treadmills like the **Sportsmaster T610** and **Runfit 99**. While these machines are mechanically robust, their electronics often lack modern features like high-resolution displays, quick-access interval keys, and wireless connectivity.

This project uses an ESP32-S3 to interface with the treadmill's internal hardware, providing a tablet-based control system and Bluetooth FTMS support.

## ✨ Features
- **Modern Tablet UI:** A clean, responsive dashboard designed for a 10.1" mounted tablet.
- **Interval Shortcuts:** Dedicated "Rest" and "Sprint" buttons for instant speed and incline transitions.
- **Precision Tracking:** Direct interrupt-based reading of motor pulses for 100% accurate speed and distance.
- **Bluetooth FTMS:** Built-in support for Zwift, Strava, and other training apps.
- **Non-Destructive Integration:** Uses a Man-in-the-Middle (MitM) approach to inject commands without removing original functionality.

## 🛒 Bill of Materials (BOM)
To replicate this project, you will need:
- **MCU:** ESP32-S3-N16R8 (Dual Core, 16MB Flash, 8MB PSRAM).
- **Tablet:** Lenovo Tab M10 FHD (10.1", 1920x1200) or similar.
- **Analog Switch:** SN74HC4066 (For keyboard bus injection).
- **Isolation:** PC817 Optocoupler (for 12V speed pulses) and BSS138 Level Shifter (for 5V incline pulses).
- **Serial Interface:** MAX3232 RS232-to-TTL converter (for CSAFE status port).

## 🚀 Getting Started
1. Review the [DESIGN_GUIDE.md](./DESIGN_GUIDE.md) for technical specs and wiring.
2. Build the hardware interface for signal isolation and injection.
3. Flash the ESP32 firmware (source coming soon).
