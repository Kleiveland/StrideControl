# StrideControl

StrideControl is a hardware modification project for the Sportsmaster T610 and Runfit 99 treadmills. These machines use a 3.5 HP AC motor and robust mechanical components, but the original interface lacks modern features like tablet integration, interval shortcuts, and Bluetooth connectivity.

This project uses an ESP32-S3 to interface with the treadmill's internal hardware. It provides a web-based dashboard for tablet use and adds Bluetooth FTMS support for applications such as Zwift.

## Features
* Tablet-optimized web dashboard replacing legacy segment displays.
* Hardware-level interval shortcuts (Rest/Sprint) via MitM injection.
* Real-time speed and distance tracking using motor controller interrupts.
* Bluetooth FTMS broadcasting for third-party app compatibility.
* Non-destructive design: The original console remains functional as a backup.

## Bill of Materials (BOM)
* **MCU:** ESP32-S3-N16R8 (Dual Core, 16MB Flash, 8MB PSRAM).
* **Display:** Lenovo Tab M10 FHD or similar 10.1" tablet.
* **Analog Switch:** SN74HC4066 (For 16-pin keyboard bus injection).
* **Speed Isolation:** PC817 Optocoupler + 1k Ohm resistor (For 12V speed pulses).
* **Incline Shifter:** BSS138 Logic Level Converter (For 5V incline pulses).
* **Serial Interface:** MAX3232 RS232-to-TTL module (For CSAFE status port).

## Project Structure
* `/src` - ESP32 firmware (Sensors, Logic, MitM, and FTMS modules).
* `/web` - Dashboard files for the tablet UI.
* `/docs` - Hardware logs and design specifications.
