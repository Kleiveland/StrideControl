# StrideControl

StrideControl is a custom hardware and software replacement project for professional-grade treadmills, specifically targeting the Sportsmaster T610 and Runfit 99 models. 

The original hardware on these treadmills is mechanically excellent, but the user interface lacks modern features like quick-access interval keys and Bluetooth connectivity. This project uses an ESP32-S3 microcontroller to intercept the original hardware signals, allowing you to control the treadmill via a custom self-hosted web dashboard on a mounted tablet, while adding Bluetooth FTMS support for apps like Zwift.

## Core Capabilities
* Tablet-optimized web interface replacing the original segment displays.
* Hardware-level interval shortcuts (custom Rest and Sprint speeds).
* Direct interrupt-based reading of the motor controller for true speed and distance tracking.
* Bluetooth FTMS broadcasting.
* Non-destructive Man-in-the-Middle (MitM) command injection via the internal keyboard bus.

## Bill of Materials (BOM)
To build the hardware interface, the following components are required:

* Microcontroller: ESP32-S3-N16R8 Development Board (Dual Core, 16MB Flash, 8MB PSRAM, external antenna recommended).
* Display: Lenovo Tab M10 FHD (10.1", WUXGA 1920x1200) or similar tablet.
* Analog Switch: SN74HC4066 (Used to isolate the 16-pin capacitive keyboard bus during MitM injection).
* Speed Signal Isolation: PC817 Optocoupler + 1k Ohm resistor (Steps down the 12V motor speed pulse).
* Incline Signal Shifter: BSS138 Bi-directional Logic Level Converter (Steps down the 5V incline pulse).
* Serial Interface: MAX3232 RS232-to-TTL module (Connects ESP32 to the treadmill's RJ45 CSAFE port).

## Project Structure
* `/src` - C++ firmware for the ESP32 (Divided into Sensors, Logic, MitM, and Webserver).
* `/web` - HTML/CSS/JS files for the tablet dashboard (served via LittleFS).
* `/docs` - Hardware schematics and logic analyzer logs.
