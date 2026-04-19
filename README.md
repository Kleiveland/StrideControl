# StrideControl

Older commercial treadmills like the Sportsmaster T610 and Runfit 99 are built like tanks, but their control panels are completely outdated. No Bluetooth, no interval shortcuts, and clunky segment displays.

StrideControl is a hardware mod that fixes this. It uses an ESP32-S3 to hijack the internal keyboard bus (MitM) and reads raw hardware interrupts directly from the motor controller. This lets you control the machine from a web dashboard on a mounted tablet, while also broadcasting your actual speed and incline to Zwift via Bluetooth FTMS.

## What it does
* Replaces the old panel interface with a self-hosted web dashboard.
* Adds physical-feeling interval shortcuts (Rest/Sprint) that the original panel lacks.
* Tracks true speed and distance by counting motor pulses, bypassing the inaccurate stock display.
* Broadcasts data to Zwift and Strava over Bluetooth FTMS.
* Leaves the original hardware intact: if the ESP32 dies, the original panel still works.

## Parts Needed (BOM)
To build the interface board, you need:
* **MCU:** ESP32-S3-N16R8 (Dual Core, 16MB Flash, 8MB PSRAM). Get one with an external antenna connector; the motor casing has a lot of EMI.
* **Display:** Lenovo Tab M10 FHD or any modern tablet.
* **Analog Switch:** SN74HC4066 to intercept and inject signals into the 16-pin capacitive keyboard bus.
* **Speed Isolation:** PC817 Optocoupler + 1k resistor. (The motor pulse is 12V, the ESP32 will fry without this).
* **Incline Shifter:** BSS138 Logic Level Converter (Drops the 5V incline pulse to 3.3V).
* **Serial Interface:** MAX3232 RS232-to-TTL module to read the treadmill's RJ45 CSAFE port.

## Setup
Everything you need to know about the hardware timings and pin logic is documented in [DESIGN_GUIDE.md](./DESIGN_GUIDE.md).
