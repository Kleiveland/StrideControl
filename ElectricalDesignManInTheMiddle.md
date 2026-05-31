# StrideControl PCB Architecture & Hardware Fail-Safe Design

This document details the core hardware architecture for the StrideControl Man-in-the-Middle (MitM) interface board. The design is strictly built around robust hardware fail-safes, ensuring that the treadmill remains 100% operational even if the ESP32 loses power, crashes, or is in the middle of a boot sequence.

## 🧠 Bill of Materials (BOM)

* **MCU:** ESP32-S3 (e.g., WROOM-1 module)
* **IC1 (TXS0108E):** 8-channel Bi-directional Level Shifter (O1 Bus Sniffing + MUTE Control)
* **IC2 (TXS0108E):** 8-channel Bi-directional Level Shifter (O2 Bus Injection, all 8 channels)
* **IC3 (CD4066B):** Quad Bilateral Analog Switch (Handles O2 Bits 0–3)
* **IC4 (CD4066B):** Quad Bilateral Analog Switch (Handles O2 Bits 4–7)
* **R1 (10 kΩ):** Pull-down resistor to GND (TXS Output Enable / Latch-up protection)
* **R2 (10 kΩ):** Pull-up resistor to 5V (MUTE Fail-Safe protection)

---

## ⚡ 1. Power Distribution & Latch-up Protection (R1)

This section ensures the level shifters (IC1 and IC2) remain in a high-impedance (High-Z) state until the ESP32 has fully booted and stabilized its 3.3V logic rail.

**Power Routing:**

* **Touch Console Pin 1 (5V)** ➔ PCB 5V Rail ➔ **Motor Controller Pin 1 (5V)**
* *Internal 5V distribution:* Feeds IC1 (VCCB), IC2 (VCCB), IC3 (VDD), and IC4 (VDD).


* **Touch Console Pin 2 (GND)** ➔ PCB GND Plane ➔ **Motor Controller Pin 2 (GND)**
* *Internal GND distribution:* Feeds all ICs and the ESP32.


* **Internal Power:** 5V Rail feeds ESP32-S3 (VIN). The ESP32's internal LDO generates 3.3V.
* **ESP32-S3 (3.3V Out)** ➔ PCB 3.3V Rail ➔ IC1 (VCCA) & IC2 (VCCA).

**Safe Boot via R1 (Latch-up Protection):**

* **ESP32-S3 (GPIO 2)** ➔ IC1 (OE) & IC2 (OE)
* **IC1/IC2 (OE)** ➔ **R1 (10 kΩ)** ➔ GND
* *Hardware Logic:* While the ESP32 is unpowered or booting, R1 actively pulls the Output Enable (OE) pins to 0V. Both TXS chips are forced into a High-Z state, physically isolating the 3.3V and 5V domains and preventing silicon latch-up if the 5V rail energizes first.

---

## 🔇 2. MUTE Circuit & Hardware Fail-Safe (R2)

The CD4066 bilateral switches require 5V on their control pins to remain CLOSED (allowing normal console operation). If the ESP32 is off, IC1 is disabled by R1, leaving the control pins floating. R2 solves this by anchoring them to 5V.

**MUTE Signal Level Shifting:**

* **ESP32-S3 (GPIO 21)** ➔ IC1 (Channel A8, 3.3V)
* **IC1 (Channel B8, 5V)** ➔ PCB MUTE Trace ➔ Control Pins on IC3 (Pins 5, 6, 12, 13) & IC4 (Pins 5, 6, 12, 13)

**Hardware Fail-Safe via R2:**

* **PCB MUTE Trace** ➔ **R2 (10 kΩ)** ➔ PCB 5V Rail
* *Hardware Logic:* When the ESP32 is unpowered or booting, IC1 is disabled (High-Z). R2 pulls the MUTE trace up to 5V, forcing all CD4066 switches CLOSED. The treadmill operates 100% normally.
* *Software Logic (Active-Low):* To mute the console and inject a frame, the ESP32 must actively drive GPIO 21 LOW. This overcomes R2, pulling the CD4066 control pins to 0V and opening the switches, temporarily isolating the console.

---

## 🟢 3. O1 Bus (Sniffing / Scanning via IC1)

The O1 scanner signals pass straight through the PCB uninterrupted. They are passively "tapped" via IC1 and shifted down to 3.3V for the ESP32 to monitor.

* **Pin 3 (ROW_A):** Console ➔ Pass-through ➔ Motor Controller. (Tapped: B1 ➔ A1 ➔ GPIO 4)
* **Pin 4 (Common):** Console ➔ Pass-through ➔ Motor Controller. *(Not tapped, blind to ESP32).*
* **Pin 5 (ROW_B):** Console ➔ Pass-through ➔ Motor Controller. (Tapped: B2 ➔ A2 ➔ GPIO 5)
* **Pin 6 (ROW_C):** Console ➔ Pass-through ➔ Motor Controller. (Tapped: B3 ➔ A3 ➔ GPIO 6)
* **Pin 7 (ROW_D):** Console ➔ Pass-through ➔ Motor Controller. (Tapped: B4 ➔ A4 ➔ GPIO 7)
* **Pin 8 (ROW_E):** Console ➔ Pass-through ➔ Motor Controller. (Tapped: B5 ➔ A5 ➔ GPIO 15)

> ⚠️ **Design Rules for IC1 (TXS0108E):**
> 1. **Software PinMode:** GPIOs 4, 5, 6, 7, and 15 must be explicitly configured as `INPUT` in the ESP32 firmware to prevent bus contention on the bi-directional level shifter.
> 2. **Unused Channels:** Channels 6 and 7 on IC1 are unused. To prevent noise and ghost-switching, tie pins **A6** and **A7** directly to GND. (Leave B6 and B7 floating).
> 
> 

---

## 🟠 4. O2 Bus (Injection via IC3 & IC4)

The O2 data bus is routed *through* the CD4066 switches. The ESP32 injects its level-shifted 5V signals on the downstream side (towards the motor controller) when the switches are opened (MUTE active).

**IC3 (Handles Bits 0–3):**

* **Pin 9 (Bit 0):** Console ➔ IC3 (Switch 1) ➔ Motor Controller. (ESP Injection: GPIO 41 ➔ IC2 A1/B1 ➔ Downstream trace)
* **Pin 10 (Bit 1):** Console ➔ IC3 (Switch 2) ➔ Motor Controller. (ESP Injection: GPIO 42 ➔ IC2 A2/B2 ➔ Downstream trace)
* **Pin 11 (Bit 2):** Console ➔ IC3 (Switch 3) ➔ Motor Controller. (ESP Injection: GPIO 8 ➔ IC2 A3/B3 ➔ Downstream trace)
* **Pin 12 (Bit 3):** Console ➔ IC3 (Switch 4) ➔ Motor Controller. (ESP Injection: GPIO 9 ➔ IC2 A4/B4 ➔ Downstream trace)

**IC4 (Handles Bits 4–7):**

* **Pin 13 (Bit 4):** Console ➔ IC4 (Switch 1) ➔ Motor Controller. (ESP Injection: GPIO 10 ➔ IC2 A5/B5 ➔ Downstream trace)
* **Pin 14 (Bit 5):** Console ➔ IC4 (Switch 2) ➔ Motor Controller. (ESP Injection: GPIO 11 ➔ IC2 A6/B6 ➔ Downstream trace)
* **Pin 15 (Bit 6):** Console ➔ IC4 (Switch 3) ➔ Motor Controller. (ESP Injection: GPIO 12 ➔ IC2 A7/B7 ➔ Downstream trace)
* **Pin 16 (Bit 7):** Console ➔ IC4 (Switch 4) ➔ Motor Controller. (ESP Injection: GPIO 13 ➔ IC2 A8/B8 ➔ Downstream trace)
