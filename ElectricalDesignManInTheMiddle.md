# StrideControl PCB Architecture & Hardware Fail-Safe Design

This document details the core hardware architecture for the StrideControl interface board for the Sportsmaster T610 treadmill. The design utilizes galvanic isolation, opto-isolated sensor interfaces (PC817), an isolated RS232-to-TTL bridge for CSAFE (SP3232), and an active I²C bus accelerator (LTC4311) to interface safely with an ESP32-S3 microcontroller.

## Table of Contents
1. [Bill of Materials (BOM)](#bill-of-materials-bom)
2. [Power Distribution & Grounding Architecture](#1-power-distribution--grounding-architecture)
3. [Terminal Header Pinout (X1 & X2)](#2-terminal-header-pinout-x1--x2)
4. [Optocoupler Input & Output Interfaces (U1–U3)](#3-optocoupler-input--output-interfaces-u1u3)
5. [CSAFE Isolated RS232 Subsystem (SP3232)](#4-csafe-isolated-rs232-subsystem-sp3232)
6. [I²C Bus Accelerator & IMU Interface (LTC4311)](#5-ic-bus-accelerator--imu-interface-ltc4311)

---

## Bill of Materials (BOM)

* **MCU:** ESP32-S3 (WROOM-1 / DevKit)
* **U1 (PC817):** Optocoupler DIP-4 (Speed pulse input)
* **U2 (PC817):** Optocoupler DIP-4 (Incline pulse input)
* **U3 (PC817):** Optocoupler DIP-4 (Buzzer detection)
* **U4 (SP3232 Module):** Isolated 5V RS232-to-TTL transceiver module (CSAFE protocol)
* **U5 (LTC4311):** I²C bus active terminator / rise time accelerator breakout
* **C1:** 100 µF / 16V electrolytic capacitor (bulk decoupling on 5V rail)
* **Resistors:**
  * **R1 (10 kΩ, 0.25W):** Current-limiting resistor for U1 (sized for 11.4V resting voltage)
  * **R2 (1 kΩ, 0.25W):** Current-limiting resistor for U2 (sized for 4.68V pulse signal)
  * **R3 (220 Ω, 0.25W):** Series resistor for U3 input (part 1)
  * **R4 (220 Ω, 0.25W):** Series resistor for U3 input (part 2 – total 440 Ω in series)
  * **R5 (10 kΩ, 0.25W):** Pull-up resistor to +3.3V for U1 collector
  * **R6 (10 kΩ, 0.25W):** Pull-up resistor to +3.3V for U2 collector
  * **R7 (10 kΩ, 0.25W):** Pull-up resistor to +3.3V for U3 collector
  * **R8 (1 kΩ, 0.25W):** Voltage divider upper leg (5V TTL to 3.3V) for CSAFE RXD
  * **R9 (2 kΩ, 0.25W):** Voltage divider lower leg to GND (2 × 1 kΩ in series)
* **Connectors:** 2 × 12-pin screw terminal headers (X1 for external/treadmill, X2 for MCU/power)

---

## 1. Power Distribution & Grounding Architecture

The board operates across strictly separated power and ground domains:

### Power Rails
* **+5V Rail (Red wire):** Supplied externally via USB to **X1:12**. Filtered across bulk capacitor **C1**, feeds the SP3232 module VCC (5V0), and routes through **X2:9** to power the ESP32 via its `5V/VIN` pin.
* **+3.3V Rail (Blue wire):** Regulated by the ESP32 internal LDO, entering the board at **X2:7**. Supplies pull-up resistors **R5**, **R6**, **R7**, the LTC4311 module (`VIN` and `EN`), and provides remote IMU power via **X1:8**.

### Ground Domains
* **Common System 0V / GND (Brown wire):** Anchored at **X2:8** (ESP32 GND). Unifies external USB 0V (**X1:9**), capacitor **C1 (−)**, treadmill sensor ground pin 2 (**X1:6**), optocoupler emitters/cathodes (**U1** and **U2** pins 2 & 3 bridge, **U3** pin 3), and SP3232 TTL GND (**X2:10**).
* **Isolated RS232 GND (Brown-White wire):** Dedicated exclusively to **X1:2** and the SP3232 module's RS232-side ground. It maintains complete galvanic isolation from System 0V. The Cat6 cable shield terminates at **X1:2** on the PCB side, but **must remain cut and insulated at the treadmill end** to prevent ground loops.

---

## 2. Terminal Header Pinout (X1 & X2)

### X1 Header (Inputs / Treadmill / External Interfaces)

| Pin | Function | Direction | Signal Level | Wiring / Description |
| :---: | :--- | :---: | :---: | :--- |
| **X1:1** | **Buzzer (+)** | Input | Pulsed DC | Treadmill buzzer (+) wire $\rightarrow$ R3/R4 $\rightarrow$ U3 pin 1 (Anode) |
| **X1:2** | **RS232 GND** | Reference | Isolated GND | CSAFE isolated ground + Cat6 cable shield |
| **X1:3** | **RS232 TXD** | Output | $\pm$5V to $\pm$12V | SP3232 TXD pad $\rightarrow$ Treadmill RXD line |
| **X1:4** | **RS232 RXD** | Input | $\pm$5V to $\pm$12V | Treadmill TXD line $\rightarrow$ SP3232 RXD pad |
| **X1:5** | **Speed In** | Input | 11.4V Idle | Treadmill sensor cable pin 7 $\rightarrow$ R1 (10 kΩ) $\rightarrow$ U1 pin 1 |
| **X1:6** | **Sensor GND** | Reference | 0V | Treadmill sensor cable pin 2 $\rightarrow$ System 0V |
| **X1:7** | **Incline In** | Input | 4.68V Logic | Treadmill sensor cable pin 11 $\rightarrow$ R2 (1 kΩ) $\rightarrow$ U2 pin 1 |
| **X1:8** | **+3.3V Out** | Output | +3.3V DC | Power feed to remote IMU over Cat6 cable |
| **X1:9** | **USB 0V In** | Reference | 0V | External USB cable Black wire $\rightarrow$ C1(−) and System 0V |
| **X1:10** | **I²C SCL** | Bidirectional | 3.3V Logic | IMU Cat6 Green wire $\rightarrow$ LTC4311 SCL |
| **X1:11** | **I²C SDA** | Bidirectional | 3.3V Logic | IMU Cat6 Orange wire $\rightarrow$ LTC4311 SDA |
| **X1:12** | **+5V In** | Input | +5.0V DC | External USB cable Red wire $\rightarrow$ C1(+) and SP3232 5V0 |

### X2 Header (ESP32 Microcontroller & Power Routing)

| Pin | Function | Direction | Logic Level | ESP32 Pin / Description |
| :---: | :--- | :---: | :---: | :--- |
| **X2:1** | **Buzzer (−)** | Input | Floating | Treadmill buzzer (−) wire $\rightarrow$ U3 pin 2 (Cathode). *Not connected to ESP32 or 0V* |
| **X2:2** | **I²C SDA** | Bidirectional | 3.3V CMOS | **GPIO 47** $\leftrightarrow$ LTC4311 SDA |
| **X2:3** | **I²C SCL** | Bidirectional | 3.3V CMOS | **GPIO 48** $\leftrightarrow$ LTC4311 SCL |
| **X2:4** | **Buzzer Signal** | Output | 3.3V Active-Low | **GPIO 16** $\leftarrow$ U3 Collector / R7 pull-up |
| **X2:5** | **Incline Pulse**| Output | 3.3V Active-Low | **GPIO 14** $\leftarrow$ U2 Collector / R6 pull-up |
| **X2:6** | **Speed Pulse** | Output | 3.3V Active-Low | **GPIO 3** $\leftarrow$ U1 Collector / R5 pull-up |
| **X2:7** | **+3.3V In** | Input | +3.3V DC | From ESP32 3V3 pin (supplies R5–R7, LTC4311, and IMU) |
| **X2:8** | **0V / GND** | Reference | 0V | From ESP32 GND pin (main anchor for System 0V) |
| **X2:9** | **+5V Out** | Output | +5.0V DC | To ESP32 5V/VIN pin (powered from C1+) |
| **X2:10** | **SP3232 GND** | Reference | 0V | Bridged internally to X2:8 (TTL ground reference) |
| **X2:11** | **CSAFE RXD** | Output | 3.3V UART TTL | **GPIO 40** $\leftarrow$ Voltage divider mid-point (R8/R9) |
| **X2:12** | **CSAFE TXD** | Input | 3.3V UART TTL | **GPIO 17** $\rightarrow$ SP3232 TTL RXD pad |

---

## 3. Optocoupler Input & Output Interfaces (U1–U3)

### Speed Pulse (U1 - PC817)
* **Input Circuit:** Treadmill pin 7 (11.4V idle) $\rightarrow$ **X1:5** $\rightarrow$ **R1 (10 kΩ)** $\rightarrow$ **U1 Pin 1 (Anode)**.
* **Ground Bridge:** U1 Pin 2 (Cathode) and Pin 3 (Emitter) bridged via bare wire across the 7.62 mm DIP gap $\rightarrow$ routed to System 0V (**X2:8**).
* **Output Circuit:** Pin 4 (Collector) tied to pull-up **R5 (10 kΩ)** to +3.3V (**X2:7**). The junction routes to **X2:6**, read by **GPIO 3** as an active-low pulse stream.

### Incline Pulse (U2 - PC817)
* **Input Circuit:** Treadmill pin 11 (4.68V pulsed) $\rightarrow$ **X1:7** $\rightarrow$ **R2 (1 kΩ)** $\rightarrow$ **U2 Pin 1 (Anode)**.
* **Ground Bridge:** U2 Pin 2 (Cathode) and Pin 3 (Emitter) bridged via bare wire $\rightarrow$ routed to System 0V (**X2:8**).
* **Output Circuit:** Pin 4 (Collector) tied to pull-up **R6 (10 kΩ)** to +3.3V (**X2:7**). The junction routes to **X2:5**, read by **GPIO 14** as an active-low pulse stream.

### Buzzer Detection (U3 - PC817)
* **Input Circuit (High-Side):** Treadmill buzzer (+) wire $\rightarrow$ **X1:1** $\rightarrow$ **R3 (220 Ω)** in series with **R4 (220 Ω)** ($R_{\text{total}} = 440\text{ }\Omega$) $\rightarrow$ **U3 Pin 1 (Anode)**.
* **Floating Return (Low-Side):** Treadmill buzzer (−) wire $\rightarrow$ **X2:1** $\rightarrow$ **U3 Pin 2 (Cathode)**. Maintained fully floating (isolated from System 0V) to prevent bypassing low-side switching drivers.
* **Output Circuit:** Pin 3 (Emitter) tied to System 0V (**X2:8**). Pin 4 (Collector) tied to pull-up **R7 (10 kΩ)** to +3.3V (**X2:7**). The junction routes to **X2:4**, read by **GPIO 16** as active-low.

---

## 4. CSAFE Isolated RS232 Subsystem (SP3232)

* **Module:** SP3232 RS232-to-TTL transceiver powered by 5V from the USB rail.
* **RS232 Bus Interface (Treadmill side):**
  * Module TXD pad $\rightarrow$ **X1:3** $\rightarrow$ Treadmill CSAFE RX.
  * Module RXD pad $\leftarrow$ **X1:4** $\leftarrow$ Treadmill CSAFE TX.
  * Isolated GND pad $\rightarrow$ **X1:2** (terminates Cat6 shield; shield cut/isolated at treadmill RJ45 end).
* **TTL Microcontroller Interface (ESP32 side):**
  * **Transmit:** **GPIO 17** $\rightarrow$ **X2:12** $\rightarrow$ SP3232 TTL RXD pad.
  * **Receive & Level Shifting:** SP3232 TTL TXD pad (5V output) routes to voltage divider **R8 (1 kΩ)** and **R9 (2 kΩ)** to GND. The scaled 3.3V midpoint routes via **X2:11** into **GPIO 40**.

---

## 5. I²C Bus Accelerator & IMU Interface (LTC4311)

* **Accelerator Integration:** The LTC4311 breakout board connects in parallel across the I²C bus lines to handle line capacitance over the extended Cat6 cable run to the IMU.
* **Enable Configuration:** `VIN` and `EN` pins on the LTC4311 board are bridged directly together and tied to +3.3V (**X2:7**), ensuring the slew-rate accelerators remain permanently enabled during operation.
* **Bus Routing:**
  * **SDA:** Bridges IMU SDA (**X1:11**), LTC4311 SDA, and ESP32 **GPIO 47** (**X2:2**).
  * **SCL:** Bridges IMU SCL (**X1:10**), LTC4311 SCL, and ESP32 **GPIO 48** (**X2:3**).
* **Cable Shielding:** Unused conductor pairs in the Cat6 IMU cable are grouped together and tied to System 0V (**X2:8**) to provide a reference ground plane.
