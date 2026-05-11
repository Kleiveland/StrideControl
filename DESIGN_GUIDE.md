# StrideControl Technical Design Guide (V2.6)

This document contains all verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system (V2.6 Architecture) safely and predictably.

> **Primary UI Goal**
> StrideControl provides a clean, tablet-based web interface for direct speed and incline control. Because the tablet completely covers the original console, the UI must display the treadmill's **actual speed** and **actual incline** derived from post-EMA hardware feedback, not just requested target values.

---

## 1. Project Goal

The objective is to build a safe, reversible ESP32-S3 interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:
* Hosts a local web interface for a tablet.
* Allows direct user control of speed and incline.
* Reads actual physical state directly from the hardware.
* Parses machine state from the CSAFE port.
* Broadcasts telemetry via Bluetooth FTMS.
* Preserves original hardware safety features (the console must work if the ESP32 loses power).

This is a **non-destructive overlay system**, not a replacement for the main motor controller.

---

## 2. System Scope & Architecture

* **Role Split:** The ESP32 handles the UI, telemetry, FTMS, and command injection. The treadmill's mainboard retains full control over the 3.5 HP AC drive motor, incline motor power, and safety logic (e-stop key).
* **Core Design:** The web UI is stateless. Physical console buttons are not spoofed at the capacitive level; instead, command injection happens on the decoded 5V logic bus after the touch-controller IC.

---

## 3. Protocol & Hardware Mapping

### 3.1 Harness Measurements (Verified)
* **Black Wire:** `+11.75V DC` (Constant)
* **Brown Wire:** `0V` (GND / Reference)
* **Pin 6:** `5.0V DC` (Constant)
* **Pin 12:** `5.0V DC` (Constant)

### 3.2 Front-Panel Bus Mapping
Console communication uses time-division multiplexing across two 5V logic buses:
* **O1 Bus (Scanner):** Pins 3, 4, 5, 6, 7, 8. Line 4 acts as a shared active line, pulled LOW during ROW_A through ROW_D scans.
* **O2 Bus (Data/Inject):** Pins 9 through 16. An 8-bit parallel data bus used to return key presses during specific O1 scan windows.

### 3.3 GPIO Mapping (ESP32-S3 N16R8)

| Signal | ESP32 GPIO | Connection Target |
| :--- | :--- | :--- |
| **PIN_ROW_A** | GPIO 4 | O1 Line 3+4 (ROW_A) |
| **PIN_ROW_B** | GPIO 5 | O1 Line 4+5 (ROW_B) |
| **PIN_ROW_C** | GPIO 6 | O1 Line 4+6 (ROW_C) |
| **PIN_ROW_D** | GPIO 7 | O1 Line 4+7 (ROW_D) |
| **PIN_ROW_E** | GPIO 15 | O1 Line 8 (ROW_E) |
| **O2_BUS (0-7)** | 41, 42, 8, 9, 10, 11, 12, 13 | O2 Lines 9-16 |
| **MUTE_PIN** | GPIO 21 | Controls 74HC4066 gates |
| **CSAFE_RX/TX**| GPIO 16 / 17 | MAX3232 Interface |
| **SPEED_IN** | GPIO 3 | Pin 7 (via PC817) |
| **INCLINE_IN** | GPIO 14 | Pin 11 (via BSS138) |

---

## 4. Sensor Interpretation

### 4.1 Speed (Pin 7)
Read via a `PC817` optocoupler for 100% isolation. The software applies a Minimum Pulse Width (blanking) filter to strip AC motor noise, followed by an Exponential Moving Average (EMA) filter. This provides the single source of truth for actual speed.

### 4.2 Incline (Pin 11)
Read via a `BSS138` level shifter. Calculated by counting pulses relative to a 0% homed baseline. The pulse count is committed to **NVS** every time physical movement stops, ensuring accurate position recovery across reboots.

### 4.3 Cadence (LSM6DSOX)
An `LSM6DSOX` IMU is mounted to the frame via I2C (actively terminated via `LTC4311`). It uses hardware step-counting derived from footstrike shockwaves. Polled asynchronously to prevent blocking Core 1 motor interrupts.

---

## 5. Calibration Constants

* **Speed:** `km/h = Hz * 1.1148` (Anchored at 10 km/h = 8.97 Hz).
* **Odometer:** `1 meter = 3.2292 pulses`.
* **Incline:** **[BLOCKER]** Final pulses-per-percent constant is pending a physical 0-15% sweep validation.

---

## 6. Hardware Interface Choices

### 6.1 Mute Circuit (74HC4066)
* **Normal State:** `MUTE_PIN` is **LOW**. The bilateral switches are closed, allowing native console communication.
* **Injection State:** `MUTE_PIN` is **HIGH**. The switches open, isolating the console while the ESP32 drives the O2 bus.
* **Fail-Safe:** A 10kΩ **pull-down** resistor on `MUTE_PIN` ensures the hardware defaults to normal console operation if the ESP32 crashes or loses power.

### 6.2 Modular PCB Architecture
* **Board 1 (Interceptor Shield):** Inline with the 16-pin ribbon. Houses the `SN74HC4066N` switches and fail-safe logic.
* **Board 2 (Core Brain):** Houses the ESP32-S3, `HLK-PM01` PSU, level shifters (`TXS0108E`), and optical isolators.

---

## 7. Software Architecture

### 7.1 Dual-Core Task Split
* **Core 0:** WiFi, Async Web Server, WebSockets, Bluetooth FTMS, LittleFS.
* **Core 1:** Hard real-time tasks (pulse counting ISRs, hardware command injection, safety watchdogs).
* *Note:* All shared memory structures must be protected by a `portMUX_TYPE` spinlock.

### 7.2 Memory & Storage
* **Hardware:** The `N16R8` profile (16MB Flash, 8MB PSRAM) is required to prevent heap fragmentation from the dual-role BLE stack and WebSockets.
* **Serialization:** Handled deterministically using `ArduinoJson` with statically allocated buffers.
* **Partitions:** A custom CSV partition table is required (allocating >1.8MB for `app0` and a dedicated `LittleFS` partition for web assets).

### 7.3 Injection Logic
Injection strictly occurs within a ~3ms window triggered by a hardware interrupt (ISR) detecting the correct O1 row pattern. For fractional targets (e.g., 12.6 km/h), the queue uses `round()` logic to calculate the shortest path via macro base numbers plus discrete increments.

---

## 8. Timing & Safety Rules

* **3.5s Startup Sync:** The ESP32 ignores speed/distance telemetry for 3.5 seconds after transitioning to `STARTING` to match the console's physical countdown beeps.
* **1500ms CSAFE Watchdog:** If no valid `0xF2` CSAFE frame is received for 1500ms, internal state is forced to `STOPPED`. This is the primary method for detecting physical e-stop key removal.
* **Incline Timeout:** Maximum continuous incline motion is capped at 60 seconds. Exceeding this triggers a fault state.
* **I2C Graceful Degradation:** If the LSM6DSOX sensor disconnects, cadence defaults to `0 SPM` without triggering a kernel panic.

---

## 9. Pro Interval Coach (Visual Engine)

* **No Automated Control:** The interval engine is strictly visual/auditory (utilizing the Web Audio API). It **never** automates speed or incline hardware injections.
* **Profile Binding:** Interval targets are stored in `profiles.json` and dynamically loaded based on the active user profile.

---

## 10. Open Items

| Item | Status | Priority | Owner |
| :--- | :--- | :--- | :--- |
| Final incline span calibration | Pending physical sweep | **HIGH** | User |
| ~~Fan command verification~~ | ✅ Verified & mapped in Appx A | Closed | User |

---

## Appendix A: Verified Button Map

A button is uniquely identified by its **KEY Byte**, **Frame Sequence**, and active **O1 Row**.

| Function | KEY Byte | Frame Sequence (O2) | O1 Row | Pattern | Line |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Speed +** | `0x23` | `0x01→0x23→0x00→0xFF` | ROW_D | `0x2D` | 4+7 |
| **Speed −** | `0x13` | `0x01→0x13→0x00→0xFF` | ROW_E | `0x1F` | 8 |
| **Incline +** | `0x83` | `0x01→0x83→0x00→0xFF` | ROW_A | `0x3C` | 3+4 |
| **Incline −** | `0x07` | `0x01→0x07→0x00→0xFF` | ROW_C | `0x35` | 4+6 |
| **Quick Start** | `0x0B` | `0x01→0x0B→0x00→0xFF` | ROW_C | `0x35` | 4+6 |
| **Stop** | `0x43` | `0x01→0x43→0x00→0xFF` | ROW_C | `0x35` | 4+6 |
| **Enter** | `0x11` | `0x01→0x11→0x01→0x03→0x00→0xFF` | ROW_C | `0x35` | 4+6 |
| **Clear** | `0x81` | `0x81→0x01→0x03→0x00→0xFF` | ROW_D | `0x2D` | 4+7 |
| **Instant Speed** | `0x03` | `0x01→0x03→0x01→0x03→0x00→0xFF`| ROW_E | `0x1F` | 8 |
| **Instant Incline**| `0x03` | `0x03→0x01→0x03→0x00→0xFF` | ROW_D | `0x2D` | 4+7 |
| **Num 0** | `0x05` | `0x01→0x05→0x01→0x03→0x00→0xFF` | ROW_E | `0x1F` | 8 |
| **Num 1** | `0x41` | `0x41→0x01→0x03→0x00→0xFF` | ROW_E | `0x1F` | 8 |
| **Num 2** | `0x09` | `0x01→0x09→0x01→0x03→0x00→0xFF` | ROW_C | `0x35` | 4+6 |
| **Num 3** | `0x21` | `0x01→0x21→0x01→0x03→0x00→0xFF` | ROW_C | `0x35` | 4+6 |
| **Num 4** | `0x21` | `0x21→0x01→0x03→0x00→0xFF` | ROW_D | `0x2D` | 4+7 |
| **Num 5** | `0x09` | `0x09→0x01→0x03→0x00→0xFF` | ROW_A | `0x3C` | 3+4 |
| **Num 6** | `0x41` | `0x01→0x41→0x01→0x03→0x00→0xFF` | ROW_C | `0x35` | 4+6 |
| **Num 7** | `0x11` | `0x11→0x01→0x03→0x00→0xFF` | ROW_A | `0x3C` | 3+4 |
| **Num 8** | `0x05` | `0x05→0x01→0x03→0x00→0xFF` | ROW_E | `0x1F` | 8 |
| **Num 9** | `0x81` | `0x01→0x81→0x01→0x03→0x00→0xFF` | ROW_D | `0x2D` | 4+7 |
| **Fan High** | `0x09` | `0x01→0x09→0x03→0x00→0xFF` | IDLE | `0x3F` | — |
| **Fan Low** | `0x03` | `0x01→0x03→0x00→0xFF` | IDLE | `0x3F` | — |
| **Fan On/Off** | `0x05` | `0x01→0x05→0x03→0x00→0xFF` | ROW_D | `0x2D` | 4+7 |

> *Note: Num 5 (formerly ROW_B) and Num 7 (formerly ROW_D) have been revised to ROW_A based on raw log analysis showing 111/114 and 107/111 synchronized hits respectively.*
> *Num 6: O1 row assignment (ROW_C) is based on legacy analysis. Fresh raw data analysis was inconclusive due to a lack of recording overlap. This should be verified with a new, simultaneous capture.*
