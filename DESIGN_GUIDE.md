# StrideControl Technical Design Guide

## 1. Project Goal

The goal of the project is to build a safe and reversible ESP32-S3 based interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:

- hosts a local web interface for a tablet
- reads actual speed and incline movement directly from treadmill hardware
- reads treadmill state from the CSAFE port
- broadcasts treadmill telemetry over Bluetooth FTMS
- preserves the original console and treadmill safety behavior
- keeps the original treadmill usable even if the ESP32 is off or rebooting

The project is designed as a non-destructive overlay system, not a full treadmill controller replacement.

---

## 2. System Scope and Main Decisions

### Chosen system role

The ESP32-S3 is responsible for:

- web UI
- telemetry processing
- FTMS broadcasting
- local logging
- future command injection

The treadmill itself remains responsible for:

- belt drive control (3.5 HP AC motor)
- incline motor power
- original safety logic (safety key, emergency stops)
- original console behavior

### Important design choices

- The tablet UI is stateless and only renders live data sent from the ESP32.
- The original treadmill must continue to operate if the ESP32 loses power.
- Raw capacitive touch sensing in the front panel will not be spoofed directly.
- Future command injection will target the front-panel bus after the touch electronics, not the raw capacitive panel itself.

---

## 3. Verified Physical Findings

### 3.1 Console and Wiring

- The console uses a main control PCB marked `APM 900T / APS469B`.
- The front panel is capacitive.
- The RJ45/RS232 interface is the CSAFE port.
- The console-to-treadmill harness has 12 signal conductors, plus 2 separate fan power conductors.

### 3.2 Harness Measurements (Verified)

- **Black wire:** `+11.75V DC`, constant
- **Brown wire:** `0V`, constant (measurement reference / GND)
- **Pin 6:** `5.0V DC`, constant
- **Pin 12:** `5.0V DC`, constant

### Pins with dynamic behavior

#### Pin 7 (Speed)

- `11.4V DC` at rest
- At low speed, DC reading fluctuates rapidly between `5V` and `13V`
- `8.97 Hz` at `10 km/h`
- `22.3 Hz` at `25 km/h`

#### Pin 11 (Incline)

- `4.682V DC` at rest
- averages `2.2V` and outputs a constant `394 Hz` while the incline is physically moving
- returns to approximately `0V` or `4.682V` when motion stops
- pulses **only** while the incline is moving

#### Pin 10

- `0.09V DC` at rest
- not used in the current design

### Unmapped lines

- Pins `3`, `4`, `5`, `8`, and `9` measure at approximately `4.3V DC` in standby
- these are not yet mapped to named functions

### 3.3 Mechanical Incline Behavior

- A `QUICK START` command always drives the incline physically down to `0%`
- Measured incline travel time under user load:
  - `0% -> 15% = 49 seconds`
  - `15% -> 0% = 48 seconds`

This behavior is used as the incline homing baseline in software.

---

## 4. Sensor Interpretation Used by the Project

### 4.1 Speed Input

**Verified:** Pin 7 is used as the actual speed feedback input.

**Reason:**  
It has a stable rest voltage and provides a frequency output directly proportional to belt movement.

**Implementation:**  
Pin 7 is treated as the single source of truth for treadmill actual belt speed.

### 4.2 Incline Input

**Verified:** Pin 11 is used as the incline movement feedback input.

**Reason:**  
It has a stable rest state and emits a pulse train only while the incline motor is physically moving, stopping immediately when movement ceases.

**Implementation:**  
Pin 11 is not treated as a direct analog angle value. Actual incline is calculated in software by homing to `0%` and then counting movement pulses.

---

## 5. Calibration Data and Software Constants

### 5.1 Speed Calibration

### Verified reference points

- `10 km/h = 8.97 Hz`
- `25 km/h = 22.3 Hz`

### Derived factors

- `10 / 8.97 = 1.1148`
- `25 / 22.3 = 1.1211`

### Chosen working constant

```text
Speed (km/h) = Frequency (Hz) * 1.1148
