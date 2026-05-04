# StrideControl Technical Design Guide

This document contains all verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system (V2.2 Architecture) safely and predictably.

## 1. Project Goal
The goal of the project is to build a safe and reversible ESP32-S3 based interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:
* Hosts a local web interface for a tablet.
* Allows the user to set treadmill speed and incline directly from that tablet interface.
* Reads actual speed and incline movement directly from treadmill hardware.
* Reads treadmill state from the CSAFE port.
* Broadcasts treadmill telemetry over Bluetooth FTMS.
* Preserves the original console and treadmill safety behavior.
* Keeps the original treadmill usable even if the ESP32 is off or rebooting.

The project is designed as a non-destructive overlay system, not a full treadmill controller replacement.

### Primary User-Facing Goal
The primary user-facing goal of StrideControl is to provide a clean, tablet-based web interface mounted on the treadmill for direct speed and incline control. Because the tablet will cover the original console’s speed and incline presentation, the web UI must also show the treadmill’s actual speed and actual incline as derived from hardware feedback, not just target values.

## 2. System Scope and Main Decisions

**Chosen system role**
The ESP32-S3 is responsible for:
* Web UI
* Telemetry processing
* FTMS broadcasting
* Local logging
* Command injection

The treadmill itself remains responsible for:
* Belt drive control (3.5 HP AC motor)
* Incline motor power
* Original safety logic (safety key, emergency stops)
* Original console behavior

**Important design choices**
* The tablet UI is stateless and only renders live data sent from the ESP32.
* The original treadmill must continue to operate if the ESP32 loses power.
* Raw capacitive touch sensing in the front panel will not be spoofed directly.
* Command injection targets the decoded front-panel bus after the touch electronics.

## 3. Verified Physical Findings

### 3.1 Console and Wiring
* The console uses a main control PCB marked APM 900T / APS469B.
* The front panel is capacitive.
* The RJ45/RS232 interface is the CSAFE port.
* The console-to-treadmill harness has 12 signal conductors, plus 2 separate fan power conductors.

### 3.2 Harness Measurements (Verified)
* **Black wire:** +11.75V DC, constant
* **Brown wire:** 0V, constant (measurement reference / GND)
* **Pin 6:** 5.0V DC, constant
* **Pin 12:** 5.0V DC, constant

**Pins with dynamic behavior (Telemetry)**
* **Pin 7 (Speed):** 11.4V DC at rest. At low speed, DC reading fluctuates rapidly. Emits 8.97 Hz at 10 km/h, and 22.3 Hz at 25 km/h.
* **Pin 11 (Incline):** 4.682V DC at rest. Outputs a constant 394 Hz while the incline is physically moving. Returns to approx 0V or 4.682V when motion stops.
* **Pin 10:** 0.09V DC at rest. Not used in the current design.

### 3.3 Front-Panel Bus Mapping (Verified)
The post-touch front-panel communication utilizes time-division multiplexing across two buses operating at 5V logic:
* **O1 Bus (Scanner/Listen):** Pins 3, 5, 6, 7, 8 (mapping to Rows A through E). The mainboard pulls these lines low sequentially to scan the matrix.
* **O2 Bus (Data/Inject):** Pins 9 through 16. This is an 8-bit parallel data bus used to transmit key presses back to the mainboard during the specific scanner window.

### 3.4 Mechanical Incline Behavior
* A QUICK START command always drives the incline physically down to 0%.
* Measured incline travel time under user load: 0% -> 15% = 49 seconds; 15% -> 0% = 48 seconds.
* This behavior is used as the incline homing baseline in software.

### 3.5 Console Button Behavior (Verified)
The physical Speed (+/-) and Incline (+/-) buttons are ignored by the treadmill controller unless the belt is actively moving. It is physically impossible to pre-set a target speed or incline via the physical console before starting the treadmill.

## 4. Sensor Interpretation Used by the Project

### 4.1 Speed Input
* **Verified:** Pin 7 is used as the actual speed feedback input.
* **Reason:** It has a stable rest voltage and provides a frequency output directly proportional to belt movement.
* **Implementation:** Pin 7 is treated as the single source of truth for treadmill actual belt speed.

### 4.2 Incline Input
* **Verified:** Pin 11 is used as the incline movement feedback input.
* **Reason:** It has a stable rest state and emits a pulse train only while the incline motor is physically moving, stopping immediately when movement ceases.
* **Implementation:** Pin 11 is not treated as a direct analog angle value. Actual incline is calculated in software by homing to 0% and then counting movement pulses.

## 5. Calibration Data and Software Constants

### 5.1 Speed Calibration
* **Verified reference points:** 10 km/h = 8.97 Hz; 25 km/h = 22.3 Hz.
* **Derived factors:** 10 / 8.97 = 1.1148; 25 / 22.3 = 1.1211.
* **Chosen working constant:** Speed (km/h) = Frequency (Hz) * 1.1148. Anchored to the 10 km/h reference point.
* **Derived odometer constant:** 1 meter = 3.2292 pulses.
* **Dynamic Speed Injection (AutoCal):** Linear interpolation calculates precise offsets for requested target speeds. Offsets are saved to LittleFS only when the treadmill stops to minimize flash wear.

### 5.2 Incline Calibration
* **Verified facts:** Pin 11 emits 394 Hz while moving. QUICK START homes to 0%. Full travel is approx 49s upward, 48s downward.
* **Chosen software rule:** QUICK START homes incline to 0%. Pulse counting starts from that baseline. A final fixed pulses-per-percent constant is pending final physical sweep validation.

## 6. Hardware Interface Choices

### 6.1 MCU and Isolation
* **Controller:** ESP32-S3 (Dual-Core).
* **Speed Isolation (Pin 7):** Read via GPIO 3 with a 10 kOhm series resistor for protection.
* **Incline Interface (Pin 11):** Scaled from 4.68V to ~3.1V using a 10 kOhm / 20 kOhm voltage divider.
* **Logic Level Translation:** Two TXS0108E 8-channel bi-directional level shifters safely bridge the 5V O1 and O2 buses to the 3.3V ESP32 GPIOs.

### 6.2 The "Data Gate" (Mute Circuit)
To prevent bus contention during command injection, a physical interrupt circuit is utilized.
* **Hardware:** 2x SN74HC4066N (Quad Bilateral Switch) placed inline on the 8-bit O2 data bus.
* **Operation:** During normal use, the switches are closed (HIGH), connecting the physical console to the mainboard. During injection, the ESP32 drives the control pins LOW, disconnecting the console data lines for approximately 3 milliseconds while the ESP32 writes to the bus.
* **Fail-Safe Requirement:** The 5V power supply to the console is never interrupted. The default state allows the original console path to remain functional if the ESP32 loses power.

### 6.3 Power Supply
* **Hardware:** HLK-PM01 AC-to-5V module. Provides permanent internal power without relying on external USB. Physical isolation and mains-side fusing are required.

### 6.4 Bootstrap, Recovery and Local Configuration
The device must be deployable and recoverable without requiring a USB cable. First boot, or missing WiFi credentials, forces Access Point mode to expose a local configuration page (WiFi setup, machine profile, FTMS toggle).

## 7. Software Architecture Rules

### 7.1 Dual-Core Task Split
* **Core 0:** WiFi, LittleFS web server, WebSocket communication, Bluetooth FTMS, system logging.
* **Core 1:** Pulse counting interrupts (Pin 7, Pin 11), real-time GPIO injection tasks, safety watchdog tasks.
* **Thread Safety & Data Synchronization:** Shared data structures between Core 0 and Core 1 MUST be protected by a global hardware spinlock (`portMUX_TYPE`).

### 7.2 Safety and Logging
* **Watchdog:** A 1.0-second hardware watchdog timer (WDT) must be enabled.
* **Logging:** State transitions, FTMS connection events, CSAFE anomalies, and timeouts are logged to a local buffer.

### 7.3 Command Injection and Math Logic
* **Injection Timing:** Injection is asynchronous. User commands queue on Core 0. Core 1 waits for the exact O1 hardware interrupt (ISR), drops the 74HC4066N gate, executes the 8-bit hexadecimal frame payload on the O2 pins, and releases the gate.
* **Fractional Target Math:** To bypass the console's whole-number limitations, the software uses integer `floor` logic. A target of 12.6 km/h injects the macro for `12`, followed by exactly 6 discrete `Speed +` increments.
* **Fan Quarantine:** Fan control commands are disabled (`TODO` status). Log analysis shows potential frame overlaps between Fan Low and Instant Speed (`0x03`). Fan injection remains quarantined pending definitive logic analyzer confirmation.

### 7.4 Telemetry Rate Control and Backpressure
Real-time pulse capture, WebSocket rendering, CSAFE parsing, and FTMS broadcasting must not compete for timing. Hardware pulse counting is interrupt-driven; no UI work occurs inside ISRs. Downstream consumers (WebSockets, FTMS) read from internal state at fixed rates.

### 7.5 Configuration Classes and Persistent Settings
The firmware shall separate persistent values into at least these classes:
* **Hardware calibration:** Speed factor, pulses per meter, incline span, homing parameters.
* **Device preferences:** WiFi settings, FTMS device name, debug mode.
* **User profile settings:** Drag target defaults, pause target defaults, preset values.
* **Runtime-only state:** Raw counters, current treadmill state. 
* **Safety rule:** Runtime-only state must never be treated as durable calibration data.

## 8. CSAFE Rules (RS232)

* **Connection:** RJ45 to MAX3232 (9600 baud, 8-N-1).
* **Polling Structure:** Split into Keep-alive (`0xF1 0x85 0x85 0xF2`) and Status request (`0xF1 0x80 0x80 0xF2`) at 200-250 ms intervals.
* **Forbidden Commands:** 0xAA, 0xA5, 0x9C are not used due to observed buffer corruption on the DK-City mainboard.
* **State Decoding:** Treadmill state extracted by masking payload byte with `0x0F`. Raw states map strictly to internal `SystemState` enums (STOPPED, PAUSED, STARTING, RUNNING, IDLE).
* **Data Integrity:** Frame validity is determined by detecting `0xF1` start frames and `0xF2` end frames. Checksum validation is not treated as a hard reject condition.
* **CSAFE Transport-Layer Contract:** CSAFE handling shall be implemented as a dedicated transport/parser layer, not as ad-hoc serial reads. The stack must be divided into: UART transport, frame detection/reassembly, command scheduler, response matcher, raw state extractor, and treadmill-specific state translator. Application code must never read directly from UART.

## 9. Timing and Safety Rules

* **Incline Timeout:** Maximum continuous incline motion is 60 seconds. Exceeding this triggers an emergency fault state.
* **Start-Up Synchronization:** Upon entering State 8 (Starting), the ESP32 ignores speed/distance accumulation for 3.5 seconds to align with the console countdown.
* **Resume Debounce:** A 700 ms debounce filter applies when transitioning from Paused to InUse.
* **Ghost-Running Detection:** If CSAFE reports InUse but Pin 7 reads 0 Hz for 2.0 seconds, the UI resets and logs a mismatch fault.
* **CSAFE RX Watchdog:** No valid `0xF2` frame for 1500 ms forces the internal state to STOPPED.
* **Command Injection Lockout:** Injector discards speed/incline adjustments unless the internal state is RUNNING.
* **Command Queuing (5-Second Rule):** Commands issued during STARTING are queued. Once RUNNING is reached, they are injected. If RUNNING is not reached within 5 seconds, the queue expires.

## 10. UI and Data Storage Rules

### 10.1 Tablet UI
* The web UI is the primary control surface. It stores no treadmill state locally; the ESP32 sends live JSON via WebSockets (5-10 Hz).
* Must display actual speed and actual incline natively derived from hardware alongside requested targets.
* **UI Feedback:** Uses native treadmill acoustic feedback (beeps) to confirm queued commands during the STARTING phase to minimize visual clutter.

### 10.2 Filesystem, Web UI and OTA Policy
* HTML, CSS, JavaScript, and configuration files (`profiles.json`) are stored in LittleFS.
* **First-Boot Formatting:** The storage manager uses `LittleFS.begin(true)` to format uninitialized flash.
* **Flash Wear Policy:** Odometer and persistent state are only written to flash when the treadmill state changes to STOPPED.
* **OTA Rules:** Firmware OTA and filesystem OTA shall be treated as separate artifacts. The design guide must define when a filesystem update is mandatory together with firmware. A failed OTA must not leave the system without a recovery path.

## 11. Bluetooth FTMS Rules

* **BLE Stack:** NimBLE-Arduino (Dual-Role operation required).
* **Broadcast Rate:** Exactly 1 Hz.
* **Connection Policy:** FTMS exposed as a dedicated server. Disconnect events reset transient state. Loss of a BLE client must not corrupt treadmill state tracking.
* **Data Formatting:** Speed sent as integer with 0.01 resolution (12.50 km/h = 1250). Incline sent as integer with 0.1 resolution (3.5% = 35).
* **Control Points:** External target requests route through the internal macro queue system, functioning identically to local UI inputs.

## 12. GUI Architecture and Frontend Logic

* **Layout Engine:** 5-Row Symmetrical CSS Grid. Forces primary telemetry to the mathematical center.
* **Smart Telemetry:** Distance capped at 10-meter resolution (0.00 km) to prevent visual strobe effects. `font-variant-numeric: tabular-nums;` strictly enforced.
* **Interaction Model:** Uses `pointerdown` events to bypass mobile browser lag. Employs CSS brightness filters for immediate haptic visual feedback without triggering layout shifts.
* **Technical Frontend Stack:** Vanilla JavaScript only. Frameworks (React/Vue) are prohibited. Bi-directional data exchange uses a single WebSocket stream.
* **Contextual Logic & Precision UI:** Status overlays are absolute positioned. Setup and User Profile selectors are disabled during STARTING or RUNNING states. Technical terminology (e.g., 'ESP32') shall strictly not be exposed to the user. Textual status overlays in the center grid are strictly limited to the 'P A U S E' state; IDLE, STOPPED, or READY states shall remain text-free to minimize visual clutter.
* **Heart Rate Belt & Bluetooth LE (BLE) Discovery:** 
  * **Device Management:** The interface shall not use a simple toggle. It must implement a discovery engine.
  * **Workflow:** User initiates "SCAN FOR DEVICES". ESP32 returns a JSON list of available BLE device names and IDs. UI populates a selection list (e.g., "Polar H10", "Garmin HRM").
  * **Persistence:** Connection status is persisted in the ESP32 NVS/LittleFS for automatic reconnection in future sessions.

## 13. Current Open Items

* Final incline span calibration from measured 0% -> 15% pulse counting.
* Verification of Fan command frame payloads via logic analyzer to remove software quarantine.

## 14. Future Development

* **Physical Console Key Synchronization:** Utilizing the mapped O1/O2 buses, the ESP32 will listen for physical button activity using interrupts to synchronize the web UI with physical inputs.
* **Observability and Remote Debug:** Expose structured logs for boot sequence, transport state, FTMS events, and command injection attempts over the network.

## 15. Current Project Status

**Verified and Implemented**
* Actual speed reading from Pin 7 and incline reading from Pin 11.
* Incline homing to 0% via QUICK START.
* Console lockout behavior verified.
* CSAFE state polling logic and fragmentation handling.
* Stateless tablet UI architecture (Locked 5-row mathematical grid).
* Bluetooth FTMS via NimBLE (Dual-Role).
* Command queuing logic defined for UI inputs during STARTING state.
* Decoded front-panel post-touch bus mapping (O1/O2 buses).
* Final command injection method via 74HC4066N Data Gate and Core 1 ISRs.
* Fractional target math using integer `floor` logic.

**Chosen and Fixed for Implementation**
* ESP32-S3 as controller (dual-core task split).
* GPIO 3 with 1k/10k series protection for speed input.
* 10k/20k voltage divider for incline input.
* 2x TXS0108E logic level translators for O1/O2 buses.
* HLK-PM01 for permanent power.
