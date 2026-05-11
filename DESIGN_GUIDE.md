# StrideControl Technical Design Guide (V2.5)

This document contains all verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system (V2.5 Architecture) safely and predictably.

> **Primary User-Facing Goal**
> The primary user-facing goal of StrideControl is to provide a clean, tablet-based web interface mounted on the treadmill for direct speed and incline control. Because the tablet will cover the original console’s speed and incline presentation, the web UI must also show the treadmill’s **actual speed** and **actual incline** as derived from hardware feedback, not just target values.

---

## 1. Project Goal

The goal of the project is to build a safe and reversible `ESP32-S3` based interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:

* Hosts a local web interface for a tablet.
* Allows the user to set treadmill speed and incline directly from that tablet interface.
* Reads actual speed and incline movement directly from treadmill hardware.
* Reads treadmill state from the CSAFE port.
* Broadcasts treadmill telemetry over Bluetooth FTMS.
* Preserves the original console and treadmill safety behavior.
* Keeps the original treadmill usable even if the ESP32 is off or rebooting.

The project is designed as a **non-destructive overlay system**, not a full treadmill controller replacement.

---

## 2. System Scope and Main Decisions

### Chosen System Role
The `ESP32-S3` is responsible for:
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

### Important Design Choices
* The tablet UI is stateless and only renders live data sent from the ESP32.
* The original treadmill must continue to operate if the ESP32 loses power.
* Raw capacitive touch sensing in the front panel will not be spoofed directly.
* Command injection targets the decoded front-panel bus after the touch electronics.

---

## 3. Verified Physical Findings

### 3.1 Console and Wiring
* The console uses a main control PCB marked `APM 900T / APS469B`.
* The front panel is capacitive.
* The RJ45/RS232 interface is the CSAFE port.
* The console-to-treadmill harness has 12 signal conductors, plus 2 separate fan power conductors.

### 3.2 Harness Measurements (Verified)
* **Black wire:** `+11.75V DC`, constant
* **Brown wire:** `0V`, constant (measurement reference / GND)
* **Pin 6:** `5.0V DC`, constant
* **Pin 12:** `5.0V DC`, constant

**Pins with dynamic behavior (Telemetry):**
* **Pin 7 (Speed):** `11.4V DC` at rest. At low speed, DC reading fluctuates rapidly. Emits `8.97 Hz` at 10 km/h, and `22.3 Hz` at 25 km/h.
* **Pin 11 (Incline):** `4.682V` DC at rest. Outputs a constant `394 Hz` while the incline is physically moving. Returns to approx `0V` or `4.682V` when motion stops.
* **Pin 10:** `0.09V DC` at rest. Not used in the current design.

### 3.3 Front-Panel Bus Mapping (Verified)
The post-touch front-panel communication utilizes time-division multiplexing across two buses operating at 5V logic:
* **O1 Bus (Scanner/Listen):** Pins 3, 5, 6, 7, 8 (mapping to Rows A through E). The mainboard pulls these lines low sequentially to scan the matrix.
* **O2 Bus (Data/Inject):** Pins 9 through 16. This is an 8-bit parallel data bus used to transmit key presses back to the mainboard during the specific scanner window.

### 3.4 Mechanical Incline Behavior
* A `QUICK START` command always drives the incline physically down to 0%.
* Measured incline travel time under user load: `0% -> 15% = 49 seconds`; `15% -> 0% = 48 seconds`.
* This behavior is used as the incline homing baseline in software.

### 3.5 Console Button Behavior (Verified)
The physical Speed (+/-) and Incline (+/-) buttons are **ignored** by the treadmill controller unless the belt is actively moving. It is physically impossible to pre-set a target speed or incline via the physical console before starting the treadmill.

---

## 4. Sensor Interpretation Used by the Project

### 4.1 Speed Input
* **Verified:** Pin 7 is used as the actual speed feedback input.
* **Reason:** It has a stable rest voltage and provides a frequency output directly proportional to belt movement.
* **Implementation:** Pin 7 is treated as the single source of truth for actual belt speed. To ensure system stability against electrical noise from the 3.5 HP AC motor, the software implements a **Minimum Pulse Width (blanking time)** to reject physically impossible short pulses. Furthermore, an **Exponential Moving Average (EMA)** filter is applied to the calculated speed to provide smooth telemetry and prevent UI/FTMS "jitter".

### 4.2 Incline Input
* **Verified:** Pin 11 is used as the incline movement feedback input.
* **Reason:** It has a stable rest state and emits a pulse train only while the incline motor is physically moving, stopping immediately when movement ceases.
* **Implementation:** Pin 11 is not treated as a direct analog angle value. Actual incline is calculated in software by counting movement pulses relative to a 0% baseline. To avoid requiring a full homing sequence (0% -> 15%) on every boot, the current pulse count is committed to **Non-Volatile Storage (NVS)** every time the incline motor ceases movement. The system loads this value on boot to immediately resume from its known physical position.

### 4.3 Cadence and Presence Input (LSM6DSOX)
* **Verified:** A single `LSM6DSOX` 6-DoF IMU housed in a protective plastic enclosure, mounted on the lower frame of the treadmill near the running deck.
* **Reason:** The mechanical shock of footstrikes transfers through the frame sufficiently for the IMU's built-in hardware step counter to derive highly accurate running cadence (strides per minute) without relying on motor load spikes. It also acts as a presence detector.
* **Implementation:** Communicates via I2C. Polled asynchronously to avoid blocking the main interrupt-driven motor sensors.

---

## 5. Calibration Data and Software Constants

### 5.1 Speed Calibration
* **Verified reference points:** `10 km/h = 8.97 Hz`; `25 km/h = 22.3 Hz`.
* **Derived factors:** `10 / 8.97 = 1.1148`; `25 / 22.3 = 1.1211`.
* **Chosen working constant:** `Speed (km/h) = Frequency (Hz) * 1.1148`. Anchored to the 10 km/h reference point.
* **Derived odometer constant:** `1 meter = 3.2292 pulses`.
* **Dynamic Speed Injection (AutoCal):** Linear interpolation calculates precise offsets for requested target speeds. Offsets are saved to LittleFS only when the treadmill stops to minimize flash wear.

### 5.2 Incline Calibration
* **Verified facts:** Pin 11 emits 394 Hz while moving. `QUICK START` homes to 0%. Full travel is approx 49s upward, 48s downward.
* **Chosen software rule:** `QUICK START` homes incline to 0% if NVS state is missing or invalid. A final fixed pulses-per-percent constant is pending final physical sweep validation.

---

## 6. Hardware Interface Choices

### 6.1 MCU and Isolation
* **Controller:** ESP32-S3 (Dual-Core). 
* **Hardware Requirement:** **N16R8** (16MB Flash, 8MB PSRAM) is strictly required. The PSRAM is critical to support the dual-role BLE stack, WebSockets, and `ArduinoJson` serialization without heap fragmentation over time. The 16MB Flash provides sufficient space for the custom partition table handling the LittleFS web assets and OTA updates.
* **Speed Isolation (Pin 7):** 100% optically isolated using a PC817 optocoupler. While the PC817 has microsecond-level switching latency, the treadmill's low frequency output (8-25 Hz) makes this negligible. The ESP32 reads the signal via an internal pull-up (INPUT_PULLUP), and software-level blanking completely mitigates any edge jitter, fully protecting the 3.3V logic from the treadmill's high-voltage/noisy motor controller.
* **Incline Interface (Pin 11):** Stepped down to a safe 3.3V logic level using a BSS138 MOSFET bi-directional level shifter.
* **Logic Level Translation:** Two `TXS0108E` 8-channel bi-directional level shifters safely bridge the 5V O1 and O2 buses to the 3.3V ESP32 GPIOs.

### 6.2 The "Data Gate" (Mute Circuit)
To prevent bus contention during command injection, a physical interrupt circuit is utilized.
* **Hardware:** 2x `SN74HC4066N` (Quad Bilateral Switch) placed inline on the 8-bit O2 data bus.
* **Operation:** During normal use, the switches are closed (HIGH), connecting the physical console to the mainboard. During injection, the ESP32 drives the control pins LOW, disconnecting the console data lines for approximately 3 milliseconds while the ESP32 writes to the bus.
* **Fail-Safe Requirement:** A 10 kΩ pull-up resistor ensures the switches remain closed by default. The original console path remains fully functional if the ESP32 loses power or crashes.

### 6.3 Modular PCB Architecture (Two-Board Strategy)
To isolate high-voltage/motor noise from the sensitive data bus interception, the hardware is split across two 3x7 cm perfboards:
* **Board 1 (Interceptor Shield):** Placed strictly in-line with the 16-pin console ribbon cable. Houses the `SN74HC4066N` switches and the fail-safe pull-up logic.
* **Board 2 (Core Brain):** Houses the ESP32-S3, `HLK-PM01` power supply, logic-level converters, and the PC817/BSS138 optocouplers. Connected to Board 1 via soft, stranded silicone wires to withstand mechanical vibration.

### 6.4 I2C Sensor Networking
The LSM6DSOX sensor requires a 2-meter cable run from the frame to the motor compartment.
* **Cabling:** A repurposed, shielded USB cable is used for the main run to protect I2C signals (SDA/SCL) from the 3.5 HP AC motor's EMI.
* **Terminals:** Sensor connections at the frame are spliced using vibration-proof Wago 221 connectors. The ESP32 enclosure uses a 4-pin GX12 Aviation plug for modularity.
* **Active Termination:** An `LTC4311` I2C Extender is installed on Board 2 to actively drive the bus and overcome the capacitance of the long cable.

### 6.5 Power Supply
* **Hardware:** `HLK-PM01` AC-to-5V module. Provides permanent internal power without relying on external USB. Physical isolation and mains-side fusing are required.

### 6.6 Bootstrap, Recovery and Local Configuration
The device must be deployable and recoverable without requiring a USB cable. First boot, or missing WiFi credentials, forces Access Point mode to expose a local configuration page (WiFi setup, machine profile, FTMS toggle).

---

## 7. Software Architecture Rules

### 7.1 Dual-Core Task Split
* **Core 0:** WiFi, LittleFS web server, WebSocket communication, Bluetooth FTMS, system logging.
* **Core 1:** Pulse counting interrupts (Pin 7, Pin 11), real-time GPIO injection tasks, safety watchdog tasks.
* **Thread Safety & Data Synchronization:** Shared data structures between Core 0 and Core 1 MUST be protected by a global hardware spinlock (`portMUX_TYPE`).

### 7.2 Safety and Logging
* **Watchdog:** A 1.0-second hardware watchdog timer (WDT) must be enabled.
* **Logging:** State transitions, FTMS events, CSAFE anomalies, and timeouts are logged to a local buffer.
* **I2C Graceful Degradation:** The firmware must handle sudden I2C bus failures (e.g., cable disconnection) gracefully. A sensor timeout or bus error must not trigger a kernel panic or halt Core 1 interrupts. If the cadence sensor is lost, the system shall default to reporting `0 SPM` and continue uninterrupted speed, incline, and FTMS operations.

### 7.3 Command Injection and Math Logic
* **Hardware-Synced Injection:** The ESP32 MUST attach hardware interrupts (`FALLING` edge) to the specific O1 Row pins (Pin 6 for Speed, Pin 4 for Incline). Command injection on the O2 bus only occurs within the ~3ms window when the target row is actively scanned by the treadmill's mainboard.
* **Injection Timing:** Injection is asynchronous. User commands queue on Core 0. Core 1 waits for the exact O1 hardware interrupt (ISR), drops the 74HC4066N gate, executes the 8-bit hexadecimal frame payload on the O2 pins, and releases the gate.
* **Fractional Target Math (Nearest Integer Optimization):** To bypass the console's whole-number limitations while minimizing the number of injected frames on the O2 bus, the software uses `round()` logic to find the shortest path to the target. 
  * *Example:* A target of `12.6 km/h` injects the macro for the nearest integer (`13`), followed by exactly 4 discrete `Speed -` decrements. 
  * This optimization significantly reduces the total injection time and UI latency compared to strictly adding upon `floor()` values.
* **Elevation Gain Integration:** The FTMS specification requires Positive Elevation Gain, which the hardware does not natively provide. The software must algorithmically accumulate this in the main loop by integrating the current incline percentage over the distance traveled during each tick (`Elevation Gain = Distance_interval * (Incline / 100)`).
* **Fan Quarantine:** Fan control commands are disabled (`TODO` status).

### 7.4 Telemetry Rate Control and Backpressure
Real-time pulse capture, WebSocket rendering, CSAFE parsing, and FTMS broadcasting must not compete for timing. Hardware pulse counting is interrupt-driven; no UI work occurs inside ISRs. Downstream consumers read from internal state at fixed rates.

### 7.5 Configuration Classes and Persistent Settings
The firmware shall separate persistent values into classes:
1. **Hardware calibration:** Speed factor, pulses per meter, incline span, homing parameters.
2. **Device preferences:** WiFi settings, FTMS device name, debug mode.
3. **User profile settings:** Drag and Rest speed/incline defaults, and last-used interval configurations (work/rest durations, series, and reps).
4. **Runtime-only state:** Raw counters, current treadmill state.

> **Safety rule:** Runtime-only state must never be treated as durable calibration data.

---

## 8. CSAFE Rules (RS232)

* **Connection:** RJ45 to MAX3232 (9600 baud, 8-N-1).
* **Polling Structure:** Split into Keep-alive (`0xF1 0x85 0x85 0xF2`) and Status request (`0xF1 0x80 0x80 0xF2`) at 200-250 ms intervals.
* **Forbidden Commands:** `0xAA`, `0xA5`, `0x9C` are not used due to observed buffer corruption on the DK-City mainboard.
* **State Decoding:** Raw states map strictly to internal `SystemState` enums (`STOPPED`, `PAUSED`, `STARTING`, `RUNNING`, `IDLE`).
* **CSAFE Transport-Layer Contract:** Application code must never read directly from UART. Handled as a dedicated transport/parser layer.

---

## 9. Timing and Safety Rules

* **Incline Timeout:** Maximum continuous incline motion is 60 seconds. Exceeding this triggers an emergency fault state.
* **Start-Up Synchronization:** Upon entering State 8 (Starting), the ESP32 ignores speed/distance accumulation for 3.5 seconds.
* **Resume Debounce:** A 700 ms debounce filter applies when transitioning from Paused to InUse.
* **Ghost-Running Detection:** If CSAFE reports InUse but Pin 7 reads 0 Hz for 2.0 seconds, the UI resets and logs a mismatch fault.
* **CSAFE RX Watchdog:** No valid `0xF2` frame for 1500 ms forces the internal state to STOPPED.
* **Command Queuing (5-Second Rule):** Commands issued during STARTING are queued. Injected once RUNNING is reached. Expires after 5 seconds if RUNNING is not achieved.

---

## 10. UI and Data Storage Rules

### 10.1 Tablet UI
* The web UI is the primary control surface. It stores no treadmill state locally; the ESP32 sends live JSON via WebSockets (5-10 Hz). 
* To prevent heap fragmentation and memory leaks during rapid telemetry streaming, payload serialization must be strictly handled by a deterministic JSON library (e.g., `ArduinoJson` using statically allocated memory buffers).
* Must display actual speed and actual incline natively derived from hardware alongside requested targets.

### 10.2 Filesystem, Web UI and OTA Policy
* HTML, CSS, JS, and `profiles.json` are stored in LittleFS.
* **Flash Wear Policy:** Odometer and persistent state (such as the current incline NVS pulse count) are only written to flash when the treadmill state changes to STOPPED, minimizing write cycles.
* **OTA Rules:** Firmware OTA and filesystem OTA are separate artifacts.
* **Custom Partition Table:** The default ESP32 partition scheme is insufficient for a dual-role BLE stack combined with an Async Web Server. A custom partition table must be explicitly defined in the build environment. Leveraging the N16R8 hardware, the layout must allocate at least 1.8MB for the `app0` binary partition and a dedicated partition for `LittleFS` to prevent compilation overflow and ensure robust local asset storage.

---

## 11. Bluetooth FTMS Rules

* **BLE Stack:** NimBLE-Arduino (Dual-Role operation required).
* **Broadcast Rate:** Exactly 1 Hz.
* **Data Formatting:** Cadence (LSM6DSOX) is included. Speed is sent as integer with 0.01 resolution (12.50 km/h = 1250). Incline is sent as integer with 0.1 resolution (3.5% = 35).
* **Connection Policy:** FTMS exposed as a dedicated server. Disconnect events reset transient state. Loss of a BLE client must not corrupt treadmill state tracking.
* **Control Points:** External target requests route through the internal macro queue system, functioning identically to local UI inputs.

---

## 12. GUI Architecture and Frontend Logic

* **Technical Frontend Stack:** Vanilla JavaScript only. Frameworks (React/Vue/etc.) are **strictly prohibited** to ensure the files remain small enough for the LittleFS partition and to keep the browser memory footprint minimal.
* **Layout Engine:** 5-Row Symmetrical CSS Grid. Forces primary telemetry to the mathematical center.
* **Smart Telemetry:** Distance capped at 10-meter resolution (0.00 km). `font-variant-numeric: tabular-nums` strictly enforced.
* **Interaction Model:** Uses `pointerdown` events to bypass mobile browser lag. Employs CSS brightness filters for haptic visual feedback.
* **Contextual Logic:** Status overlays absolute positioned. Setup and User Profile selectors disabled during STARTING or RUNNING.
* **Heart Rate Proxy & BLE Discovery:** The interface implements a discovery engine. Workflow: User initiates "SCAN FOR DEVICES". ESP32 returns a JSON list of available BLE heart rate monitors (e.g., "Polar H10"). The ESP32 connects to the selected belt as a BLE Client and acts as an HR Proxy, natively forwarding the pulse data seamlessly within the combined FTMS broadcast signal. Connection status is persisted in NVS for automatic reconnection.

### 12.1 Pro Interval Coach (Visual Guidance Engine)
To support advanced structured workouts without compromising the safety and predictability of manual control, the UI implements a standalone "Visual Coach".

* **No Automatic Actuation:** The interval engine is purely visual and auditory. It tracks phases (Work, Rest, Cooldown) and prompts the user, but **never** injects speed or incline commands automatically. The user remains in full control via the persistent `HVILE` and `DRAG` macro buttons.
* **Focus Mode UI:** When a workout begins, the DOM transitions into a `focus-mode` state via CSS class toggling. Telemetry data shrinks and moves to the periphery, while the countdown timer scales up significantly in the mathematical center. 
* **Dynamic Workout Graph:** A CSS-based flexbox graph dynamically generates a timeline of the session (including nested series and series-rests), providing peripheral progression feedback.
* **Web Audio API Cues:** To support "hands-free" and "eyes-free" operation, the UI utilizes the native browser Web Audio API to generate synthetic oscillator beeps (3-2-1 countdowns and phase-shift alerts), avoiding external `.mp3` dependencies.
* **On-the-Fly Adjustments:** The engine exposes emergency UI controls during active intervals:
  * **Lifebuoy (+30s):** Allows the user to dynamically extend a resting phase without breaking the workout structure.
  * **Skip Phase (⏭):** Allows the user to truncate a phase and immediately jump to the next block.
* **Profile-Linked Persistence:** The interval engine must remember the user's last configured workout. When a user is selected via the Active User Indicator, the interval modal automatically pre-loads their specific last-used settings (Work/Rest durations, Reps, Series) from `profiles.json` (or `localStorage`), preventing repetitive data entry.
* **Signature Workouts:** The engine includes predefined, hardcoded workout templates (e.g., "Olympiatoppen 4x4", "Pyramid") that dynamically populate the workout array to override manual step inputs.
* **Font Constraints:** To prevent rendering artifacts on the primary timer and action buttons using the *Barlow Condensed* font, the Norwegian letter "Ø" is strictly avoided in the UI copy (e.g., using "AVBRYT" instead of "KLARGJØR ØKT").

### 12.2 Interval Data Structures (Examples)
To ensure consistent parsing between the web UI and the LittleFS backend, interval configurations follow strict JSON schemas.

**1. User Profile Persistence (`profiles.json`)**
Each user object in the profiles array stores their `last_interval` settings alongside their speed presets. When a user is selected, the UI populates the modal fields using this exact object:

 ```json
{
  "users": [
    {
      "name": "Kristian",
      "presets": { "hvile": 6.0, "drag": 16.0 },
      "last_interval": {
        "work_m": 0,
        "work_s": 45,
        "rest_m": 0,
        "rest_s": 15,
        "reps": 10,
        "series": 2,
        "series_rest_m": 3
      }
    }
  ]
}
```
**2. The Dynamic Schedule Array (In-Memory Engine)**
Regardless of whether the user inputs manual fields or selects a "Signature Workout" (like a Pyramid), the UI JavaScript compiles the workout into a flat, sequential array of objects before execution. This makes it trivial to implement complex workouts.

Example: The first 3 steps of a Pyramid Workout:

```json
[
  { "type": "work", "time": 60, "name": "DRAG 1 MIN" },
  { "type": "rest", "time": 60, "name": "PAUSE" },
  { "type": "work", "time": 120, "name": "DRAG 2 MIN" },
  { "type": "rest", "time": 60, "name": "PAUSE" }
]
```

Note: time is always evaluated in total seconds. The type key dictates both CSS graph coloring (red/green) and the Audio Cue triggered upon entering the phase.

---

## 13. Current Open Items

* Final incline span calibration from measured 0% -> 15% pulse counting.
* Verification of Fan command frame payloads via logic analyzer to remove software quarantine.

---

## 14. Future Development

* **Physical Console Key Synchronization:** Utilizing the mapped O1/O2 buses, the ESP32 will listen for physical button activity using interrupts to synchronize the web UI with physical inputs.
* **Auto-Incline via FTMS (Two-Way):** Configure the FTMS Control Point so that elevation data from apps like Zwift can automatically translate into "Incline +/-" injections on the O2 bus.
* **Observability and Remote Debug:** Expose structured logs for boot sequence, transport state, FTMS events, and command injection attempts over the network.

---

## 15. Current Project Status

### Verified and Implemented
* Actual speed reading from Pin 7 (filtered via EMA/Blanking) and incline reading from Pin 11 (NVS persistent).
* Console lockout behavior verified.
* CSAFE state polling logic and fragmentation handling.
* Stateless tablet UI architecture.
* HR Proxy functionality and Bluetooth FTMS via NimBLE (Dual-Role).
* Decoded front-panel post-touch bus mapping (O1/O2 buses).
* Final command injection method via 74HC4066N Data Gate and Core 1 ISRs.

### Chosen and Fixed for Implementation
* `ESP32-S3` as controller (dual-core task split). Required hardware profile: **N16R8**.
* 100% Optical Isolation for Speed (`PC817`) and Incline (`BSS138`).
* I2C Active Termination (`LTC4311`) for Cadence/Presence tracking (`LSM6DSOX`).
* 2x `TXS0108E` logic level translators for O1/O2 buses.
* `HLK-PM01` for permanent power.
* Deterministic memory handling via statically allocated `ArduinoJson` buffers.
* Custom Partition Table for dual-role BLE and Web Server stability.
* Algorithmic Positive Elevation Gain calculation for FTMS compliance.
