# StrideControl Technical Design Guide (V3.1)

This document contains verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system (V3.1 Architecture) safely and predictably.

> **Primary UI Goal**
> StrideControl provides a clean, tablet-based web interface for direct speed and incline control. Because the tablet completely covers the original console, the UI must display the treadmill's **actual speed** and **actual incline** derived from hardware feedback, not just requested target values.

---

## 1. Project Goal

The objective is to build a safe, reversible ESP32-S3 interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:

- Hosts a local web interface for a tablet.
- Allows direct user control of speed and incline.
- Reads actual physical state directly from the hardware.
- Parses machine state from the CSAFE port.
- Broadcasts telemetry via Bluetooth FTMS.
- Preserves original hardware safety features (the console must work if the ESP32 loses power).

This is a **non-destructive overlay system**, not a replacement for the main motor controller.

---

## 2. Safety & System Integrity Principles

### 2.1 Safety Design Principle

The system is designed under the assumption that failures may occur during operation.

All components must behave safely under fault conditions, including:

- MCU crashes
- Loss of communication
- Sensor failure
- Injection failure

The original treadmill control path must remain functional under all failure modes.

### 2.2 Real-Time Constraint Rule

Network, UI, and Bluetooth processing must never delay or interfere with real-time hardware handling.

All timing-critical logic (sensor capture, injection timing) is isolated on Core 1.

### 2.3 Fail-Safe Path Requirement

The MitM hardware must default to a passive pass-through state when unpowered.

Loss of ESP32 power must never interrupt:

- Console communication
- Safety functions
- Normal treadmill operation

### 2.4 ISR Design Rule

All Interrupt Service Routines must be:

- Non-blocking (no `delayMicroseconds()`, no waiting)
- Constant-time execution
- Limited to timestamping and minimal validation

No logging, UI updates, networking, or serial I/O is allowed inside ISRs.

---

## 3. System Scope & Architecture

- **Role Split:** The ESP32 handles the UI, telemetry, FTMS, and command injection. The treadmill's mainboard retains full control over the 3.5 HP AC drive motor, incline motor power, and safety logic (e-stop key).
- **Core Design:** The web UI is stateless. Physical console buttons are not spoofed at the capacitive level; instead, command injection happens on the decoded 5V logic bus after the touch-controller IC.

---

## 4. Protocol & Hardware Mapping

### 4.1 Harness Measurements (Verified)

- **Black Wire:** +11.75V DC (constant)
- **Brown Wire:** 0V (GND / reference)
- **Pin 6:** 5.0V DC (constant)
- **Pin 12:** 5.0V DC (constant)

**Pins with dynamic behavior (Telemetry):**

- **Pin 7 (Speed):** 11.4V DC at rest. Emits 8.97 Hz at 10 km/h and 22.3 Hz at 25 km/h.
- **Pin 11 (Incline):** 4.682V DC at rest. Outputs a constant 394 Hz while the incline motor is physically moving. Returns to rest voltage when motion stops.
- **Pin 10:** 0.09V DC at rest. Not used in the current design.

> **Historical note (still useful):** Several harness lines can measure ~4.3V in standby depending on probe/reference and scanning activity. If mapping is incomplete, document remaining unmapped pins explicitly.

### 4.2 Front-Panel Bus Mapping

Console communication uses time-division multiplexing across two 5V logic buses:

- **O1 Bus (Scanner):** Pins 3, 4, 5, 6, 7, 8. Line 4 acts as a shared active line, pulled LOW during ROW_A through ROW_D scans. Scan rate: ~167 Hz per row, full cycle ~6 ms.
- **O2 Bus (Data/Inject):** Pins 9 through 16. An 8-bit parallel data bus used to return key presses during specific O1 scan windows. Frame rate: ~330 Hz.

**O1 Row Scan Sequence:** ROW_B → ROW_C → ROW_D → ROW_E → ROW_A → IDLE → repeat

| Row | Lines LOW | O1 Pattern |
|---|---|---|
| ROW_A | 3 + 4 | 0x3C |
| ROW_B | 4 + 5 | 0x39 |
| ROW_C | 4 + 6 | 0x35 |
| ROW_D | 4 + 7 | 0x2D |
| ROW_E | 8 only | 0x1F |
| IDLE | none | 0x3F |

### 4.3 GPIO Mapping (ESP32-S3 N16R8)

| Signal | ESP32 GPIO | Connection Target |
|---|---:|---|
| **PIN_ROW_A** | GPIO 4 | O1 Line 3+4 (ROW_A) |
| **PIN_ROW_B** | GPIO 5 | O1 Line 4+5 (ROW_B) |
| **PIN_ROW_C** | GPIO 6 | O1 Line 4+6 (ROW_C) |
| **PIN_ROW_D** | GPIO 7 | O1 Line 4+7 (ROW_D) |
| **PIN_ROW_E** | GPIO 15 | O1 Line 8 (ROW_E) |
| **O2_BUS (0-7)** | 41, 42, 8, 9, 10, 11, 12, 13 | O2 Lines 9-16 |
| **MUTE_PIN** | GPIO 21 | Controls 74HC4066 gates |
| **TXS_OE (×2)** | 3.3V or dedicated GPIO | TXS0108E Output Enable — must be HIGH at all times |
| **CSAFE_RX/TX** | GPIO 16 / 17 | MAX3232 Interface |
| **SPEED_IN** | GPIO 3 | Pin 7 (via PC817) |
| **INCLINE_IN** | GPIO 14 | Pin 11 (via PC817) |

> **Note:** `GPIO.out_w1ts` / `GPIO.out_w1tc` control GPIO 0–31 only. GPIO 41 and 42 (O2 bits 0–1) must be written via `GPIO.out1_w1ts.val` / `GPIO.out1_w1tc.val`. This split must be handled in `writeO2Fast()`.

> **TXS0108E OE pin:** The TXS0108E is passive (all channels high-impedance) until OE is driven HIGH. If OE is left floating or pulled LOW, O1 and O2 communication will silently fail with no error indication. Tie OE directly to 3.3V, or drive via a GPIO set HIGH in `setup()` before any interrupt is attached.

### 4.4 Frame Injection Protocol

Each button press is uniquely identified by the combination of its KEY byte, the specific frame sequence (O2), and the active O1 row pattern.

- **Sync Pulse:** 0xFF (all O2 lines HIGH) terminates every frame at ~330 Hz.
- **Injection Window:** Injection occurs strictly within the ~3 ms window triggered by a hardware interrupt detecting the correct O1 row FALLING edge.
- **Mute Duration:** The 74HC4066 gate is opened (MUTE_PIN HIGH) for the duration of one frame only (~3 ms), then immediately released.

### 4.5 Injection Timing Detail

Injection sequence must include:

- Gate open (74HC4066 enable)
- Stabilization delay (~1 ms)
- Frame transmission
- Settle delay (~2 ms)
- Gate release

This ensures the treadmill controller reliably registers injected signals.

### 4.6 Console Button Behavior (Verified)

Physical Speed (+/−) and Incline (+/−) buttons are **ignored** by the treadmill mainboard unless the belt is actively moving. It is not possible to pre-set a target speed or incline via the console before starting. This constraint also applies to injected commands — do not queue Speed/Incline injections before the RUNNING state is confirmed.

### 4.7 Mechanical Incline Behavior

- A QUICK START command always drives the incline physically to 0% as a homing sequence.
- Measured travel time under user load: 0% → 15% = 49 seconds, 15% → 0% = 48 seconds.
- This is used as the incline homing baseline in software.

---

## 5. Sensor Interpretation

### 5.1 Data Authority Rules

- **Speed** is derived exclusively from Pin 7
- **Incline** is derived exclusively from Pin 11 + homing model
- **System state** is derived exclusively from CSAFE

No subsystem (UI, FTMS, command queue) may override or infer these values independently.

All output systems must consume the same internal state variables.

---

## 6. Speed Sensor (Pin 7)

To read the speed flawlessly from the treadmill, we use a combination of optical hardware isolation and a non-blocking mathematical software filter to reject signal bounce.

### 6.1 Hardware Configuration — PC817 Output Stage

The PC817 output is a transistor switch — it cannot produce voltage or current on its own. It can only connect or disconnect two wires. This determines the entire pull-up architecture.

#### Wiring (Collector Side)

```text
3.3V ──── 10kΩ ──┬──── GPIO 3 (SPEED_IN)
                 │
             PC817 Pin 4 (Collector)
             PC817 Pin 3 (Emitter) ──── GND
```

#### Why the Pull-up Resistor is Mandatory

Without it, GPIO 3 is connected to nothing when the PC817 is off. A floating pin acts as an antenna — it picks up WiFi signals, static electricity, and AC motor EMI, producing false readings.

The 10 kΩ pull-up resistor solves this by holding the input HIGH when the optocoupler transistor is open.

#### The Slow-Rise Side Effect & Bouncing

When the PC817 switches off, the 10 kΩ resistor must pull the voltage back up to 3.3V. This rise is not instant — during the transition, the signal may cross the ESP32's logic threshold multiple times. These threshold crossings can produce additional digital edges (bounce), especially at low speeds.

```text
Voltage
 3.3V ─────────────╮          ╭─ bouncing ─╮          ╭──────────
                   │          │            │          │
                   ╰──────────╯            ╰──────────╯
                   ↑                       ↑
               Magnet in               Magnet out
               (clean LOW)         (slow rise + bounce)
```

#### Verified Circuit Parameters

| Sensor | Source Voltage | Series Resistor (LED Side) | Forward Current | Pull-up (Collector Side) |
|--------|--------------|-----------------------------|----------------|--------------------------|
| Speed (Pin 7) | 11.4V | 10 kΩ | ~1.1 mA | 10 kΩ to 3.3V |
| Incline (Pin 11) | 4.68V | 1 kΩ | ~3.8 mA | 10 kΩ to 3.3V |

> **Note:** Earlier prototypes attempted lower series resistors on the LED side. This can overload the harness line and cause treadmill errors. The 10 kΩ input resistor is the validated low-load configuration for Speed.

### 6.2 Software Filtering (Mathematical Glitch Rejection)

When the ESP32 detects the signal going LOW, a real magnet will keep the signal down for a duration in the millisecond range (observed ~2–5 ms at 10–25 km/h in verified captures).

Electrical bouncing and motor noise occur on the microsecond scale and may produce multiple rapid transitions around the threshold during both entry and exit of the magnet. At extremely low speeds (e.g., 1 km/h), exit bounce and threshold oscillation can occur milliseconds after the initial trigger.

Because the ESP32 must also monitor the fast-paced O1/O2 communication bus, we cannot use blocking delays inside the ISR.

#### Two-Stage Non-Blocking Filter

- **Micro-glitch rejection (~300 µs):** Extremely short intervals are discarded immediately.
- **Double-edge lockout (≈2 ms):** Prevents additional edges after a valid pulse from being counted as a new physical rotation.

The exact value (typically 1–3 ms) is chosen to:

- be significantly larger than observed microsecond-scale bounce
- but much smaller than the minimum physical pulse interval (~45 ms at maximum speed)

#### Verified Code (Non-Blocking Speed ISR)

```cpp
volatile unsigned long isr_lastPulseUs = 0;
volatile unsigned long isr_intervalUs  = 0;
volatile bool isr_newPulse = false;

// Stage 1: Reject threshold bounce and motor noise
const unsigned long GLITCH_REJECT_US = 300;

// Stage 2: Prevent threshold oscillation and slow exit-bounce
const unsigned long LOCKOUT_US = 2000;

void IRAM_ATTR isrSpeed() {
    unsigned long now = micros();
    unsigned long delta = now - isr_lastPulseUs;

    // Reject micro-glitches and most exit-bounce edges.
    // Remaining robustness relies on appropriate LOCKOUT_US tuning.
    if (delta < GLITCH_REJECT_US) return;
    if (delta < LOCKOUT_US) return;

    // Valid physical rotation detected
    isr_intervalUs  = delta;
    isr_lastPulseUs = now;
    isr_newPulse    = true;
}
```

### 6.3 Expected Speed Signal Characteristics (Post-Filtering)

- One valid falling edge per physical rotation
- No additional edges within <2 ms after a valid pulse
- Stable frequency corresponding to:
  - ~2 Hz at 1 km/h
  - ~9 Hz at 10 km/h
  - ~22 Hz at 25 km/h

### 6.4 Speed Continuity & Fault Handling

#### Speed Continuity Rule

- Last valid speed value may be maintained briefly if pulses are temporarily missing
- If no valid pulse is received within a timeout window → speed is set to 0

#### Speed Signal Fault Detection (Ghost Running)

If:

- CSAFE reports RUNNING
- and no valid speed pulses are detected for >2 seconds

Then:

- UI resets speed to 0
- a fault condition is logged

---

## 7. Incline Sensor (Pin 11)

### 7.1 Interpretation

Pin 11 emits a constant pulse train (~394 Hz) while the incline motor is physically moving, and returns to rest voltage when motion stops.

Incline is not treated as a direct analog angle value. Actual incline is calculated in software by homing to 0% and then counting movement pulses.

### 7.2 Hardware Interface Options (Compatibility Note)

**V3.1 chosen interface:** PC817 optocoupler for isolation, matching the speed design philosophy.

```text
Pin 11 (4.68V) ──── 1kΩ ──── PC817-2 Anode
                             PC817-2 Katode ──── GND

3.3V ──── 10kΩ ──┬──── GPIO 14 (INCLINE_IN)
                 │
             PC817-2 Collector
             PC817-2 Emitter ──── GND
```

**Legacy/previously used interface (still relevant for troubleshooting):** BSS138-based level shifting has been used successfully in some prototypes. If incline is already stable and verified in a given build, it may remain unchanged until a full system regression test is complete.

### 7.3 Homing Rule

A QUICK START command always drives the incline physically to 0% as a homing sequence. The system waits for the 394 Hz activity to stop, then locks its internal tracker to `0.0%`.

### 7.4 Persistence

Incline pulse count is committed to persistent storage when movement stops (or when system transitions to STOPPED), ensuring position recovery across reboots. Runtime-only counters are never written on every pulse.

---

## 8. Calibration & Software Constants

### 8.1 Speed Calibration

Verified reference points:

- 10 km/h = 8.97 Hz
- 25 km/h = 22.3 Hz

Working conversion:

- `Speed (km/h) = Frequency (Hz) * 1.1148` (anchored to 10 km/h)

Derived odometer constant:

- `1 meter = 3.2292 pulses`

### 8.2 Speed Calibration and AutoCal

The system supports a dynamic calibration model:

- Base conversion uses a fixed factor (`km/h = Hz * constant`)
- A correction table refines accuracy per speed range
- Linear interpolation is used between known calibration points

Learning rules:

- Calibration updates occur only when speed is stable
- Values are updated in RAM during operation
- Persistent writes occur only when treadmill enters STOPPED state

This minimizes flash wear and prevents incorrect learning during transient states.

### 8.3 Incline Calibration

- Pin 11 emits ~394 Hz while incline is moving
- QUICK START homes incline to 0%
- Full travel is ~49 s upward and ~48 s downward under load

A final fixed pulses-per-percent constant is intentionally not declared until a full measured 0% → 15% sweep pulse count has been captured and validated.

---

## 9. Hardware Interface Choices

### 9.1 MCU & Profile

- Controller: ESP32-S3 (dual-core)
- Required profile: N16R8 (for long-session stability with WiFi + BLE + web assets)

### 9.2 Logic Level Translation

- Two TXS0108E 8-channel bi-directional level shifters bridge 5V O1/O2 buses to 3.3V GPIO
- OE must be held HIGH at all times

### 9.3 Data Gate (Mute Circuit)

- Hardware: SN74HC4066N inline on O2 data bus
- Default state must preserve native console communication
- Fail-safe pull-down on MUTE_PIN ensures pass-through on MCU reset/power loss

### 9.4 Power Supply

- HLK-PM01 AC-to-5V module for permanent internal power
- Mains-side fusing and isolation are required

### 9.5 CSAFE Physical Interface

- RJ45 to MAX3232 (9600 baud, 8-N-1)

---

## 10. Software Architecture Rules

### 10.1 Dual-Core Task Split

- **Core 0:** WiFi, Web server, WebSockets, LittleFS, Bluetooth FTMS, system logging
- **Core 1:** Sensor ISRs (Pin 7, Pin 11), injection timing tasks, safety watchdog tasks

Thread safety:

- Shared state between cores must be protected by a global `portMUX_TYPE` spinlock.

### 10.2 Telemetry Rate Control & Backpressure

Real-time pulse capture, WebSocket rendering, CSAFE parsing, and FTMS broadcasting must not compete directly for timing.

Rules:

- Hardware capture is interrupt-driven
- No UI/network work inside ISR
- Downstream consumers read from internal state at fixed rates
- If consumers cannot keep up, newest state replaces pending state (stale frames may be dropped)
- Pulse counters and homing-critical state must never be dropped

### 10.3 Filesystem, Persistence & OTA

- Web assets and configuration are stored in LittleFS
- Persistent writes are batched and only committed on STOPPED transitions where possible
- Firmware OTA and filesystem OTA are treated as separate artifacts
- Configuration changes should be written atomically (tmp-write + rename)

---

## 11. CSAFE Rules (RS232)

- Connection: RJ45 to MAX3232 (9600 baud, 8-N-1)
- Polling is split into two separate frames (merging may destabilize the treadmill serial controller):

  - Keep-alive: `0xF1 0x85 0x85 0xF2`
  - Status request: `0xF1 0x80 0x80 0xF2`

- Polling interval: 200–250 ms

Forbidden commands (do not send):

- `0xAA`, `0xA5`, `0x9C` (known to cause buffer corruption on DK-City mainboard)

Frame handling:

- Reset buffer on `0xF1`
- Parse only when `0xF2` is received
- State byte extracted and mapped to internal enums

Application code must never read UART directly. CSAFE must be handled as a dedicated transport/parser layer.

---

## 12. Timing & Safety Rules

- **Incline Timeout:** Maximum continuous incline motion is capped at 60 seconds.
- **3.5s Startup Sync:** Ignore speed/distance accumulation for 3.5 seconds after STARTING to align with console countdown.
- **700 ms Resume Debounce:** Debounce when transitioning from PAUSED to RUNNING.
- **1500 ms CSAFE Watchdog:** If no valid CSAFE end frame is received for 1500 ms, force internal state to STOPPED.
- **Ghost-Running Detection:** If CSAFE reports RUNNING but Pin 7 reads 0 Hz for >2 seconds, log mismatch fault.
- **Command Queue Expiry:** Commands queued during STARTING expire after 5 seconds if RUNNING is not reached.

---

## 13. Bluetooth FTMS Rules

- BLE stack: NimBLE-Arduino
- Broadcast rate: 1 Hz
- Speed format: integer with 0.01 resolution (e.g., 12.50 km/h → 1250)
- Incline format: integer with 0.1 resolution (e.g., 3.5% → 35)
- Loss of BLE client must not stop telemetry capture or corrupt treadmill state tracking

---

## 14. GUI & Frontend Rules (High-Level)

- Vanilla JavaScript only (no frameworks)
- Stateless UI rendering from WebSocket telemetry
- WebSocket update rate bounded (5–10 Hz)
- Use `pointerdown` events for low-latency interaction
- Avoid visual flicker (tabular numbers; sensible distance resolution)

---

## 15. Open Items

- Final incline span calibration (0% → 15% pulse count)
- Verification of any remaining bus mapping details
- Validation of injection timing margins on final hardware (stabilization and settle delays)
