# StrideControl Technical Design Guide (V4)

This document contains verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system (V4 Architecture) safely and predictably.

> **Primary UI Goal**
>
> StrideControl provides a clean, tablet-based web interface for direct speed and incline control. Because the tablet completely covers the original console, the UI must display **measured belt speed** and the **hardware-tracked incline estimate** derived from physical hardware feedback, not just requested target values. When IMU verification is available and calibrated, the UI may additionally identify the incline estimate as IMU-verified.

---

## 1. Project Goal

The objective is to build a safe, reversible ESP32-S3 interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:

- Hosts a local web interface for a tablet.
- Allows direct user control of speed and incline.
- Measures physical belt speed from the treadmill speed-feedback signal and tracks incline position from homing and calibrated movement-pulse integration.
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

ISRs must be minimal and non-blocking where possible, limited to timestamping and minimal validation. No logging, UI updates, networking, or serial I/O is allowed inside ISRs.

- **Exception Rule:** Short blocking delays are strictly prohibited *unless* required for critical signal validation and EMI rejection, and must be explicitly documented.
- *Current Exception:* The Incline ISR (Pin 11) uses a brief `delayMicroseconds(10)` to filter heavy motor noise on the falling edge.

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


## 4.2 Verified O1/O2 Bus Model

The treadmill console uses two related parallel logic buses:

- **O1** is the scanner and phase-reference bus.
- **O2** is the response bus used by the physical control panels to report button states to the treadmill controller.

The ESP32 does not generate or modify O1. O1 remains a physical pass-through signal and is read-only from the ESP32.

During command injection, the ESP32 isolates the physical O2 sources and temporarily generates the complete O2 response expected by the treadmill controller.

### 4.2.1 O1 GPIO Mapping

O1 is read as a 5-bit value:

| O1 bit | ESP32 GPIO |
|---:|---:|
| Bit 0 | GPIO 4 |
| Bit 1 | GPIO 5 |
| Bit 2 | GPIO 6 |
| Bit 3 | GPIO 7 |
| Bit 4 | GPIO 15 |

### 4.2.2 Verified O1 Phases

The following O1 values are the authoritative phase model:

| Phase | O1 value | Normal O2 baseline |
|---|---:|---:|
| Phase A | `0x0F` | `0x80` |
| Phase B | `0x17` | `0x80` |
| Phase C | `0x1B` | `0x80` |
| Phase D | `0x1D` | `0xC0` |
| Idle | `0x1F` | Time-dependent |

The previous ROW_A–ROW_E model and O1 patterns such as `0x3C`, `0x39`, `0x35`, `0x2D`, and `0x3F` are superseded and must not be used by the active emulator.

### 4.2.3 Idle Response

Idle does not use one fixed O2 value.

When O1 enters `0x1F`, the emulator must generate:

- O2 `0x00` during the first 1200 µs
- O2 `0xFF` after 1200 µs

The phase timer is restarted whenever O1 changes into a new phase. Idle therefore cannot be implemented correctly as one static lookup-table value.

---

## 4.3 GPIO Mapping, O2 Bus, and Physical Isolation

### 4.3.1 ESP32-S3 GPIO Mapping

| Signal | ESP32 GPIO | Function |
|---|---:|---|
| O1 bit 0 | GPIO 4 | Read-only phase input |
| O1 bit 1 | GPIO 5 | Read-only phase input |
| O1 bit 2 | GPIO 6 | Read-only phase input |
| O1 bit 3 | GPIO 7 | Read-only phase input |
| O1 bit 4 | GPIO 15 | Read-only phase input |
| O2 bit 0 | GPIO 41 | Bidirectional O2 data |
| O2 bit 1 | GPIO 42 | Bidirectional O2 data |
| O2 bit 2 | GPIO 8 | Bidirectional O2 data |
| O2 bit 3 | GPIO 9 | Bidirectional O2 data |
| O2 bit 4 | GPIO 10 | Bidirectional O2 data |
| O2 bit 5 | GPIO 11 | Bidirectional O2 data |
| O2 bit 6 | GPIO 12 | Bidirectional O2 data |
| O2 bit 7 | GPIO 13 | Bidirectional O2 data |
| Shared MUTE | GPIO 21 | Active-low isolation of physical O2 sources |
| TXS0108E OE | GPIO 2 | Output enable for both TXS0108E devices |
| CSAFE RX/TX | GPIO 16 / 17 | MAX3232 interface |
| Speed input | GPIO 3 | Isolated speed-pulse input |
| Incline input | GPIO 14 | Isolated incline-pulse input |
| I2C SDA | GPIO 47 | LSM6DSOX data |
| I2C SCL | GPIO 48 | LSM6DSOX clock |

### 4.3.2 O2 Bus

O2 is an 8-bit parallel response bus.

GPIO 41 and GPIO 42 are located in the upper ESP32 GPIO register bank. Atomic O2 output updates must therefore use both register groups:

- GPIO 0–31 through `GPIO.out_w1ts` and `GPIO.out_w1tc`
- GPIO 32 and above through `GPIO.out1_w1ts` and `GPIO.out1_w1tc`

O2 must be updated as one logical byte. Sequential `digitalWrite()` calls are not suitable for phase-critical output updates.

### 4.3.3 Atomic Direction Change

The required O2 output value must be preloaded into the GPIO output latches before the O2 pins are changed from `INPUT` to `OUTPUT`.

For the final implementation, the direction change should also be performed as one logical bus operation through the ESP32 GPIO-enable registers rather than through eight sequential `pinMode(..., OUTPUT)` calls.

The implementation must account for both GPIO register banks:

- GPIO 0–31 through the corresponding `GPIO.enable_w1ts` and `GPIO.enable_w1tc` registers
- GPIO 32 and above through the corresponding upper-bank GPIO-enable registers

This prevents a short transition window where only part of the O2 byte is actively driven.

**Current implementation:** The required O2 value is preloaded into the output latches before the O2 pins are changed sequentially with `pinMode(..., OUTPUT)`.

**Target implementation:** Change all O2 direction bits atomically through the GPIO-enable registers after the register behavior has been validated on the ESP32-S3.

Until atomic direction switching has been physically validated, sequential pin-direction changes remain an implementation limitation and must be treated as a timing-margin item.

### 4.3.4 Physical Panel Isolation

The physical touch console and the separate side panel are independent signal sources, but both are isolated using one shared MUTE control.

The installed analog switches are `SN74HC4066N` devices.

The shared MUTE node is controlled by:

- ESP32 GPIO 21
- TXS0108E level translation from 3.3 V to 5 V
- A 10 kΩ pull-up on the 5 V side

MUTE behavior:

| MUTE state | Physical panel state |
|---|---|
| HIGH | SN74HC4066N channels closed; physical panels connected |
| LOW | SN74HC4066N channels open; physical panel outputs isolated |

MUTE is strictly active-low.

The shared MUTE signal isolates the relevant output paths from both:

- the touch-console controller
- the separate side-panel switches

The emergency-stop circuit is not part of the switched or injected signal path.

### 4.3.5 TXS0108E OE and Safe Startup

The TXS0108E devices are passive while OE is LOW.

OE must start LOW to avoid undefined translation during power sequencing. This is enforced by a 10 kΩ hardware pull-down to GND.

The required startup sequence is:

1. Configure TXS OE as `OUTPUT` and drive it LOW.
2. Configure MUTE as `OUTPUT` and drive it HIGH.
3. Configure all O1 and O2 GPIOs as `INPUT`.
4. Drive TXS OE HIGH after the GPIO states are safe.

### 4.3.6 Passive State

When command injection is inactive:

- O1 GPIOs remain configured as `INPUT`.
- O2 GPIOs remain configured as `INPUT`.
- MUTE remains HIGH.
- TXS0108E OE remains HIGH after safe startup.
- Physical console and side-panel operation pass through normally.

---

## 4.4 Verified O2 Emulation Principle

The treadmill is not controlled by replaying a captured multi-byte frame.

The verified control method is continuous, phase-dependent O2 emulation.

During injection, the ESP32 generates the complete expected O2 baseline for every observed O1 phase. A button is represented by replacing the normal O2 value in one specific phase while maintaining the normal response in all other phases.

### 4.4.1 Baseline Generation

The baseline response is:

- O1 `0x0F` → O2 `0x80`
- O1 `0x17` → O2 `0x80`
- O1 `0x1B` → O2 `0x80`
- O1 `0x1D` → O2 `0xC0`
- O1 `0x1F` → O2 `0x00`, followed by `0xFF` after 1200 µs

The baseline must continue:

- before a button becomes active
- during pauses between macro steps
- after a button is released
- until the physical panels are reconnected

The ESP32 must never inject only the changed bit while leaving the remaining O2 lines undefined. The complete O2 byte must always be driven.

### 4.4.2 Command Injection Sequence

The verified control sequence is:

1. Keep the physical panels connected while observing O1.
2. Wait for a valid O1 phase transition.
3. Determine the active O1 phase.
4. Calculate the correct O2 baseline for that phase.
5. Preload the ESP32 O2 output latches with the baseline.
6. Assert the shared MUTE signal LOW.
7. Change all O2 GPIOs from `INPUT` to `OUTPUT` as one logical bus operation where supported.
8. Continuously generate the required O2 response for every O1 phase.
9. Apply the selected button response for the configured hold period.
10. Return to complete baseline emulation between button steps.
11. Keep MUTE active for the entire command or macro.
12. After the final step, continue baseline emulation until O1 enters IDLE.
13. Wait until the emulator is safely inside the IDLE phase.
14. Return all O2 GPIOs to `INPUT`.
15. Release MUTE HIGH and reconnect the physical panels.
16. Allow a settling period before passive physical-button monitoring resumes.

The physical panels must not be disconnected while the ESP32 O2 bus is undefined.

---

## 4.5 Verified Button Response Matrix

The authoritative button mapping is the combination of:

- the active O1 phase
- the complete O2 value generated during that phase

All phases not listed for a selected button retain their normal baseline values.

| Function | Active O1 phase | Active O2 value |
|---|---:|---:|
| Instant Incline | `0x0F` | `0xC0` |
| Number 1 | `0x0F` | `0x82` |
| Number 4 | `0x0F` | `0x84` |
| Number 5 | `0x0F` | `0x90` |
| Number 7 | `0x0F` | `0x88` |
| Number 8 | `0x0F` | `0xA0` |
| Instant Speed | `0x17` | `0xC0` |
| Enter | `0x17` | `0x88` |
| Number 0 | `0x17` | `0xA0` |
| Number 2 | `0x17` | `0x90` |
| Number 3 | `0x17` | `0x84` |
| Number 6 | `0x17` | `0x81` |
| Number 9 preload | `0x17` | `0x81` |
| Number 9 active phase | `0x1B` | `0x81` |
| Fan On/Off | `0x1B` | `0xA0` |
| Fan High | `0x1B` | `0x90` |
| Fan Low | `0x1B` | `0xC0` |
| Speed + | `0x1D` | `0xC4` |
| Speed − | `0x1D` | `0xC8` |
| Incline + | `0x1D` | `0xC1` |
| Incline − | `0x1D` | `0xE0` |
| Physical Quick Start | `0x1D` | `0xD0` |
| Physical Stop | `0x1D` | `0xC2` |

### 4.5.1 Number 9 Exception

Number 9 is a confirmed timing exception.

A response that changes O2 to `0x81` only after O1 has entered Phase C is too late for reliable detection.

The required behavior is:

- While Number 9 is active and O1 is Phase B, output `0x81`.
- Continue outputting `0x81` when O1 transitions into Phase C.
- Return to the normal baseline outside the required preload and active phases.

This early-preload rule is specific to Number 9 and must not be generalized to other buttons without physical verification.

### 4.5.2 Quick Start and Stop

The confirmed physical values are:

- Quick Start: O1 `0x1D`, O2 `0xD0`
- Stop: O1 `0x1D`, O2 `0xC2`

These functions are retained for passive recognition.

The current control architecture does not expose Quick Start or Stop as web-injected commands. They are treated as physical, read-only events.

---

## 4.6 Timing, Watchdog, and Recovery Rules

### 4.6.1 Initial Synchronization

Before asserting MUTE, the emulator must observe O1 while the physical panel remains connected.

A command may begin only after:

- O1 changes from the previously observed value
- the new O1 value is one of the valid phases:
  - `0x0F`
  - `0x17`
  - `0x1B`
  - `0x1D`
  - `0x1F`

If no valid phase transition is observed within the configured synchronization timeout, injection must not begin.

### 4.6.2 Current Timing Values

The following timing values are the verified operational configuration:

- Button hold time: 500 ms
- Baseline pause between steps: 100 ms
- Initial O1 synchronization timeout: 20 ms
- O1 phase watchdog: 25 ms
- Final IDLE search timeout: 25 ms
- Minimum IDLE age before reconnect: 200 µs
- O2/MUTE transition stabilization: 50 µs
- Passive monitor restart delay after reconnect: 10 ms

The 500 ms button hold and 100 ms pause are the verified conservative operational values. Any later reduction must be regression-tested across all mapped commands before adoption.

### 4.6.3 Phase Watchdog

While the ESP32 controls O2, valid O1 phase transitions must continue.

If no valid O1 phase transition is observed for more than 25 ms:

1. Abort the active command.
2. Return every O2 GPIO to `INPUT`.
3. Wait 50 µs to ensure that the ESP32 has released the O2 bus.
4. Release MUTE HIGH.
5. Wait 10 ms for bus and panel settling.
6. Re-enable passive physical-button monitoring.
7. Clear the active-hardware state.
8. Report the command as failed.

An unknown O1 value must not count as a valid watchdog-resetting phase transition.

The same ordered recovery sequence and timing margins apply to every abnormal exit path, including:

- initial synchronization failure
- phase watchdog timeout
- failure to locate IDLE before reconnect
- future hardware-state-machine faults

### 4.6.4 Controlled Reconnect

After the last command step:

1. Return to normal O2 baseline generation.
2. Wait for O1 to enter IDLE.
3. Continue generating the correct time-dependent IDLE response.
4. Wait until IDLE has been active for at least 200 µs.
5. Set O2 to `INPUT`.
6. Wait 50 µs to ensure that the ESP32 has released the bus.
7. Release MUTE HIGH.
8. Wait 10 ms before restarting passive button monitoring.

If IDLE is not observed within the timeout, the command is aborted through the same controlled recovery path.

---

## 4.7 Command and Macro Execution

The browser sends user intent rather than a timed sequence of individual button presses.

Examples:

- Set speed to 15.7 km/h
- Set incline to 5.5%
- Press Speed+
- Press Incline−

The ESP32 validates and plans the complete sequence.

### 4.7.1 Fractional Target Algorithm

For fractional targets:

1. Round the requested value to the nearest valid integer.
2. Clamp the integer to the permitted range.
3. Execute the corresponding Instant Speed or Instant Incline sequence.
4. Enter the integer using the number buttons.
5. Send Enter.
6. Apply the required number of `+` or `−` adjustments.

Examples:

- 12.6 → base 13, followed by four Speed− steps
- 12.4 → base 12, followed by four Speed+ steps

Valid command ranges:

- Speed: 0.8–25.0 km/h
- Incline: 0.0–15.0%

### 4.7.2 Macro Isolation Rule

A complete macro is executed during one continuous O2-ownership period.

The physical panels are not reconnected between:

- Instant Speed or Instant Incline
- number entry
- Enter
- fractional `+` or `−` adjustments

Normal O2 baseline is generated during the pause between all individual macro steps.

### 4.7.3 Command Queue and Status

Command requests are transferred to the hardware task through a bounded FreeRTOS queue.

Supported command states are:

- `queued`
- `started`
- `step`
- `completed`
- `rejected`
- `failed`

A new request must never overwrite an active command. Commands are either queued, rejected, or executed sequentially.
### 4.8 Console Button Behavior (Verified)

Physical Speed (+/−) and Incline (+/−) buttons are **ignored** by the treadmill mainboard unless the belt is actively moving. It is not possible to pre-set a target speed or incline via the console before starting. This constraint also applies to injected commands — do not queue Speed/Incline injections before the RUNNING state is confirmed.

### 4.9 Mechanical Incline Behavior

- A QUICK START command always drives the incline physically to 0% as a homing sequence.
- Measured travel time under user load: 0% → 15% = 49 seconds, 15% → 0% = 48 seconds.
- This is used as the incline homing baseline in software.

- **Board 2 (Brain Board / Level Shifting):**
  - Takes the 16-pin TAP cable from Board 1
  - Contains 2x TXS0108E bidirectional level shifters
  - Translates 5V signals to 3.3V safe logic for the ESP32-S3

  - **MUTE signal:**  
    Level-shifted via TXS0108E from ESP32 (3.3V) to 5V to override R2 pull-up

  - **TXS Output Enable (OE):**  
    Held LOW by hardware **10 kΩ pull-down (R1)** during boot  
    Enabled by ESP32 GPIO 2 after power stabilization

  - Outputs directly to ESP32 GPIOs

---

## 5. Sensor Interpretation

### 5.1 Data Authority Rules

- **Speed** is derived exclusively from Pin 7
- **Incline** is maintained as a hardware-tracked estimate derived from Pin 11, a verified homing reference, calibrated movement-pulse integration, and direction-dependent conversion factors
- **System state** is derived exclusively from CSAFE

No subsystem (UI, FTMS, command queue) may independently overwrite these authoritative state variables. The IMU may provide an independent, filtered deck-angle measurement for verification and drift detection, but it does not replace the pulse-integrated incline tracker during movement.

All output systems must consume the same internal state variables.

### 5.2 Cadence (LSM6DSOX)

An LSM6DSOX IMU is rigidly mounted to the moving treadmill deck or another structure that follows the deck angle, via I2C (SDA: GPIO 47, SCL: GPIO 48). It is used to derive cadence from footstrike shockwaves and to provide a filtered physical deck-angle measurement for incline verification. The built-in hardware step counter may be used only after it has been validated against raw accelerometer-based detection on the installed treadmill. The I2C bus is actively terminated via an LTC4311 extender to handle the 2-meter cable run from the frame to the motor compartment.

* **Cabling:** Shielded repurposed USB cable to protect against 3.5 HP motor EMI.
* **Connectors:** Wago 221 vibration-proof connectors at the frame splice; GX12 Aviation plug at the ESP32 enclosure for modularity.
* **Graceful Degradation:** If the sensor disconnects or the I2C bus faults, cadence defaults to 0 SPM. This must not trigger a crash or interfere with Core 1 motor pulse processing.
* **Polling:** Must be polled asynchronously to prevent blocking Core 1 interrupts.
* **Constraint:** The cadence subsystem must run entirely on Core 0 and must never interfere with ISR timing or injection logic.

### 5.3 Runner Presence Detection (Cadence Validation)

To prevent false activity logging in external systems (e.g. Zwift, sports watches), the system must detect whether a runner is physically present on the treadmill.

This is derived from cadence (LSM6DSOX) in combination with measured speed.

#### Operational Rule

If:

- **Measured speed > 0 km/h** (Pin 7 indicates belt is moving)
- AND **Cadence = 0 SPM** (no footstrike detected for a defined timeout window)

Then:

- The system must classify this as **"No Runner Present"** (ghost running condition)

#### Effects

When "No Runner Present" is active:

- **FTMS speed output must be forced to 0**
- **Distance accumulation must be paused or suppressed**
- **Cadence remains 0**
- Internal treadmill state remains unchanged (no interference with motor control)

#### Timeout Definition

- A cadence timeout window of approximately **2–3 seconds** is recommended to prevent false triggering during step transitions.

#### Safety Constraint

- This logic **must not interfere with Core 1 real-time processing**
- Detection and filtering must be handled on Core 0

#### Design Intent

This feature ensures that external systems:

- do not log running activity when the user steps off the treadmill
- maintain accurate training data integrity
  
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
| --- | --- | --- | --- | --- |
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

    // Note: The first check (GLITCH_REJECT_US) is technically caught by the 
    // second check (LOCKOUT_US), but both are explicitly kept for readability 
    // and future individual tuning of bounce vs. oscillation metrics.
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

Incline is not treated as a direct analog angle value. The primary incline value is a hardware-tracked estimate calculated by homing to 0% and integrating physical movement pulses with separate calibrated factors for upward and downward travel. A filtered IMU deck-angle measurement may be used to verify the estimate after movement has stopped.

### 7.2 Hardware Interface Options (Compatibility Note)

**V3.1 chosen interface:** PC817 optocoupler for isolation, matching the speed design philosophy.

```text
Pin 11 (4.68V) ──── 1kΩ ──── PC817-2 Anode
                             PC817-2 Cathode ──── GND

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

> ⚠️ **FUNCTIONAL REQUIREMENT: NVS Trust Model & Operational Drift Handling**
> **Consequences if ignored:** If there is no manual re-homing mechanism, mechanical drift or untracked manual movement while the ESP32 is unpowered can permanently desynchronize the stored NVS value. This can drive the incline motor beyond intended limits and damage the mechanism.
> **Operational Rule:** The system operates under a trust model where the NVS-stored pulse count is assumed correct on boot without physical verification. To mitigate inevitable drift over time (e.g., after power loss or manual movement), a dedicated **Re-home Incline** action must be exposed in the web UI / AP configuration. This action forces a full homing sequence down to 0% to re-synchronize the NVS counter.

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

### 8.3 Incline Calibration (Verified)

- Pin 11 emits ~394 Hz while incline is moving.
- A QUICK START command homes the incline to 0%.
- Physical measurements confirm an asymmetry in pulse counts due to gravity assist on descent.

*Verified calibration constants (from physical sweep):*

- `PULSES_PER_PERCENT_UP = 3086.0f`
- `PULSES_PER_PERCENT_DOWN = 2943.0f`

---

## 9. Hardware Interface Choices

### 9.1 MCU & Profile

- Controller: ESP32-S3 (dual-core)
- Required profile: N16R8 (for long-session stability with WiFi + BLE + web assets)

### 9.2 Logic Level Translation

- Two TXS0108E 8-channel bi-directional level shifters
- Bridge 5V O1/O2 buses to 3.3V GPIO

- **OE Control Rule:**
  - OE must **start LOW via 10 kΩ pull-down (R1)**
  - Prevents latch-up during power sequencing
  - ESP32 must actively drive OE HIGH after boot

## 9.3 Data Gate and Shared MUTE Circuit

The O2 data-gate circuit uses `SN74HC4066N` bilateral switches.

The default hardware state must preserve native console and side-panel communication.

### 9.3.1 Fail-Safe State

The shared MUTE node has a 10 kΩ pull-up to 5 V.

This creates the following fail-safe behavior:

| Condition | Result |
|---|---|
| ESP32 unpowered | MUTE pulled HIGH; physical panels connected |
| ESP32 booting with TXS OE LOW | Physical panels remain connected through the hardware pull-up |
| Normal passive operation | MUTE HIGH; physical panels connected |
| Active command injection | MUTE LOW; physical O2 sources isolated |

### 9.3.2 Active-Low MUTE Logic

MUTE is strictly active-low:

- Drive MUTE LOW → open the SN74HC4066N channels and isolate the physical O2 sources
- Drive MUTE HIGH → close the SN74HC4066N channels and restore physical pass-through

MUTE is level-shifted through the TXS0108E from 3.3 V to 5 V. The ESP32 MUTE pin must therefore be configured as a push-pull `OUTPUT`, not open-drain.

### 9.3.3 Shared Isolation Scope

One shared MUTE output controls both:

- the touch-console O2 source
- the separate side-panel switch source

The emergency-stop circuit is excluded from the switched and injected path.

### 9.3.4 Ownership and Release Sequence

MUTE remains LOW during the complete command or macro. The physical panels are not reconnected between individual macro steps.

Before MUTE is released HIGH:

1. The ESP32 must return to normal O2 baseline generation.
2. O1 must enter the defined reconnect point in IDLE.
3. All O2 GPIOs must be returned to `INPUT`.
4. The ESP32 must wait 50 µs to ensure that the O2 bus has been released.
5. MUTE may then be released HIGH.
6. Passive monitoring resumes after a 10 ms settling period.

The same release sequence applies during watchdog recovery and every abnormal exit path.

### 9.4 Power Supply

- HLK-PM01 AC-to-5V module for permanent internal power
- Mains-side fusing and isolation are required

>  **FUNCTIONAL REQUIREMENT: Power Budget & Thermal Margin**
> **Consequences if ignored:** When WiFi, BLE, and the WebSocket server operate simultaneously, current draw can spike. Insufficient margin can cause unpredictable brownout resets during operation.
> **Power Rule:** Continuous load must remain comfortably below the power supply's rated continuous capacity to preserve thermal margin. Short peaks are acceptable, but must not be sustained.
> **Design Target:** Continuous load should remain below ~70–80% of the PSU rated continuous capacity to maintain thermal margin and long-term stability.

>  **VALIDATION REQUIRED: Hardware Power Verification Protocol**
> Before final enclosure assembly, perform the following:
> 1. **Sustained Load Measurement:** Measure steady-state current draw under full wireless load (WiFi + BLE FTMS + WebSocket at 10 Hz).
> 2. **Peak Capture:** Capture voltage sags on 3.3V and 5V rails during radio activity using an oscilloscope.
> 3. **Thermal Stress Test:** Run the system continuously for at least 30 minutes at maximum traffic and verify no brownout events or unexpected watchdog resets.

### 9.5 CSAFE Physical Interface

- RJ45 to MAX3232 (9600 baud, 8-N-1)

---

## 10. Software Architecture Rules

### 10.1 Dual-Core Task Split

- **Core 0:** WiFi, Web server, WebSockets, LittleFS, Bluetooth FTMS, system logging
- **Core 1:** Sensor ISRs (Pin 7, Pin 11), injection timing tasks, safety watchdog tasks

Thread safety:

- Shared state between tasks and cores must use an explicit synchronization mechanism appropriate to the data.
- FreeRTOS queues shall be used for command requests, status events, and hardware events.
- Atomic types may be used for simple shared flags.
- A `portMUX_TYPE` critical section may be used for short access to shared multi-field state that cannot be transferred through a queue.
- Network and UI code must never access the O1/O2 hardware state directly.
- The Core 1 hardware task owns active MUTE and O2 control.

> **FUNCTIONAL REQUIREMENT: Watchdog Feeding Strategy & Standby Keepalive**
> **Consequences if ignored:** If the watchdog is only fed when O1 interrupts occur, the device can enter a restart loop during standby/IDLE/STOPPED when bus activity stops.
> **Operational Rule:** Core 1 must implement a watchdog keepalive strategy that runs independently of O1 interrupt activity to guarantee stable standby operation.
> **Core Responsibility:** Watchdog feeding must be owned by Core 1 only. Core 0 must never reset the watchdog, ensuring that failure in real-time tasks (Core 1) cannot be masked by network/UI activity.

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

* **Custom Partition Table:** A custom CSV partition scheme is strictly required. The default partitions will fail due to the heavy flash footprint of Dual-Role BLE and AsyncWebServer combined. Allocate a minimum of `1.8 MB` for the primary application partition (`app0`), leaving the remainder for the LittleFS data partition.
* **OTA Constraint:** If OTA updates are required, a dual-application layout (`app0` + `app1`) must be used. This reduces available application size and must be considered when defining partition sizes.

### 10.4 Bootstrap, Recovery & AP Mode

* **Network Fallback:** The device must be deployable and recoverable without serial/USB intervention. If pre-configured WiFi credentials fail or are missing, the system must gracefully fall back to an Access Point (AP) mode.
* **Trigger Condition:** AP mode must activate after repeated connection failures (e.g. >5 retries or ~30 seconds timeout during boot).
* **Configuration Portal:** In AP mode, a lightweight captive portal must be exposed to allow the user to input new WiFi credentials, configure device settings (including FTMS name), and execute the manual **Re-home Incline** calibration routine.

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
- **FTMS Elevation Gain:** The FTMS specification requires Positive Elevation Gain, which the hardware does not natively provide. Accumulate algorithmically per update:
  `elevation_gain += distance_interval_km * 1000 * (incline_pct / 100)`

* **Cadence Integration:** Cadence derived from the LSM6DSOX hardware step-counter must be included in the FTMS payload when available.

* **BLE Heart Rate Proxy (Dual-Role):** The NimBLE stack must operate in **Dual-Role** mode. The ESP32 acts as:
  - a **Client** to scan and connect to external BLE Heart Rate monitors
  - a **Server** to broadcast FTMS data

  The proxy injects received HR data into the FTMS payload. The selected device MAC must be persisted in NVS for automatic reconnection.

* **Constraint:** All BLE operations must remain on Core 0. BLE scanning, connections, and FTMS broadcasting must never interfere with Core 1 ISR timing or injection logic.

---

## 14. GUI Architecture & Interval Coach

### 14.1 Stateless UI & Quick Keys
* **Stack:** Vanilla JavaScript only. No frameworks. Stateless UI rendering from WebSocket telemetry.
* **Interaction:** `pointerdown` events bypass mobile browser tap delay.
* **Quick Keys (Grid):** 8-button symmetrical grids at the top (Incline) and bottom (Speed) for instant manual control.
* **Macro Presets (HVILE / DRAG):** Prominent action buttons populated dynamically from `profiles.json`.
* **Persistent Availability:** Manual controls must always be available, even during interval focus mode.

### 14.2 Pro Interval Coach (Visual Engine)
* **No Automated Control:** The interval engine is strictly visual and auditory. It never injects commands automatically.
* **Focus Mode:** UI transitions to `focus-mode`, minimizing distractions and centering countdown timers.
* **Audio Cues:** Beeps generated using Web Audio API. No external audio files required.
* **Psychological Overlays:**
  - ETA projection (finish time)
  - Phase completion banners
  - Post-workout RPE prompt

### 14.3 User Profile Schema (profiles.json)

```json
{
  "users": [
    {
      "name": "Kristian",
      "presets": {
        "hvile": 6.0,
        "drag": 16.0
      },
      "quick_keys": {
        "speed": [4, 6, 8, 10, 12, 14, 16, 18],
        "incline": [0, 1, 2, 4, 6, 8, 10, 12]
      },
      "last_interval": {
        "work_m": 0,
        "work_s": 45,
        "rest_m": 0,
        "rest_s": 15,
        "reps": 10,
        "series": 2,
        "series_rest_m": 3
      },
      "history": [] 
    }
  ]
}
```
*Note: The `history: []` array is reserved for future implementation of RPE (Rate of Perceived Exertion) logging and session history.*

---

## 15. Open Items

- [x] Final incline span calibration *(Closed: 3086/2943 pulses per percent).*
- [x] Verification of bus mapping *(Closed: Split-board MitM architecture verified).*
- [x] Injection timing and settle sequence verified with the current operational values.
- [x] Number 6 mapping verified: O1 `0x17`, O2 `0x81`.

---

## Appendix: Engineering Notes (Non-Critical)

*This section preserves historical context, architectural rationale, and optimization strategies that explain **why** the V4 specifications are designed the way they are. These notes are for engineering reference and do not override the functional rules above.*

### A.1 PC817 Optocoupler Switching Dynamics & Historical Blanking Thresholds

- **Switching Latency:** Under standard electrical loads, the PC817 exhibits a typical rise time (~4 µs) and a typical fall time (~18 µs). Under higher pull-up resistance (e.g., 10 kΩ), the release transition can stretch to 50–80 µs due to parasitic capacitance, which contributes to micro-glitches around the logic threshold.
- **Historical Heuristic:** Earlier iterations used a fixed blanking threshold of ~2 ms (≈500 Hz equivalent cutoff) as a simple rule-of-thumb to reject motor switching noise and EMI. This heuristic has been replaced by the dynamic, non-blocking software filters described in Section 6.2.

### A.2 Evolution of the Speed Filtering Algorithm (Deprecated Historical Approach)

Earlier prototypes tested a blocking “confirm and discard” mechanism inside the ISR, using a fixed `delayMicroseconds(500)` followed by a 20 ms lockout window.

```cpp
// DEPRECATED / HISTORICAL APPROACH — DO NOT USE IN PRODUCTION
void IRAM_ATTR isrSpeedDeprecated() {
    if (digitalRead(SPEED_IN) == LOW) {
        delayMicroseconds(500); // CRITICAL TIMING VIOLATION FOR O1/O2 BUS
        if (digitalRead(SPEED_IN) == LOW) {
            unsigned long now = micros();
            isr_intervalUs = now - isr_lastPulseUs;
            isr_lastPulseUs = now;
        }
    }
}
```

**Rationale for replacement:** The old method blocks Core 1 inside a time-critical interrupt, introducing jitter and risking missed frames on the fast O1/O2 buses. The current two-stage non-blocking filter (300 µs glitch rejection + 2000 µs lockout) uses timestamping via `micros()` so Core 1 returns immediately and remains responsive.

### A.3 Injection Diagnostics Ring Buffer (Development Note)

For bring-up and diagnostics (development only), implement a circular log buffer (ring buffer) with 64 entries on Core 1. Each entry should record the timestamp, observed O1 phase, generated O2 response byte, active command step, MUTE state, and result status. Expose the buffer via a debug-only HTTP endpoint or UART output, ensuring it does not interfere with Core 1 real-time behavior.

### A.4 Configuration Class Separation (4-Way Split)

To avoid unnecessary flash wear and prevent runtime state from being persisted as calibration data, split configuration strictly into four independent classes:

1. **Calibration Data:** Fixed hardware constants (e.g., pulses per percent, sensor offsets). Written only during manual calibration.
2. **User Preferences:** UI preferences (e.g., WiFi credentials, units). Written only on explicit change.
3. **User Profile:** Per-user profiles and targets.
4. **Runtime Volatile State:** Continuously changing variables (e.g., current speed, raw counters, current state, fault flags). Must never be written to flash/NVS during operation and should remain in RAM.

### A.5 BLE MTU Negotiation Optimization

To reduce latency and avoid payload splitting, the software may request MTU negotiation (e.g., `setMTU(64)`) at BLE stack initialization. This is an optional optimization and must be tested against the target client applications, as support and benefit vary by receiver Bluetooth stack.

### A.6 Future Enhancement: IMU-Based Incline Verification

The current incline model is based on homing and pulse counting (open-loop estimation). While this is sufficient for normal operation, it does not verify actual physical incline.

#### Proposed Enhancement

The LSM6DSOX IMU may be used to measure absolute tilt (gravity vector) of the treadmill deck to:

- verify the pulse-integrated incline estimate against the filtered physical deck angle
- detect drift or missed pulses
- provide automatic recalibration without requiring a full homing cycle

#### Potential Use Cases

- Detect mechanical slip or missed incline pulses
- Validate NVS-stored incline position after power loss
- Improve long-term accuracy without manual recalibration

#### Constraints

- IMU readings are subject to vibration noise from the motor and user impact
- Filtering and averaging are required (low-pass filtering / Kalman filtering)
- Must not interfere with Core 1 timing or real-time injection logic

#### Status

This feature is included in the approved V4 architecture but remains pending implementation, calibration, and regression testing.

### A.7 TXS0108E Power Sequencing Requirement (Bring-up Lesson)

During hardware bring-up, one TXS0108E was permanently damaged due to latch-up.

Cause:
- 5V arrived before 3.3V

Result:
- IC stopped translating signals (silent failure)

#### Required power-on sequence

1. No 5V present
2. Apply ESP32 power (3.3V)
3. Wait ~2 seconds
4. Apply 5V

#### Rule

3.3V MUST be stable before 5V is applied.

#### Final solution

- Hardware **10 kΩ pull-down (R1)** on OE
- Forces HIGH-Z state during power-up
- Prevents latch-up regardless of supply order
