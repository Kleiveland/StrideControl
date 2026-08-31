# StrideControl Technical Design Guide

This document contains verified physical measurements, protocol specifications, architecture rules, implementation decisions, current repository facts, and planned development constraints required to operate StrideControl safely and predictably.

> **Primary UI Goal**
>
> StrideControl provides a clean, tablet-based web interface for direct speed and incline control. Because the tablet completely covers the original console, the UI must display **measured belt speed** and the **hardware-tracked incline estimate** derived from physical hardware feedback, not only requested target values. When IMU verification is available and calibrated, the UI may additionally identify the incline estimate as IMU-verified.

## Evidence Classification

Use these evidence levels consistently:

```text
VERIFIED BY FINAL SOURCE INSPECTION
VERIFIED BY COMPILATION
VERIFIED BY EXECUTED HOST TEST
VERIFIED BY EXECUTED ESP32 TEST
VERIFIED BY RECORDED-DATA REPLAY
VERIFIED ON PHYSICAL TREADMILL
NOT EXECUTED
```

A clean build, zero warnings, comments, pseudocode, a compile probe, a Python model, or an implementation agent's own report is not functional verification.

## Current Repository Reality

At the latest read-only inspection:

- `ConsoleInterface`, `SpeedSensor`, `InclineSensor`, `ImuInterface`, `CsafeInterface`, and `RunnerDynamics` exist as libraries.
- These libraries are not instantiated or orchestrated by the current `src/main.cpp`.
- The current `src/main.cpp` is a standalone CLR Scanner V1.3 prototype for passive O1/O2 and buzzer analysis.
- `RunnerDynamics` has compiled and linked through a temporary compile probe.
- Actually executed RunnerDynamics functional tests: 0.
- No `test/` or `tests/` directory, host-native harness, embedded Unity runner, or recorded-data dataset was found.
- The existing tablet GUI is a single HTML/CSS/JavaScript mockup with an embedded simulator and is not connected to a production API.
- Git status, commit status, and push status must be checked explicitly.

This status is descriptive and must be rechecked after later repository changes.

---

## 1. Project Goal

The objective is to build a safe, reversible ESP32-S3 interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:

- Hosts a local tablet web interface.
- Allows direct user control of speed and incline.
- Measures physical belt speed.
- Tracks incline from homing and calibrated pulse integration.
- Parses machine state from CSAFE.
- Broadcasts telemetry through Bluetooth FTMS.
- Preserves the original console and safety path if the ESP32 loses power.
- Uses qualified buzzer feedback to verify injected commands.
- Retries only responses confidently classified as `NO_RESPONSE`.
- Restores numeric entry to a known state before restarting an ambiguous sequence.

This is a **non-destructive overlay system**, not a replacement for the main motor controller.

---

## 2. Safety and System Integrity Principles

### 2.1 Safety Design Principle

The system assumes that failures may occur, including MCU crashes, communication loss, sensor failure, injection failure, missing buzzer feedback, and ambiguous or duplicate numeric registration. The original treadmill control path must remain functional.

### 2.2 Real-Time Constraint Rule

Network, UI, Bluetooth, and storage processing must not delay real-time hardware handling. Timing-critical sensor capture, O1/O2 processing, active baseline generation, buzzer-edge capture, and command injection are isolated from UI and network work.

### 2.3 Fail-Safe Path Requirement

The MitM hardware defaults to passive pass-through when unpowered. Loss of ESP32 power must not interrupt console communication, safety functions, or normal treadmill operation.

### 2.4 ISR Design Rule

ISRs are minimal and non-blocking. They may timestamp and perform minimal validation, but must not log, update the UI, use networking, or perform serial I/O.

- The incline ISR may use a documented 10 microsecond validation delay in the hardware profile where that input is enabled.
- The buzzer ISR performs no blocking validation and stores only timestamped edges in a bounded ring buffer.

### 2.5 Control Versus Observation

Interface and analysis modules report observations. Control actions belong to explicit control modules.

```text
RunnerDynamics observes SideRails.
RunnerDynamics does not stop the treadmill.
```

No automatic stop or pause shall be introduced without separate safety requirements, failure analysis, and physical testing.

---

## 3. System Scope and Architecture

- The ESP32 owns UI, telemetry, FTMS, command planning, command injection, qualified buzzer ACK, and recovery.
- The treadmill mainboard retains motor power, incline power, and original safety logic.
- The web UI is stateless and consumes authoritative internal state.
- Instant Speed, Instant Incline, Num0-Num9, CLR, and Enter are actively generated with continuous O2 emulation.
- Speed+ and Speed- are actively generated through separate relay outputs.
- Physical Quick Start, Stop, Speed+/-, and Incline+/- are passively recognized from O1/O2 activity.
- No Phase D command is actively injected through O2.
- Incline targets are whole percentages because active Incline+/- outputs are not installed.

### 3.1 Layered Module Architecture

```text
Hardware and interface modules
            ↓
Analysis, calibration, and verification modules
            ↓
Application orchestration and stable snapshots
            ↓
System and workout domain models
            ↓
Services, APIs, GUI, and external adapters
```

1. **Hardware and interface:** `ConsoleInterface`, `SpeedSensor`, `InclineSensor`, `ImuInterface`, `CsafeInterface`, and later `HeartRateClient` under shared BLE ownership.
2. **Analysis, calibration, and verification:** `RunnerDynamics`, `SpeedCalibration`, and later `InclineVerifier`.
3. **Cross-cutting infrastructure:** executable tests, `DiagnosticsService` core, `SettingsService` core, `RecordedDataService` core, and `BleManager`.
4. **Coordination and domain models:** `ApplicationOrchestrator`, `ApplicationSnapshot`, `SystemManager`,`TreadmillController`, `WorkoutSession`, and `MaintenanceService`.
5. **Communication and presentation:** `FtmsService`, `WebServerManager`, API models, the existing Precision UI, the PC commissioning page, and later external adapters.

   

Prefer explicit update calls and read-only snapshots over a web of callbacks when timing and ownership permit it.

### 3.2 State and Responsibility Ownership

```text
SpeedSensor.speedKmh
= measured physical belt speed

RunnerDynamics.runnerSpeedKmh
= speed currently qualified for runner credit

MaintenanceService mechanical distance
= physical belt distance, including SideRails periods

RunnerDynamics validatedDistanceKm
= distance qualified as runner distance
```

These values must not be mixed.

Each public state field has one authoritative owner. Higher layers may cache snapshots but must not create a competing source of truth.

Reset operations retain precise meanings. A reset owned by one module must not silently erase data owned by another module.

### 3.3 Speed Measurement and Command Calibration Ownership

```text
SpeedSensor
= converts accepted physical pulses into measured belt speed

SpeedCalibration
= maps desired physical speed to the treadmill command most likely to achieve it

Command layer
= validates the request and executes the corrected command

TreadmillController
= validates speed and incline requests, applies SpeedCalibration when required, submits commands through ConsoleInterface, and tracks command completion.

ConsoleInterface
= owns numeric command execution, O2 ownership, ACK qualification, retry handling, CLR recovery, and command macro execution.

SettingsService
= persists approved calibration and last-known-good state

PC settings, diagnostics, and commissioning interface
= performs external reference calibration and commissioning
```

`SpeedCalibration` is a separate pure-logic library. It does not own GPIO, pulse capture, O1/O2 injection, relay output, NVS, WebServer routes, or GUI code.

The tablet training UI uses the calibrated achievable speed range but must not expose external speed commissioning controls.

---

## 4. Protocol and Hardware Mapping

### 4.1 Harness Measurements, Verified

- Black wire: +11.75 V DC constant
- Brown wire: 0 V reference
- Pin 6: 5.0 V DC constant
- Pin 12: 5.0 V DC constant
- Pin 7, speed: 11.4 V at rest; 8.97 Hz at 10 km/h and 22.3 Hz at 25 km/h
- Pin 11, incline: 4.682 V at rest; approximately 394 Hz while the incline motor moves
- Pin 10: 0.09 V at rest; unused

### 4.2 Verified O1/O2 Bus Model

O1 is the read-only scanner and phase-reference bus. O2 is the response bus used by the physical panel. During injection, the ESP32 isolates the physical O2 sources and generates the complete O2 response.

#### 4.2.1 O1 GPIO Mapping

| O1 bit | ESP32 GPIO |
|---:|---:|
| Bit 0 | GPIO 4 |
| Bit 1 | GPIO 5 |
| Bit 2 | GPIO 6 |
| Bit 3 | GPIO 7 |
| Bit 4 | GPIO 15 |

#### 4.2.2 Verified O1 Phases

| Phase | O1 value | Normal O2 baseline |
|---|---:|---:|
| Phase A | `0x0F` | `0x80` |
| Phase B | `0x17` | `0x80` |
| Phase C | `0x1B` | `0x80` |
| Phase D | `0x1D` | `0xC0` |
| Idle | `0x1F` | Time-dependent |

The old ROW_A to ROW_E model is superseded. Short intermediate O1 combinations may occur from parallel-bit skew and must not be treated as valid phases.

#### 4.2.3 Idle Response

When O1 enters `0x1F`, output O2 `0x00` for the first 1200 microseconds and `0xFF` afterward. The phase timer restarts on each new valid phase.

### 4.3 ESP32-S3 GPIO Mapping

| Signal | ESP32 GPIO | Function / status |
|---|---:|---|
| O1 bit 0-4 | 4, 5, 6, 7, 15 | Read-only phase input |
| O2 bit 0-7 | 41, 42, 8, 9, 10, 11, 12, 13 | Bidirectional O2 data |
| Shared MUTE | 21 | Active-low isolation of physical O2 sources |
| TXS0108E OE | 2 | Output enable |
| Buzzer ACK | 16 | Interrupt-driven closed-loop capture |
| Speed Sensor | 3 | Isolated pulse input, falling edge |
| Incline Sensor | 14 | Isolated pulse input |
| Speed+ Relay | 39 | Solid-state or relay output, active low |
| Speed- Relay | 38 | Solid-state or relay output, active low |
| CSAFE TX / RX | 17 / 40 | 9600 baud serial interface |
| I2C SDA / SCL | 47 / 48 | LSM6DSOX IMU and cadence |
| E-Stop, reserved | 18 | Hardware safety monitor |

**Integration requirement:** The final combined PCB must resolve GPIO16, GPIO14, and GPIO47 conflicts before all subsystems are enabled simultaneously.

### 4.4 O2 Bus and Isolation

O2 is changed atomically across both GPIO register banks. Sequential `digitalWrite()` calls are not suitable for phase-critical updates.

Ownership transition:

1. Calculate and preload baseline.
2. Assert MUTE LOW.
3. Enable all O2 outputs atomically.

Release transition:

1. Continue valid baseline through the reconnect point.
2. Return O2 to input atomically.
3. Wait 50 microseconds.
4. Release MUTE HIGH.

SN74HC4066N switches isolate the touch-console and side-panel O2 sources through one shared active-low MUTE signal. A 10 kOhm pull-up keeps the default state connected.

| MUTE | Physical sources |
|---|---|
| HIGH | Connected |
| LOW | Isolated |

The emergency-stop path is excluded.

OE starts LOW through a 10 kOhm pull-down. Configure OE LOW, MUTE HIGH, and O1/O2 as inputs before enabling OE.

With injection inactive, O1 and O2 are inputs, MUTE is HIGH, OE remains enabled after safe startup, and the original panels pass through normally.

### 4.5 Verified O2 Emulation Principle

The treadmill is controlled by continuous phase-dependent O2 emulation, not by replaying frames.

Baseline:

- `0x0F` to `0x80`
- `0x17` to `0x80`
- `0x1B` to `0x80`
- `0x1D` to `0xC0`
- `0x1F` to `0x00`, then `0xFF` after 1200 microseconds

The complete baseline continues before, between, and after steps, during ACK observation, retry cooldown, and recovery. The inverted initial cache value is only a sentinel and is never driven.

Closed-loop sequence:

1. Observe O1 with physical panels connected.
2. Obtain a valid phase transition and fresh IDLE.
3. Preload baseline.
4. Assert MUTE LOW and take O2 ownership atomically.
5. Generate complete baseline continuously.
6. Wait for a fresh target phase.
7. Generate the full active O2 byte.
8. Hold for the configured minimum and release after leaving the target phase.
9. Return immediately to baseline.
10. Classify the qualified buzzer response.
11. Continue, retry, or recover according to classification.
12. Keep MUTE and baseline active throughout the O2 macro.
13. Send Enter only after the complete numeric sequence is confirmed.
14. Reconnect in fresh IDLE, return O2 to input, wait 50 microseconds, and release MUTE.
15. Wait before relay adjustment or passive monitoring resumes.

### 4.6 Verified Button Response Matrix

Active outputs:

- O2: Instant Speed, Instant Incline, Num0-Num9, CLR, Enter
- Separate relays: Speed+, Speed-

Passively recognized:

- Quick Start, Stop, Speed+/-, Incline+/-

Not actively generated:

- Quick Start, Stop, Incline+/-, or any Phase D function through O2

| Function | O1 | O2 | Current use |
|---|---:|---:|---|
| Instant Incline | `0x0F` | `0xC0` | Active O2 |
| Number 1 | `0x0F` | `0x82` | Active O2 |
| Number 4 | `0x0F` | `0x84` | Active O2 |
| Number 5 | `0x0F` | `0x90` | Active O2 |
| Number 7 | `0x0F` | `0x88` | Active O2 |
| Number 8 | `0x0F` | `0xA0` | Active O2 |
| Instant Speed | `0x17` | `0xC0` | Active O2 |
| Enter | `0x17` | `0x88` | Active O2 |
| Number 0 | `0x17` | `0xA0` | Active O2 |
| Number 2 | `0x17` | `0x90` | Active O2 |
| Number 3 | `0x17` | `0x84` | Active O2 |
| Number 6 | `0x17` | `0x82` | Active O2 |
| Number 9 preload | `0x17` | `0x81` | Active O2 |
| Number 9 active | `0x1B` | `0x81` | Active O2 |
| Fan On/Off | `0x1B` | `0xA0` | Mapped only |
| Fan High | `0x1B` | `0x90` | Mapped only |
| Fan Low | `0x1B` | `0xC0` | Mapped only |
| Speed+ | `0x1D` | `0xC4` | Passive; active via relay |
| Speed- | `0x1D` | `0xC8` | Passive; active via relay |
| Incline+ | `0x1D` | `0xC1` | Passive only |
| Incline- | `0x1D` | `0xE0` | Passive only |
| Quick Start | `0x1D` | `0xD0` | Passive only |
| Stop | `0x1D` | `0xC2` | Passive only |
| CLR | `0x0F` | `0x81` | Active O2 |

Number 9 outputs `0x81` in Phase B as preload and continues `0x81` through Phase C.

For passive Phase D recognition, O1/O2 are read atomically. A known Phase D combination must remain stable for approximately 30 ms. One `HardwareEvent` is generated per activation. Held buttons are not repeated. Monitoring is suspended during O2 ownership.

### 4.7 Timing, Watchdog, Buzzer ACK, and Recovery

| Parameter | Value |
|---|---:|
| General digit hold | 82 ms |
| First digit hold | 82 ms |
| Digit pause | 150 ms |
| First-digit pause | 150 ms |
| Instant hold | 200 ms |
| Post-Instant pause | 250 ms |
| Pre-Enter pause | 200 ms |
| Enter hold | 82 ms |
| Enter pause | 50 ms |
| Relay hold / pause | 80 / 80 ms |
| ACK timeout | 450 ms |
| Retry cooldown | 300 ms |
| Local retry | One additional attempt |
| Full sequence restarts | Maximum two |
| Sync / IDLE timeout | 30 / 30 ms |
| Phase watchdog | 35 ms |
| Final IDLE timeout | 35 ms |
| O2/MUTE transition | 50 microseconds |
| O2-to-relay delay | 150 ms |

At 82 ms, screening produced 131/140 `NORMAL_SINGLE`, 9/140 `NO_RESPONSE`, and no `NORMAL_LONG` or multi event.

Buzzer classifications:

- `NORMAL_SINGLE`: one normal segment
- `NO_RESPONSE`: no qualified event
- `NORMAL_LONG`: one segment longer than 110 ms, ambiguous
- `MERGED_MULTI`: multiple segments in one envelope
- `DISTINCT_MULTI`: multiple envelopes
- `EDGE_LOSS`: dropped edges
- `INVALID`: unsafe classification

Measured 161 ms and 171 ms envelopes are `NORMAL_LONG`.

An ACK requires a newer sequence and `event.startUs >= pressStartUs`. The evaluator rejects edge loss, multiple events, multiple segments, and a second event during the guard interval.

Only an unambiguous `NO_RESPONSE` permits one direct retry. No direct retry follows `NORMAL_LONG`, multi, `EDGE_LOSS`, or `INVALID`.

### 4.8 CLR-Based Numeric Recovery

CLR is the authoritative method for returning the numeric-entry buffer to a known empty state.

Verified physical signature:

- Target phase: Phase A
- O1: `0x0F`
- Active O2: `0x81`
- Preload: none observed
- Physical buzzer response: approximately 100.7 ms
- Qualified segments: one
- Verified after zero, one, and multiple entered digits
- Verified in Instant Speed and Instant Incline
- Verified to clear without committing the entered value

Recovery sequence:

```text
Ambiguous or duplicate result
→ stop current numeric sequence
→ send CLR once
→ verify CLR buzzer response
→ restart the complete intended sequence
→ send Enter only after every restarted digit is confirmed
```

Each complete sequence restart uses exactly one CLR. CLR is not retried independently within the same recovery attempt.

- Maximum direct retry per digit: one
- Maximum complete sequence restarts per macro: two

If CLR does not receive qualified `NORMAL_SINGLE`:

- Do not send additional digits.
- Do not send Enter.
- Do not perform later Speed+ or Speed- correction.
- Abort the macro.
- Return `CLEAR_FAILED`.

Initial injected CLR timing:

- CLR hold: 82 ms
- Post-CLR pause: 300 ms

Physical CLR mapping is verified. The injected 82 ms timing and complete automatic recovery still require active injection and end-to-end regression testing.

Enter is allowed only after Instant and all digits are confirmed and any retry or CLR recovery is complete. Ambiguous Enter behavior remains an open verification item.

On watchdog or hardware failure, release relays, return O2 to input atomically, wait 50 microseconds, release MUTE, settle, clear ownership, and report failure.

### 4.9 Command and Macro Execution

Speed command range accepted by the console is 0.8 to 25.0 km/h. The user-facing range may be lower after calibration.

The integer base is entered through O2 and confirmed before Enter. Fractional correction uses Speed+/- relays after controlled reconnect.

- 12.6 gives base 13, then four Speed- pulses
- 12.4 gives base 12, then four Speed+ pulses

Incline range is 0 to 15%, whole percentages only. Non-integer targets are rejected. There is no active Incline+/- output.

Instant, numeric entry, ACK, retry, CLR recovery, sequence restart, and Enter remain under one continuous O2 ownership period. Speed relay pulses occur after controlled reconnect and 150 ms settling.

Requests use bounded FreeRTOS queues with states `queued`, `started`, `step`, `completed`, `rejected`, and `failed`. Status also reports ACK classification, retry, CLR recovery, sequence restart, and recovery failure.

Physical Speed+/- and Incline+/- are ignored unless the belt is moving. Physical Quick Start, Stop, Speed+/-, and Incline+/- are recognized passively and never automatically reinjected.

Physical Quick Start homes incline to 0%. Measured travel is approximately 49 seconds upward and 48 seconds downward.

---

## 5. Sensor Interpretation

### 5.1 Data Authority

Speed comes from Pin 7, incline from the homed pulse-integrated tracker, and machine state from CSAFE. The IMU verifies incline but does not replace movement tracking.

### 5.2 Cadence and IMU

The LSM6DSOX is mounted to the moving deck, derives cadence from footstrike vibration, and provides filtered angle verification. The long I2C cable uses shielding and an LTC4311. A sensor fault returns cadence to 0 without affecting control.

The historical GPIO47 SDA allocation conflicts with the current Speed- relay and must be resolved in final hardware.

### 5.3 Runner Presence

The product goal is that measured belt movement without qualified runner footstrikes eventually results in no runner credit. FTMS speed is then forced to zero and validated runner distance pauses without changing treadmill control state.

The detailed grace periods, state transitions, regularity requirements, SideRails qualification, and resume behavior belong to `RunnerDynamics`. Higher layers must not replace this state machine with a direct speed/cadence condition.

Current evidence:

```text
C++ IMPLEMENTATION COMPILED AND LINKED
ACTUALLY EXECUTED FUNCTIONAL TESTS: 0
FUNCTIONAL BEHAVIOR NOT RUNTIME-VERIFIED
```

---

## 6. Speed Sensor

### 6.1 PC817 Interface

A PC817 collector uses a 10 kOhm pull-up to 3.3 V. Speed uses a 10 kOhm LED-side resistor at approximately 1.1 mA. Incline uses 1 kOhm at approximately 3.8 mA.

### 6.2 Software Filter

Timestamp filtering uses approximately 300 microseconds micro-glitch rejection and 2000 microseconds lockout. No blocking ISR delay is used for speed.

### 6.3 Expected Signal

Approximately 2 Hz at 1 km/h, 9 Hz at 10 km/h, and 22 Hz at 25 km/h, with one accepted falling edge per rotation.

### 6.4 Continuity and Faults

A missing pulse timeout forces speed to zero. RUNNING with no valid pulses for more than 2 seconds logs a mismatch fault.

### 6.5 Effective Zero Versus Belt Moving

Keep these concepts separate:

- Effective zero-speed epsilon: approximately 0.01 km/h
- Behavioral belt-moving threshold: approximately 0.5 km/h

Only effective zero with an operational stopped/ready status may be exempted from pulse-age freshness. A low non-zero speed still requires a fresh pulse-derived measurement.

---

## 7. Incline Sensor

Pin 11 emits approximately 394 Hz during movement. Incline is calculated from homing and direction-specific pulse integration, with optional IMU verification after movement.

The isolated concept uses PC817, 1 kOhm LED resistance, and 10 kOhm pull-up. BSS138 remains a legacy troubleshooting option. GPIO14 conflicts with the current Speed+ relay and must be reassigned in integrated hardware.

Physical Quick Start homes to 0%. When movement pulses stop, the tracker is set to 0.0%.

Store incline when movement stops or machine state becomes STOPPED. A `Re-home Incline` action is mandatory because NVS cannot detect untracked movement while powered off.

---

## 8. Calibration and Software Constants

### 8.1 Speed Sensor Calibration

Verified reference values:

- 10 km/h = 8.97 Hz
- 25 km/h = 22.3 Hz
- initial conversion: `km/h = Hz * 1.1148`
- 1 meter = 3.2292 pulses

External reference calibration may adjust the physical speed model. The externally calibrated `SpeedSensor.speedKmh` is the authority for actual belt speed.

### 8.2 SpeedCalibration Library

Create a separate library:

```text
lib/SpeedCalibration/
```

Responsibilities:

- command correction map
- automatic learning candidates
- validated calibration points
- linear interpolation
- calibration provenance and confidence
- maximum achievable physical speed
- read-only calibration snapshot

It does not measure pulses, execute commands, own GPIO, write NVS, or own HTML/API code.

### 8.3 Automatic Command Learning

Example:

```text
User request:              15.0 km/h
Previous treadmill command: 15.0 km/h
Stable measured result:     14.8 km/h
Future corrected command:   approximately 15.2 km/h
```

The correction is applied only the next time that speed is requested. The active command is not continuously adjusted while the belt is running.

A learning candidate may be created only when:

- SpeedSensor is valid and fresh.
- The command is known.
- The normal transition period is complete.
- Speed is stable for approved observation criteria.
- No command, relay correction, startup, shutdown, recovery, or unresolved control operation is active.
- The point is within the valid calibration range.

Learned observations remain in RAM while the belt runs. Persistent writes occur only when the belt is stopped and only through `SettingsService`.

A single observation must not immediately overwrite an established point. Minimum evidence, outlier rejection, bounded adaptation, and acceptance confidence must be specified and tested before implementation approval.

### 8.4 Correction Table and Interpolation

Each calibration point distinguishes:

- desired physical speed
- treadmill command
- measured physical speed
- source, automatic or external commissioning
- validity
- confidence

Interpolate the required command linearly between surrounding validated points.

Do not silently extrapolate outside the validated range unless a bounded extrapolation policy is separately approved and tested.

The corrected command cannot exceed the maximum command accepted by the treadmill, currently 25.0 km/h.

### 8.5 PC-Only External Reference Calibration

External reference calibration belongs exclusively in the PC-based **Settings / Diagnostics / Commissioning** HTML interface. It must not be exposed in the tablet training UI.

Commissioning workflow:

1. Select a treadmill command point.
2. Run and wait for stable belt speed.
3. Measure speed using an external reference.
4. Enter the external measured speed on the PC commissioning page.
5. Compare external speed with `SpeedSensor.speedKmh`.
6. Update the candidate physical speed scale and command map.
7. Validate the complete candidate calibration.
8. Stop processing safely and apply the complete configuration.
9. Persist only after successful application.
10. Restore last-known-good calibration if validation or application fails.

The page must distinguish:

- factory/default calibration
- active calibration
- stored calibration
- temporary commissioning values
- automatically learned candidates
- externally verified points
- validation state
- last-known-good calibration

The tablet receives only the resulting calibrated speed, corrected command behavior, and achievable limits.

### 8.6 Maximum Achievable Speed

The user-facing maximum is the highest verified physical speed that the treadmill can achieve, not necessarily the highest numeric command accepted by the console.

Example:

```text
Maximum accepted command:  25.0 km/h
Stable measured speed:     24.8 km/h
Maximum user-selectable:   24.8 km/h
```

If the physical SpeedSensor scale has been externally validated and command 25.0 consistently produces 24.8 km/h, then 24.8 km/h is the maximum achievable speed. The system cannot offer corrected 25.0 km/h because the treadmill command cannot exceed 25.0 km/h.

This maximum is used consistently by:

- command validation
- tablet speed controls and quick-key availability
- WorkoutSession limits
- public control API
- FTMS capability and target ranges
- PC commissioning interface

The system must not advertise an unachievable speed, relabel 24.8 as 25.0, or rescale the display to hide the physical limit.

Reducing the persisted maximum requires validated evidence. Stability, repetition, acceptance confidence, and external-calibration precedence must be defined and tested.

### 8.7 Calibration Precedence

1. Valid external commissioning points are authoritative for physical speed scaling.
2. Accepted automatic learning may refine future command correction within that speed model.
3. Unvalidated candidates do not replace last-known-good data.
4. Invalid, stale, unstable, or recovery-period measurements do not update calibration.
5. Calibration never injects commands independently.
6. The command layer validates and executes requests.
7. Calibration failure cannot disable the original console or fail-passive path.
8. Missing calibration falls back to validated defaults or last-known-good state and reports degraded calibration status.

### 8.8 Incline Calibration

- `PULSES_PER_PERCENT_UP = 3086.0f`
- `PULSES_PER_PERCENT_DOWN = 2943.0f`

---

## 9. Hardware Interface Choices

### 9.1 MCU

ESP32-S3 N16R8, dual-core.

### 9.2 Level Translation

Two TXS0108E devices are used in the prototype. OE starts LOW and is enabled only after safe GPIO configuration.

### 9.3 MUTE Gate

SN74HC4066N switches, shared active-low MUTE, 10 kOhm pull-up, fail-passive behavior, emergency stop excluded.

### 9.4 Power

HLK-PM01 with mains fusing and isolation. Target continuous use below 70 to 80% rating. Verify sustained load, radio-induced rail sag, and thermal stability.

### 9.5 CSAFE

RJ45 to MAX3232 at 9600 8-N-1. GPIO16 conflict with buzzer must be resolved.

### 9.6 Future Improvements

- Replace Speed+/- relays with a deterministic solution such as PhotoMOS, isolated transistor, or appropriate open-drain interface after electrical measurement.
- A future board may add independent protected fail-inactive Incline+/- outputs.
- Evaluate explicit-direction translators, unidirectional stages, bus switches, Schmitt-trigger receivers, or dedicated drivers instead of TXS0108E.
- Use shorter bus paths, continuous ground, controlled connectors, local decoupling, test points, and optional damping footprints.

---

## 10. Software Architecture Rules

### 10.1 Dual-Core Split

Core 0 owns Wi-Fi, web, LittleFS, BLE, logging, and IMU/cadence. Core 1 owns injection, panel monitoring, command execution, and watchdog responsibility. Queues, atomics, and short critical sections protect shared data.

Final task placement must be verified during orchestration design.

### 10.2 Rate Control and Backpressure

Capture is interrupt-driven. Consumers read at fixed rates. Stale telemetry may be dropped, but pulse counts and homing state may not. Any dropped sample data requires diagnostics.

### 10.3 Filesystem, Persistence, and OTA

Use LittleFS, batched writes, atomic configuration replacement, separate firmware/filesystem artifacts, a custom partition table, and dual app partitions if OTA rollback is required.

### 10.4 AP Recovery

After repeated Wi-Fi failure, start AP mode with a portal for credentials, FTMS name, and incline re-home.

### 10.5 Application Orchestration

The intended flow is:

```text
ImuInterface ─────────┐
SpeedSensor ──────────┤
InclineSensor ────────┤
CsafeInterface ───────┼──> ApplicationOrchestrator
HeartRateClient ──────┤              │
SpeedCalibration ─────┘              ├──> RunnerDynamics
                                     ├──> InclineVerifier
                                     └──> ApplicationSnapshot
```

The orchestrator owns ordering and scheduling, not algorithms.

It must define:

- update frequency
- maximum IMU block size
- sample-buffer ownership
- backlog and dropped-data policy
- task priority and core placement
- mutex boundaries
- source of `nowMs`
- snapshot freshness
- lifecycle and reset order
- configuration apply behavior

### 10.6 Stable Snapshots

`ApplicationSnapshot` is a small read-only transport model that aggregates authoritative subsystem snapshots without becoming a new owner.

Do not expose private algorithms, mutexes, raw pointers, or storage-specific structures through public APIs.

### 10.7 Diagnostics Core

Minimum scope:

- module health
- faults and notices
- timestamps
- counters
- build metadata
- non-blocking registration
- read-only snapshot

Diagnostics must not depend on continuous Serial output or duplicate module-owned state.

### 10.8 SettingsService Core

SettingsService owns:

- schema and version
- factory defaults
- load/save
- module configuration sections
- full validation
- staged changes
- apply/rollback
- last-known-good configuration
- calibration persistence

Modules validate and use their own configuration. Modules do not write NVS directly.

Safe application:

```text
load candidate
→ syntactic validation
→ module validation
→ stop processing
→ end affected module
→ begin with complete new configuration
→ verify success
→ persist
→ resume


WorkoutResumeSettings

Examples:

- immediateResumeThresholdSec
- reEntryRecommendationThresholdSec
- warmupRecommendationThresholdSec
- extendedWarmupThresholdSec
- skipToRecoveryRemainingFraction

These values are configurable through the PC Settings interface.

WorkoutSession consumes the settings but does not own persistence.

```

On failure, restore the previous configuration, restart the module, and report the error.

### 10.9 RecordedDataService Core

The recording format supports deterministic replay and includes at least:

- IMU timestamp and sequence
- longitudinal, lateral, and vertical acceleration
- sample validity
- SpeedSensor snapshot
- incline snapshot
- CSAFE context
- application time
- RunnerDynamics output
- treadmill command and SpeedCalibration observation
- calibration source and active calibration version
- manual test marker

Markers may include:

- `EMPTY_BELT`
- `WALKING`
- `RUNNING`
- `ENTER_SIDE_RAILS`
- `ON_SIDE_RAILS`
- `RESUME_RUNNING`
- `INCLINE_MOVING`
- `FRAME_IMPACT`
- `EXTERNAL_SPEED_REFERENCE`

### 10.10 BLE Ownership

```text
BleManager
├── HeartRateClient
└── FtmsService
```

`BleManager` owns initialization, lifecycle, scanning/advertising coordination, connection events, central/peripheral concurrency, and BLE health.

`HeartRateClient` provides heart rate, validity, data age, connection state, selected sensor, and available battery status.

`FtmsService` is implemented only after stable WorkoutSession, SystemManager, incline, SpeedCalibration, and command contracts exist.

### 10.11 Workout and Maintenance

WorkoutSession owns:

- workout lifecycle
- workout suspension and resume
- interval progression
- workout-scoped telemetry
- workout summary

WorkoutSession consumes authoritative data from:

- RunnerDynamics
- ApplicationSnapshot
- TreadmillController

WorkoutSession does not become a new owner of:

- cadence
- step count
- validated distance
- runner speed
- incline state
- heart rate

WorkoutSession shall never:

- initiate treadmill movement from standstill
- automatically resume high intensity after interruption
- automatically restart a suspended workout

WorkoutSession may only resume after explicit user approval.

### 10.11.1 Workout Suspension and Resume

Stopping treadmill motion during an active workout does not terminate
the workout automatically.

The workout enters:

Suspended

State.

WorkoutSession stores:

- current step
- elapsed step time
- completed fraction
- pause timestamp
- workout progress context

When treadmill motion is later detected again:

WorkoutSession evaluates the pause duration and workout state.

It may recommend:

- Resume Workout
- Resume With Re-Warmup
- Restart Current Interval
- Skip To Recovery
- End Workout

The final decision belongs to the user.

WorkoutSession must never automatically resume directly into a work interval.

### 10.11.2 Re-Warmup Recommendation

WorkoutSession may generate resume recommendations using configurable
SettingsService thresholds.

Typical examples:

pause < 60 s
    immediate resume may be recommended

pause > configured re-entry threshold
    short re-entry may be recommended

pause > configured warmup threshold
    re-warmup may be recommended

pause > configured extended threshold
    restart of interval or extended warmup may be recommended

These recommendations are advisory only.

WorkoutSession must not automatically execute them.

### 10.12 AI-Assisted Development Protocol

1. Read-only repository inspection.
2. Short change contract.
3. Narrow implementation of one logical group.
4. Mechanical reread and search of final source.
5. Clean build reported only as compilation evidence.
6. Executed test with runtime output.
7. Independent read-only review.

The implementation agent's own report is not approval.

Stop and review when:

- more than three existing modules require modification
- a public type changes unexpectedly
- two modules claim the same state
- runtime evidence conflicts with the model
- an interface workaround is proposed
- this guide would be changed merely to fit generated code
- permanent test hooks are required
- samples can be dropped without diagnostics
- configuration can become partially applied
- the GUI must understand private algorithm behavior
- safety behavior is proposed without separate requirements

---

## 11. CSAFE Rules

- 9600 baud, 8-N-1
- Keep-alive: `0xF1 0x85 0x85 0xF2`
- Status: `0xF1 0x80 0x80 0xF2`
- Poll every 200 to 250 ms
- Do not merge frames
- Never send `0xAA`, `0xA5`, or `0x9C`
- Reset parser on `0xF1`, parse on `0xF2`
- Application code never reads UART directly

---

## 12. Timing and Safety Rules

- Incline motion timeout: 60 seconds
- Startup sync suppression: 3.5 seconds
- Resume debounce: 700 ms
- CSAFE watchdog: 1500 ms
- Ghost-running mismatch: 2 seconds
- STARTING command expiry: 5 seconds
- Block Enter after unresolved digit, ambiguous ACK, edge loss, failed CLR, or exceeded recovery limit

---

## 13. Bluetooth FTMS Rules

- NimBLE-Arduino
- 1 Hz broadcast
- Speed in 0.01 km/h units
- Incline in 0.1% units
- Positive elevation gain from interval distance and incline
- Cadence included when available
- Dual-role HR client and FTMS server
- HR MAC stored through SettingsService/NVS ownership
- All BLE work on Core 0
- Advertised maximum and target range use calibrated maximum achievable speed

---

## 14. GUI Architecture and Interval Coach

### 14.1 Precision UI

Use Vanilla JavaScript, WebSocket telemetry, `pointerdown`, eight-button speed and incline grids, HVILE/DRAG presets, persistent manual controls, measured speed, tracked incline, and optional IMU verification.

The existing `StrideControl Precision UI` / `TabletGuiMockup` is the visual starting point and must evolve rather than be rebuilt.

- Remove the embedded simulator from the production data path.
- Retain the disconnected-state presentation.
- Drive disconnected state from transport connection, snapshot freshness, SystemManager, and relevant subsystem faults.
- Use calibrated achievable speed limits in controls and quick keys.
- Do not expose external calibration, calibration-table editing, learned-point approval, or commissioning in the tablet UI.
- Test the tablet UI on the standard iPhone 14 Pro layout as well as the target tablet.

  WorkoutSession publishes suspension and resume context through snapshots.

The GUI is responsible for presenting:

- suspension state
- pause duration
- workout progress
- resume recommendations

WorkoutSession provides facts.

The GUI decides how they are presented.

### 14.2 PC Settings, Diagnostics, and Commissioning UI

This is a separate PC-oriented HTML interface for:

- external speed reference calibration
- SpeedCalibration table inspection
- automatic learning candidate approval
- maximum achievable speed commissioning
- configuration validation and rollback
- raw IMU and sensor telemetry
- diagnostics and counters
- data recording and export
- incline calibration and verification

The page distinguishes defaults, stored, temporary, active, externally verified, automatically learned, and last-known-good values.

### 14.3 Interval Coach

Visual and auditory only, never automated control. Includes focus mode, Web Audio cues, ETA, phase banners, and RPE prompt.

### 14.4 User Profile Schema

```json
{
  "users": [{
    "name": "Kristian",
    "presets": {"hvile": 6.0, "drag": 16.0},
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
  }]
}

WorkoutSession publishes suspension and resume context through snapshots.

The GUI is responsible for presenting:

- suspension state
- pause duration
- workout progress
- resume recommendations

WorkoutSession provides facts.

The GUI decides how they are presented.

```

---

## 15. Verification and Open Items

### 15.1 Completed or Strongly Supported

- [x] Incline calibration 3086/2943 pulses per percent
- [x] Split-board MitM mapping
- [x] O1/O2 phase and baseline model
- [x] Atomic O2 data and direction control
- [x] Number 6: `0x17` / `0x82`
- [x] Number 9 preload
- [x] Active baseline and phase-coherent release
- [x] Qualified buzzer capture and causal ACK
- [x] Retry after qualified `NO_RESPONSE`
- [x] Passive Phase D recognition
- [x] Speed fractional adjustment through relays
- [x] 82 ms numeric baseline
- [x] CLR physical mapping: Phase A, O1 `0x0F`, O2 `0x81`
- [x] CLR verified in Instant Speed and Instant Incline
- [x] CLR verified after zero, one, and multiple digits
- [x] CLR verified to clear without committing
- [x] CLR response approximately 100.7 ms with one segment
- [x] No CLR preload requirement observed

### 15.2 Hardware Open Items

- [ ] Validate ambiguous Enter behavior
- [ ] End-to-end speed and incline regression
- [ ] Resolve integrated GPIO conflicts
- [ ] Future solid-state Speed+/- outputs
- [ ] Future active Incline+/- outputs
- [ ] Future deterministic TXS0108E replacement
- [ ] Final PCB/wiring revision
- [ ] Production qualification

### 15.3 Software and Integration Open Items

- [ ] Confirm a known Git baseline and commit hash
- [ ] Add executable test infrastructure without permanent hooks
- [ ] Execute deterministic RunnerDynamics and interface-contract tests
- [ ] Implement and test the separate SpeedCalibration library
- [ ] Define and test automatic learning acceptance rules
- [ ] Implement PC-only external speed commissioning
- [ ] Implement maximum-achievable-speed calibration
- [ ] Define recorded-data format before physical capture
- [ ] Implement Diagnostics core and SettingsService core
- [ ] Implement InclineVerifier without control authority
- [ ] Implement shared BleManager and HeartRateClient
- [ ] Implement ApplicationOrchestrator and ApplicationSnapshot
- [ ] Implement SystemManager without duplicating subsystem state
- [ ] Implement WorkoutSession before FTMS
- [ ] Implement recorded-data replay before aggressive tuning
- [ ] Implement MaintenanceService with separate mechanical distance
- [ ] Define stable WebServer/API contracts before GUI connection
- [ ] Connect the Precision UI and remove simulator from production
- [ ] Perform recorded-data, HIL, and physical treadmill validation

### 15.4 Required Development Sequence

1. Known repository baseline
2. Executable test infrastructure
3. RunnerDynamics and interface-contract tests
4. SpeedCalibration library and deterministic tests
5. Recorded-data format core
6. Diagnostics core
7. SettingsService core
8. InclineVerifier
9. BleManager core
10. HeartRateClient
11. ApplicationOrchestrator and ApplicationSnapshot
12. SystemManager
13. WorkoutSession
14. Recorded-data replay
15. MaintenanceService
16. FtmsService
17. WebServerManager and API model
18. PC settings, diagnostics, calibration, and commissioning page
19. Existing Precision UI integration
20. External adapters
21. SafetySupervisor only after separate safety analysis
22. Combined software testing
23. Recorded-data validation
24. Hardware-in-the-loop
25. Physical treadmill testing and tuning

Governing rule:

```text
One owner per state.
One logical change per test.
One integration per build.
Executed runtime tests before new feature expansion.
Stable APIs before GUI integration.
Recorded data before aggressive tuning.
SettingsService before the commissioning page.
WorkoutSession before FTMS.
Safety analysis before automatic control actions.
```

### 15.5 CLR Verification Plan

Validate CLR in Instant Speed and Instant Incline after zero, one, and two digits. Confirm repeated operation, buzzer response, and no value commit.

Recommended sequence tests: 10, 11, 12, 15, 20, 22, and 25.

Acceptance priority:

1. No Enter after an unverified sequence.
2. No direct retry after ambiguous or multi response.
3. Reliable CLR recovery.
4. Correct final value.
5. High success after at most one retry.
6. Low completion time.

---

## Appendix A. Engineering Notes

### A.1 PC817 Dynamics

The 10 kOhm pull-up can extend release transitions and cause threshold noise. Timestamp filtering replaces historical blanking heuristics.

### A.2 Deprecated Blocking Speed Filter

The old 500 microsecond blocking ISR confirmation is prohibited because it risks missed O1/O2 timing.

### A.3 Injection Diagnostics

A development-only 64-entry ring buffer may record O1, O2, MUTE, step, ACK, retry, and recovery status.

### A.4 Configuration Classes

Separate calibration, user preferences, user profiles, and volatile runtime state. Runtime state is not continuously written to flash.

### A.5 BLE MTU

An MTU such as 64 may be requested after client compatibility testing.

### A.6 IMU Incline Verification

The IMU may verify pulse-integrated incline, detect drift, and support recalibration. Vibration filtering and regression testing remain required.

### A.7 TXS0108E Power Sequencing

A device was damaged when 5 V preceded 3.3 V. OE pull-down and high-impedance startup are mandatory.

### A.8 Software Test and Replay Strategy

Select the first test target after inspecting actual Arduino and FreeRTOS dependencies. Options include PlatformIO native with a minimal platform seam or embedded Unity tests on ESP32. Do not alter production behavior merely for test convenience.

Minimum deterministic coverage:

- configuration validation
- valid and invalid sample blocks
- empty updates and freshness expiry
- timestamp and sequence discontinuity and rollover
- first impact and first valid pair
- Candidate, Active, SideRailsCandidate, SideRails, and resume
- cadence, regularity, Walking, and Running
- stopped speed, stale non-zero speed, and pulse age
- distance and reset semantics
- transition counters and lifecycle state
- CSAFE timeout as confidence context
- automatic speed-command learning without live correction
- calibration interpolation and boundaries
- external reference application and rollback
- maximum achievable speed at the 25.0 command ceiling

### A.9 Design Decision Summary

```text
Use approximately 82 ms as the numeric hold baseline.
Retry once only after qualified NO_RESPONSE.
Treat long, multiple, invalid, or edge-loss responses as unknown buffer state.
Use CLR after physical mapping and verification.
Send Enter only after all digits are confirmed.
Keep Speed+/- on separate relays in current hardware.
Passively read physical Quick Start, Stop, Speed+/-, and Incline+/-.
Do not perform active Phase D O2 injection.
Keep incline targets to whole percentages.
Retain speed, incline, IMU, CSAFE, FTMS, GUI, persistence, and safety architecture.
Plan solid-state side-button outputs and deterministic level translation.
Use a separate SpeedCalibration library.
Do not adjust the active speed continuously from AutoCal.
Apply learned command correction only to future requests.
Perform external speed calibration only from the PC commissioning interface.
Limit user-selectable speed to the highest verified physically achievable speed.
Do not advertise or display speeds the treadmill cannot achieve.
```
