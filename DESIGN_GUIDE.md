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

- Embedded Unity test infrastructure is active under `test/`.
- Hardware-in-the-loop execution exists on ESP32-S3 (`seeed_xiao_esp32s3_testbench`).
- Verified ESP32 test suites include:
  * `test_virtual_speed_sensor`
  * `test_virtual_incline`
  * `test_virtual_console`
  * `test_virtual_runner`
  * `test_composite_simulator`
- A dedicated testbench simulator architecture exists:
  * `VirtualTreadmill`
  * `VirtualSpeedSensorAdapter`
  * `VirtualInclineAdapter`
  * `VirtualConsoleAdapter`
  * `VirtualRunnerAdapter`
  * `TreadmillSimulatorComposite`
- Deterministic `ExternalStep` orchestration exists through `ApplicationOrchestrator`.
- Web interfaces hosted from LittleFS:
  * `index.html` = Responsive workout session companion for phone and PC (planning, history, export)
  * `tablet.html` = Dedicated full-screen tablet training UI (live telemetry, quick keys, interval coach)
  * `simulator.html` = Virtual treadmill and runner testbench stimulus panel (`STRIDECONTROL_TESTBENCH` only)
  * `service.html` = PC hardware calibration, commissioning, and diagnostics console
- Compile-time mDNS hostnames:
  * Production (`esp32-s3-devkitc-1`): `http://sportsmaster.local`
  * Testbench (`STRIDECONTROL_TESTBENCH`): `http://stridecontrol.local`
- Simulator functionality is isolated behind:
  * `STRIDECONTROL_TESTBENCH`
- Production and testbench builds are compile-time segregated.
- Git status, commit status, and push status must still be checked explicitly.

This status is descriptive and must be rechecked after later repository changes.

---

## 1. Project Goal

The objective is to build a safe, reversible ESP32-S3 interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:

- Hosts a local tablet web interface.
- Allows direct user control of speed and incline.
- Measures physical belt speed.
- Tracks incline from homing and calibrated pulse integration.
- Parses machine state from CSAFE.
- Broadcasts telemetry through Bluetooth FTMS and RSC.
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

Observation modules never generate control actions.

Examples:

- RunnerDynamics does not stop the treadmill.
- InclineVerifier does not command incline movement.
- DiagnosticsService does not clear faults autonomously.
- FtmsService does not issue treadmill commands.

No automatic stop or pause shall be introduced without separate safety requirements, failure analysis, and physical testing.

### 2.6 Future SafetySupervisor

Any future supervisory safety layer shall be architecturally separate from:

- RunnerDynamics
- FtmsService
- GUI
- DiagnosticsService

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
4. **Coordination and domain models:** `ApplicationOrchestrator`, `ApplicationSnapshot`, `SystemManager`, `TreadmillController`, `WorkoutSession`, and `MaintenanceService`.
5. **Communication and presentation:** `RscService`, `FtmsService`, `WebServerManager`, API models, the existing Precision UI, the PC commissioning page, and later external adapters.

Prefer explicit update calls and read-only snapshots over a web of callbacks when timing and ownership permit it.

### 3.1.1 Testbench Simulator Architecture

A dedicated hardware-in-the-loop simulator exists behind:

```text
STRIDECONTROL_TESTBENCH
```

The simulator is a development and verification tool, not a second production architecture.

```text
TreadmillSimulatorComposite
├── VirtualTreadmill
├── VirtualSpeedSensorAdapter
├── VirtualInclineAdapter
├── VirtualConsoleAdapter
└── VirtualRunnerAdapter
```

Purpose:
- GUI development without physical treadmill hardware
- Workout workflow validation
- RunnerDynamics validation
- Hardware-in-the-loop testing on ESP32-S3

Production ownership and module responsibilities remain unchanged.

### 3.2 Commissioning and Module Independence

Every major module shall be independently observable and commissionable.

The PC Commissioning interface is an architectural subsystem, not a late-stage testing utility.

Each module should expose:

- configuration
- diagnostics
- health
- counters
- authoritative state
- commissioning actions

without requiring the full system stack to be active.

### 3.3 State and Responsibility Ownership

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

### 3.4 Speed Measurement and Command Calibration Ownership

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
| E-Stop Input | 18 | Active-low hardware line. Polled atomically by ConsoleInterface::isEmergencyStopActive() |

#### 4.3.1 Hardware Profile and Conflict Resolution

The integrated GPIO allocation resolves historical prototype pin sharing:

- `Speed+ Relay` is mapped to **GPIO 39** (freeing GPIO 14 for `Incline Sensor`).
- `CSAFE TX` is mapped to **GPIO 17** (freeing GPIO 16 for `Buzzer ACK`).
- `Speed- Relay` is mapped to **GPIO 38** (freeing GPIO 47 for `I2C SDA`).

Any hardware profile using historical prototype assignments is incompatible with the integrated configuration and must not be enabled concurrently.

#### 4.3.2 Emergency Stop Line (GPIO 18)

The physical safety lanyard switch is directly monitored via GPIO 18 (active-low with internal pull-up). When dislodged:

- `ConsoleInterface` asserts `isEmergencyStopActive() = true`.
- Control runtimes suppress double-stop counting and prevent accidental session finalization.
- Telemetry broadcasts root `estopActive: true` and `session.isEmergencyStopped: true`, displaying an immediate recovery modal across all web views.
- On tether reset and physical restart, Interval Mode re-issues current step targets (since treadmill volatile memory is cleared by E-stop), preserving workout progress and step timing.

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

Speed commands are planned through two execution paths:

1. **Relay Fast-Path (|Δ| ≤ 0.5 km/h):**
   When the belt is already in motion and the requested speed adjustment is within ±0.5 km/h of the current command, O2 digit entry is bypassed entirely. The target is reached directly through sequential Speed+ or Speed- relay pulses (80 ms hold, 80 ms pause), eliminating console screen flashing and numeric re-entry latency.

2. **Full Numeric Injection (|Δ| > 0.5 km/h):**
   The nearest integer base is entered via O2 Instant Speed, numeric keys, and Enter under continuous baseline emulation. Fractional tenths are subsequently trimmed using Speed+/- relays after controlled bus release and settling:
   - 12.6 km/h: Base 13 entered via O2, followed by four Speed- relay pulses.
   - 12.4 km/h: Base 12 entered via O2, followed by four Speed+ relay pulses.

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

The product goal is that measured belt movement without qualified runner footstrikes eventually results in no runner credit. FTMS and RSC speed is then forced to zero and validated runner distance pauses without changing treadmill control state.

The detailed grace periods, state transitions, regularity requirements, SideRails qualification, and resume behavior belong to `RunnerDynamics`. Higher layers must not replace this state machine with a direct speed/cadence condition.

Current evidence:

```text
VERIFIED BY EXECUTED ESP32 TEST
VERIFIED BY EXECUTED COMPOSITE SIMULATION
```

Verified scenarios include:
- RunningOnBelt
- OnSideRails
- NotPresent
- SideRails grace-period behavior
- Resume from SideRails
- Cadence qualification
- IMU signal-loss handling
- Runner-distance gating
- Deterministic replay

---

## 6. Speed Sensor

### 6.1 Speed Control and Observation Principle

StrideControl commands the desired treadmill speed but does not directly control motor power, motor torque, acceleration, deceleration, or internal motor-control algorithms.

The treadmill mainboard remains solely responsible for:

- Motor control and power delivery
- Speed ramp-up and ramp-down behavior
- Acceleration and deceleration dynamics
- Closed-loop motor regulation
- Final physical belt speed response

StrideControl does not regulate motor torque and does not attempt to implement a motor-speed control loop.

The responsibility of StrideControl is strictly limited to:

- Validating requested speeds
- Applying approved SpeedCalibration correction
- Submitting speed commands through the console interface
- Measuring physical belt speed
- Detecting mismatch, drift, timeout, or sensor faults
- Reporting authoritative physical speed

```text
Treadmill Mainboard
= owns motor actuation and speed dynamics

TreadmillController
= submits validated target speed requests

SpeedCalibration
= predicts the command most likely to achieve a desired physical speed

SpeedSensor
= authoritative owner of measured physical belt speed

RunnerDynamics
= authoritative owner of runner-qualified speed

MaintenanceService
= authoritative owner of mechanical distance

WorkoutSession
= consumes speed information but does not own speed
```

FTMS, RSC, GUI, APIs, and commissioning interfaces report measured and validated speed state. They do not imply direct motor regulation by the ESP32.

### 6.2 Speed Authority Rule

Requested speed, commanded speed, and measured physical speed are distinct values.

```text
Requested Speed
      ≠
Commanded Treadmill Value
      ≠
Measured Physical Belt Speed
```

The authoritative source of actual treadmill speed is `SpeedSensor`. Higher-level modules must not substitute requested speed or command values for measured physical speed.

### 6.3 PC817 Interface

A PC817 collector uses a 10 kOhm pull-up to 3.3 V. Speed uses a 10 kOhm LED-side resistor at approximately 1.1 mA. Incline uses 1 kOhm at approximately 3.8 mA.

### 6.4 Software Filter

Timestamp filtering uses approximately 300 microseconds micro-glitch rejection and 2000 microseconds lockout. No blocking ISR delay is used for speed.

### 6.5 Expected Signal

Approximately 2 Hz at 1 km/h, 9 Hz at 10 km/h, and 22 Hz at 25 km/h, with one accepted falling edge per rotation.

### 6.6 Continuity and Faults

A missing pulse timeout forces speed to zero. RUNNING with no valid pulses for more than 2 seconds logs a mismatch fault.

### 6.7 Effective Zero Versus Belt Moving

Keep these concepts separate:

- Effective zero-speed epsilon: approximately 0.01 km/h
- Behavioral belt-moving threshold: approximately 0.5 km/h

Only effective zero with an operational stopped/ready status may be exempted from pulse-age freshness. A low non-zero speed still requires a fresh pulse-derived measurement.

---

## 7. Incline Sensor

### 7.1 Incline Control and Observation Principle

StrideControl commands only the desired whole-percent incline target.

The treadmill mainboard remains solely responsible for:

- Incline motor control and power
- Incline acceleration and deceleration
- Incline movement speed profile
- Final physical positioning behavior

StrideControl does not regulate incline movement rate and does not attempt to control how fast the treadmill reaches a requested incline.

The responsibility of StrideControl is strictly limited to:

- Requesting a target incline through the console interface
- Tracking authoritative incline position from homing and pulse integration (`InclineSensor`)
- Verifying actual incline position independently (`InclineVerifier` / IMU)
- Detecting movement timeout, mismatch, drift, or sensor faults

```text
Treadmill Mainboard
= owns incline motor actuation and movement dynamics

TreadmillController
= submits target whole-percent incline requests

InclineSensor
= authoritative owner of tracked actual incline position

InclineVerifier
= observational angle verification and drift detection (no control authority)
```

FTMS inclination and ramp angle exports report tracked and verified current state; they do not imply active trajectory or incline-rate control by the ESP32.

### 7.2 Incline Authority Rule

Requested incline and tracked physical incline are distinct values.

```text
Requested Incline
      ≠
Tracked Incline Position
```

The authoritative source of actual incline position is `InclineSensor`. Higher-level modules must not substitute requested incline targets for tracked physical incline position.

### 7.3 Pulse Tracking and Hardware Interface

Pin 11 emits approximately 394 Hz during movement. Incline is calculated from homing and direction-specific pulse integration, with optional IMU verification after movement.

The isolated concept uses PC817, 1 kOhm LED resistance, and 10 kOhm pull-up. BSS138 remains a legacy troubleshooting option.

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

Implemented as `SpeedLearningTracker`, fully automatic (no manual approval step - a deliberate
deviation from an earlier assumption; see 14.6).

Example:

```text
User request:                15.0 km/h
Previous treadmill command: 15.0 km/h
Stable measured result:     14.8 km/h
Future corrected command:   approximately 15.2 km/h
```

The correction is applied only the next time that speed is requested. The active command is not
continuously adjusted while the belt is running.

The table is NOT limited to fixed speed buckets - it grows dynamically at whatever exact speeds
the runner actually holds, capped at 10 concurrent tracked candidates (matching
`kMaxSpeedCalibrationPoints`) with lowest-confidence eviction beyond that cap.

A single observation window requires: CSAFE genuinely `InUse`, the commanded speed unchanged for
>= 8 seconds (settling), a subsequent 30-second stable-speed window (peak-to-peak <= 0.15 km/h),
no E-stop/pending command/countdown/ramp-test active. Evidence accumulates via Welford's algorithm
ACROSS THE WHOLE SESSION, not as discrete counted windows - any 30-second stable period counts,
wherever/whenever it occurs, with no requirement that windows be consecutive. A 3-tap median
pre-filter protects the running variance calculation from a single-tick transient (a stumble, a
momentary sensor spike) within an otherwise-valid window.

At session end (belt genuinely stopped, 3-second debounce), an accumulated estimate is only
committed if confidence >= 75% (requiring >= 45s of accumulated steady time with sigma <= 0.08
km/h). Two independent checks then gate acceptance:

- Cross-point consistency: the candidate must agree with what neighboring calibrated points
  already predict via linear interpolation (tolerance 0.4 km/h, slope bounded to [0.75, 1.35]).
  Outside the table's known range, prediction uses linear slope continuation from the outermost
  segment (not flat clamping), reflecting the non-zero intercept of real motor/inverter transfer
  behavior - see 8.4.
- Absolute factory-baseline safety guardrail: checked against the immutable, compile-time factory
  ROM baseline curve (never the mutable, already-adjusted SpeedConfig), rejecting any drift beyond
  ±15% / ±2.0 km/h from true physical limits. This specifically prevents multi-session "creeping"
  drift, where a series of individually-small adjustments (each measured only against the
  previous, already-drifted value) could otherwise walk the calibration arbitrarily far from
  reality.

Accepted corrections are applied via confidence-adaptive EWMA (higher existing confidence/history
reduces the weight of one new observation), not a flat clamp - bounding the maximum change any
single session can make to an established point while still allowing genuine, gradual drift to be
tracked over time.

A "dirty" flag gates NVS writes: a session's accumulated change is only persisted if it moved an
existing point meaningfully (beyond a negligible-noise threshold); a session that only absorbed
microscopic noise triggers zero flash writes. A genuinely new candidate is only inserted if it
would meaningfully improve interpolation accuracy at that speed - skipped if the current table
already predicts it within tolerance, avoiding a wasted table slot and an unnecessary write.

All tracker state uses single-precision `float` only (no `double`), for ESP32-S3 hardware FPU
compatibility and RAM economy (~432 bytes static, zero heap allocation).

### 8.4 Correction Table and Interpolation

Each calibration point distinguishes:

- desired physical speed
- treadmill command
- measured physical speed
- source, automatic or external commissioning
- validity
- confidence

Interpolate the required command linearly between surrounding validated points.

A bounded extrapolation policy is implemented. Outside the validated range, prediction uses linear slope continuation from the outermost known segment, bounded by strict monotonicity and the absolute factory-baseline safety guardrail - never unbounded silent extrapolation.

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
- FTMS and RSC capability and target ranges
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

#### Incline Command Resolution

Current simulator and console command behavior is verified as:

```text
ButtonId::InclinePlus
ButtonId::InclineMinus
```
= ±0.5% incline resolution

The command path uses 0.5% increments clamped to: `0.0% -> 15.0%`.

Observed incline telemetry may retain decimals derived from physical tracking and pulse integration. The simulator must mirror production command resolution exactly and must not introduce synthetic 0.1% incline controls.

---

## 9. Hardware Interface Choices

### 9.1 MCU
ESP32-S3 N16R8, dual-core.

### 9.2 Console MitM Architecture & Level Shifting (2× TXS0108E, 2× CD4066B)
The console Man-in-the-Middle (MitM) interface provides fail-passive pass-through and active O2 injection across the 5V treadmill panel and 3.3V ESP32 domains.

- **Latch-up & Boot Protection (R1 & OE):**
  - Two TXS0108E level shifters (IC1 for O1 scanning + MUTE, IC2 for O2 injection).
  - ESP32 GPIO 2 drives both `OE` pins.
  - A 10 kΩ pull-down resistor (**R1**) to GND holds `OE` LOW while the ESP32 is unpowered or booting, keeping all level-shifter lines in high-impedance (High-Z) mode to prevent silicon latch-up when the 5V console rail energizes first.
- **Fail-Safe MUTE Gate (R2 & 2× CD4066B):**
  - Two quad bilateral analog switches: **IC3** (O2 bits 0–3) and **IC4** (O2 bits 4–7).
  - ESP32 GPIO 21 drives MUTE via IC1 channel A8 $\rightarrow$ B8.
  - A 10 kΩ pull-up resistor (**R2**) to 5V holds the MUTE control line HIGH during unpowered or boot states, forcing all 8 analog switches CLOSED. The original console retains 100% normal, uninterrupted operation.
  - Driving GPIO 21 LOW actively isolates the console panel, enabling atomic O2 injection downstream from IC2 towards the motor controller.

### 9.3 Speed Adjustment Relays (Speed+ & Speed−)
Fractional speed trimming uses independent active-low switching outputs, physically isolated from the numeric O2 bus:
- **Speed+ Output:** Driven via **GPIO 39** (active low, 80 ms hold / 80 ms pause).
- **Speed− Output:** Driven via **GPIO 38** (active low, 80 ms hold / 80 ms pause).
- Relays actuate only after numeric O2 macro confirmation and a 150 ms bus-settling period.

### 9.4 Sensor & Buzzer Isolation (3× PC817)
- **U1 (Speed Sensor):** Treadmill cable pin 7 (11.4 V idle) through $R_1 = 10\text{ k}\Omega$ into U1 pin 1 (Anode). Pins 2 (Cathode) and 3 (Emitter) bridged directly to System 0V. Pin 4 (Collector) pulled up via $R_5 = 10\text{ k}\Omega$ to +3.3V $\rightarrow$ **GPIO 3** (active-low pulse train).
- **U2 (Incline Sensor):** Treadmill cable pin 11 (4.68 V pulse) through $R_2 = 1\text{ k}\Omega$ into U2 pin 1 (Anode). Pins 2 and 3 bridged to System 0V. Pin 4 (Collector) pulled up via $R_6 = 10\text{ k}\Omega$ to +3.3V $\rightarrow$ **GPIO 14** (active-low pulse train).
- **U3 (Buzzer Detection):** Treadmill buzzer (+) through $R_3 + R_4 = 440\text{ }\Omega$ ($2 \times 220\text{ }\Omega$) into U3 pin 1 (Anode). Treadmill buzzer (−) return connects directly to U3 pin 2 (Cathode) and is kept fully floating from System 0V to accommodate low-side switching. Pin 3 (Emitter) tied to System 0V. Pin 4 (Collector) pulled up via $R_7 = 10\text{ k}\Omega$ to +3.3V $\rightarrow$ **GPIO 16** (active-low ACK capture).

### 9.5 CSAFE RS232 Subsystem (SP3232)
- Transceiver: 5V SP3232 module providing galvanic isolation at 9600 baud, 8-N-1.
- **TX Path:** ESP32 GPIO 17 $\rightarrow$ **X2:12** $\rightarrow$ SP3232 TTL RXD $\rightarrow$ SP3232 TXD pad $\rightarrow$ **X1:3** $\rightarrow$ Treadmill CSAFE RX.
- **RX Path:** Treadmill CSAFE TX $\rightarrow$ **X1:4** $\rightarrow$ SP3232 RXD pad $\rightarrow$ SP3232 TTL TXD (5V). Scaled to 3.3V via voltage divider $R_8 = 1\text{ k}\Omega$ and $R_9 = 2\text{ k}\Omega$ to GND $\rightarrow$ **X2:11** $\rightarrow$ ESP32 GPIO 40.
- **Galvanic Grounding:** RS232 signal ground on **X1:2** is strictly isolated from System 0V. The Cat6 cable shield terminates at **X1:2** on the interface PCB and **must be cut and insulated at the treadmill RJ45 connector** to prevent ground loops.

### 9.6 I²C Bus & IMU Accelerator (LTC4311)
- LTC4311 active terminator breakout connects in parallel across the I²C bus to compensate for cable capacitance over the Cat6 run to the deck-mounted LSM6DSOX IMU.
- `VIN` and `EN` pins bridged directly together and tied to +3.3V (**X2:7**) for continuous operation.
- **SDA Line:** **X1:11** $\leftrightarrow$ LTC4311 SDA $\leftrightarrow$ **X2:2** (GPIO 47).
- **SCL Line:** **X1:10** $\leftrightarrow$ LTC4311 SCL $\leftrightarrow$ **X2:3** (GPIO 48).
- Unused Cat6 conductor pairs are grouped and tied to System 0V (**X2:8**) as a reference shield.

### 9.7 Power Distribution & Ground Domains
- **+5V Rail:** External USB 5V (Red) enters **X1:12**, decoupled by $C_1 = 100\ \mu\text{F}$ electrolytic capacitor, powers SP3232 (5V0), IC1/IC2 (VCCB), IC3/IC4 (VDD), and routes via **X2:9** to ESP32 `5V/VIN`.
- **+3.3V Rail:** ESP32 internal LDO supplies **X2:7**, powering pull-ups $R_5$–$R_7$, LTC4311 (`VIN` + `EN`), IC1/IC2 (VCCA), and remote IMU via **X1:8**.
- **System 0V (Brown):** Unified ground on **X2:8** tying ESP32 GND, USB 0V (**X1:9**), $C_1(-)$, treadmill sensor GND pin 2 (**X1:6**), U1/U2/U3 emitter returns, and LTC4311 GND.
- **Isolated RS232 GND (Brown-White):** Dedicated exclusively to **X1:2** and SP3232 RS232 ground.

### 9.8 Interface Board Terminal Pinout (X1 & X2 Headers)

#### X1 Header (Inputs / Treadmill / External Interfaces)
| Pin | Function | Level | Connection |
|:---:|:---|:---|:---|
| **X1:1** | Buzzer (+) | Pulsed DC | Treadmill buzzer (+) via $R_3/R_4$ to U3:1 |
| **X1:2** | RS232 GND | Isolated | CSAFE isolated ground & Cat6 shield |
| **X1:3** | RS232 TXD | $\pm$5V–$\pm$12V | SP3232 TXD to treadmill CSAFE RX |
| **X1:4** | RS232 RXD | $\pm$5V–$\pm$12V | Treadmill CSAFE TX to SP3232 RXD |
| **X1:5** | Speed In | 11.4 V idle | Treadmill pin 7 via $R_1$ to U1:1 |
| **X1:6** | Sensor GND | 0V | Treadmill pin 2 to System 0V |
| **X1:7** | Incline In | 4.68 V pulse | Treadmill pin 11 via $R_2$ to U2:1 |
| **X1:8** | +3.3V Out | +3.3V DC | IMU power feed via Cat6 |
| **X1:9** | USB 0V In | 0V | External USB Black wire to $C_1(-)$ & System 0V |
| **X1:10** | I²C SCL | 3.3V logic | Cat6 Green wire to LTC4311 SCL |
| **X1:11** | I²C SDA | 3.3V logic | Cat6 Orange wire to LTC4311 SDA |
| **X1:12** | +5V In | +5.0V DC | External USB Red wire to $C_1(+)$ & SP3232 5V0 |

#### X2 Header (ESP32 Microcontroller & Power Routing)
| Pin | Function | Level | Connection |
|:---:|:---|:---|:---|
| **X2:1** | Buzzer (−) | Floating | Treadmill buzzer (−) directly to U3:2 (*isolated from 0V*) |
| **X2:2** | I²C SDA | 3.3V CMOS | **GPIO 47** |
| **X2:3** | I²C SCL | 3.3V CMOS | **GPIO 48** |
| **X2:4** | Buzzer Out | 3.3V active-low | **GPIO 16** (from U3:4 / $R_7$) |
| **X2:5** | Incline Out | 3.3V active-low | **GPIO 14** (from U2:4 / $R_6$) |
| **X2:6** | Speed Out | 3.3V active-low | **GPIO 3** (from U1:4 / $R_5$) |
| **X2:7** | +3.3V In | +3.3V DC | From ESP32 3V3 pin |
| **X2:8** | 0V / GND | 0V | From ESP32 GND pin (System 0V anchor) |
| **X2:9** | +5V Out | +5.0V DC | To ESP32 5V/VIN pin (from $C_1+$) |
| **X2:10** | SP3232 GND | 0V | Bridged internally to X2:8 |
| **X2:11** | CSAFE RXD | 3.3V UART | **GPIO 40** (from $R_8/R_9$ divider) |
| **X2:12** | CSAFE TXD | 3.3V UART | **GPIO 17** (to SP3232 TTL RXD) |

### 9.9 Future Improvements
- Replace Speed+/− relays with a deterministic solid-state solution (e.g., PhotoMOS or opto-isolated open-drain FETs).
- Evaluate future active Incline+/− outputs with hardware interlocks.
- Evaluate replacing bidirectional level shifters (TXS0108E) with unidirectional buffers/Schmitt triggers for increased noise immunity.
- Consolidate all subsystems onto a single integrated production PCB with continuous ground plane and optimized routing.
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

The orchestrator also supports deterministic `ExternalStep` execution for hardware-in-the-loop simulation.

Simulation ownership:
```text
SimulationTick
      ↓
TreadmillSimulatorComposite
      ↓
ApplicationOrchestrator::step(...)
      ↓
ApplicationSnapshot
```
The simulator never bypasses `ApplicationOrchestrator` ownership of snapshot publication.

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

`ApplicationSnapshot` is the authoritative telemetry source for:

- `RscService`
- `FtmsService`
- `WebServerManager`
- Commissioning interfaces
- External adapters

ApplicationSnapshot is a telemetry DTO.

ApplicationSnapshot shall not become a GUI aggregation object.

UI-specific view models may aggregate ApplicationSnapshot together with WorkoutSessionSnapshot, TreadmillControllerSnapshot, and configuration DTOs without transferring ownership.

ApplicationSnapshot shall not contain workout-definition, interval-library, timeline, editor, UI navigation, modal state, or screen-specific presentation state. These belong to dedicated workout and UI view-model DTOs.

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

Safe application workflow:

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
├── RscService
└── FtmsService
```

`BleManager` owns:

- NimBLEDevice lifecycle
- NimBLEScan lifecycle
- NimBLEServer lifecycle
- GAP advertising
- Central / Peripheral concurrency
- connection event routing
- service attachment lifecycle
- BLE health and diagnostics

`HeartRateClient` provides heart rate, validity, data age, connection state, selected sensor, and available battery status.

V1 `FtmsService` telemetry may be implemented before `WorkoutSession`. Control-oriented FTMS functionality, elapsed time exposure, supported ranges, and future control-point functionality require stable `WorkoutSession`, `SystemManager`, `SpeedCalibration`, incline, and command contracts.

### 10.11 Workout and Maintenance

WorkoutSession owns:

- workout lifecycle
- workout suspension and resume
- active workout execution
- interval progression
- phase transitions
- cut-drag handling
- rest extension requests
- remaining-workout speed adjustment
- workout completion state
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

#### 10.11.1 Workout Suspension and Resume

Stopping treadmill motion during an active workout does not terminate the workout automatically.

The workout enters `Suspended` state.

WorkoutSession stores:

- current step
- elapsed step time
- completed fraction
- pause timestamp
- workout progress context

When treadmill motion is later detected again, WorkoutSession evaluates the pause duration and workout state.

It may recommend:

- Resume Workout
- Resume With Re-Warmup
- Restart Current Interval
- Skip To Recovery
- End Workout

The final decision belongs to the user. WorkoutSession must never automatically resume directly into a work interval.

#### 10.11.2 Re-Warmup Recommendation

WorkoutSession may generate resume recommendations using configurable `SettingsService` thresholds.

Configurable parameters in `WorkoutResumeSettings`:

- `immediateResumeThresholdSec`
- `reEntryRecommendationThresholdSec`
- `warmupRecommendationThresholdSec`
- `extendedWarmupThresholdSec`
- `skipToRecoveryRemainingFraction`

Typical operational examples:

- `pause < immediateResumeThresholdSec`: immediate resume may be recommended
- `pause > reEntryRecommendationThresholdSec`: short re-entry may be recommended
- `pause > warmupRecommendationThresholdSec`: re-warmup may be recommended
- `pause > extendedWarmupThresholdSec`: restart of interval or extended warmup may be recommended

These recommendations are advisory only. WorkoutSession must not automatically execute them.

#### 10.11.3 Workout Resume Policy

##### Purpose
The Resume Policy defines how StrideControl coordinates workout session context when belt movement stops and subsequently resumes. Because StrideControl has no active start or stop outputs, starting and stopping the physical belt is exclusively performed by the user on the treadmill console.

##### Ownership
`WorkoutSession` owns workout pause/resume lifecycle and recommendation state.

`TreadmillController` owns target speed and incline command validation and transmission.

`SpeedSensor` is the authoritative owner of measured physical belt speed.

`InclineSensor` is the authoritative owner of tracked incline position.

##### Passive Pause Behavior
When the physical belt stops (via user pressing physical Stop, E-stop, or console timeout):

1. `SpeedSensor` detects zero belt speed.
2. `WorkoutSession` transitions: `WorkoutState = Suspended`.
3. Stored workout context is frozen:
   - Elapsed workout and interval time
   - Validated runner distance
   - Stored target speed and target incline
   - Interval progression
4. `TreadmillController` clears active injected targets.

##### Resumption Sequence
Because StrideControl cannot initiate belt movement from standstill, resumption is gated entirely on physical treadmill activity:

1. The user manually starts the treadmill using the physical console (e.g., Quick Start).
2. `SpeedSensor` detects confirmed physical belt movement above the moving threshold (> 0.5 km/h).
3. `WorkoutSession` transitions directly: `Suspended` → `Running`.
4. Stored target incline is re-issued to `TreadmillController`.
5. Stored target speed is re-issued to `TreadmillController` via standard `SpeedCalibration`.

No GUI confirmation tap or resume countdown delay is required after a normal pause. Extended interruptions exceeding configured re-warmup thresholds (Section 10.11.2) fall under advisory coach recommendations.

##### Safety and Abort Rules
- If belt movement stops again, `WorkoutSession` immediately returns to `Suspended`.
- Target speed is never dispatched to `TreadmillController` while the belt is at a standstill.

##### Configuration
Managed by `SettingsService`:
- `autoResumeOnPhysicalStart`: Default `true` (Seamless resume when physical belt movement is re-detected after a standard pause)
- `resumeDelaySec`: Default `0` s (Applies only if a staged re-entry delay is configured by policy)

#### 10.11.4 WorkoutDefinition

WorkoutDefinition owns:

- workout name
- workout structure
- warmup
- cooldown
- interval groups
- repetitions
- drag steps
- rest steps
- progression configuration

WorkoutDefinition is persisted through SettingsService.

Warmup is always the first item. Cooldown is always the last item.

Warmup and cooldown may be edited but shall not be deleted, duplicated or moved.

#### 10.11.5 WorkoutExpander

WorkoutExpander is a pure-logic component.

Responsibilities:

- expand interval groups into executable sequence
- apply repetition counts
- perform final-sequence rest stripping
- resolve inherited rest speed
- generate executable timeline model

#### 10.11.6 Runtime Safety and Interlock Rules

- **QuickStart Motion Interlock:** Physical or simulated QuickStart commands are rejected if the belt is already in motion (`isBeltMoving() == true`). The console requires complete physical standstill before accepting a new start sequence.
- **Free-Run Auto-Start:** When belt motion is detected while in manual mode, `WorkoutSession` automatically starts a new free-run session from `Idle`, `Completed`, or `Aborted` states without requiring manual reset.
- **Atomic Workout Deletion:** `POST /api/v1/settings/workout/delete` safely removes a specified workout from LittleFS storage without corrupting adjacent user profiles or resetting firmware-owned `lastUsedTimestamp` records.

### 10.12 WebServerManager

WebServerManager owns:

- HTTP routing and LittleFS static asset delivery
- Telemetry serialization and polling (`/api/v1/telemetry`)
- Motion command dispatch (`/api/v1/control/*`)
- Settings and workout persistence (`/api/v1/settings/*`, `/api/settings`)
- Session history ingestion and retrieval (`/api/v1/history`)

WebServerManager does not own UI logic, timeline generation, interval expansion, navigation state or workout execution logic.

WebServerManager consumes:

- `ApplicationSnapshot`
- `TreadmillControllerSnapshot`
- `WorkoutSessionSnapshot`
- `SettingsService` snapshots

WebServerManager does not own:

- speed
- incline
- distance
- heart rate
- runner state
- workout state
- treadmill control

#### 10.12.1 Web Interface Topology

| Route | Source File | Target Display | Primary Responsibilities |
|---|---|---|---|
| `/` | `index.html` | Phone, Tablet, PC | Responsive session companion: profile switching, drag-and-drop interval builder, workout copy/delete, history telemetry charts, PNG summary card export. |
| `/tablet.html` | `tablet.html` | Console-mounted tablet | Dedicated running console: real-time telemetry, 8+8 quick keys, live interval timeline, work-phase focus dimming, ramp cancellation. |
| `/service.html` | `service.html` | Desktop browser | Hardware commissioning: speed sensor factor, dead time, IMU zero offset, piecewise `SpeedConfig` calibration table, maintenance counters. |
| `/simulator.html` | `simulator.html` | Developer testbench | Virtual T610 stimulus: belt mechanics, runner footstrike dynamics, physical button inputs (`STRIDECONTROL_TESTBENCH` only). |

#### 10.12.2 Navigation and Tool Isolation Rule

- **Developer & Service Pages:** `simulator.html` and `service.html` provide top-bar navigation links to each other, to `/` (`index.html`), and to `/tablet.html`.
- **End-User Interfaces:** `index.html` and `tablet.html` contain **zero outbound links** to service or simulator interfaces. User-facing interfaces remain strictly isolated from diagnostic and developer utilities.

### 10.13 AI-Assisted Development Protocol

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

## 13. Bluetooth FTMS and RSC Rules

- NimBLE-Arduino
- Dual-role HR client with FTMS and RSC peripheral servers
- FTMS and RSC telemetry cadences are configurable by product policy.
- Initial V1 target cadences:
  - FTMS: 4 Hz (250 ms)
  - RSC: 4 Hz (250 ms)
- Speed in 0.01 km/h units (FTMS) and 1/256 m/s units (RSC)
- Incline in 0.1% units
- Elevation Gain is omitted until an authoritative accumulator exists.
- Elapsed Time is omitted until `WorkoutSession` provides an authoritative elapsed-time source.
- Heart Rate is included only when `heartRateValid` is true.
- Optional FTMS/RSC fields are advertised only when supported by an authoritative source.
- HR MAC stored through SettingsService/NVS ownership
- All BLE work on Core 0
- Advertised maximum and target range use calibrated maximum achievable speed

### 13.1 Authoritative Ownership for FTMS and RSC Exports

```text
RSC:
- speed                  -> RunnerDynamics (authoritative runner-qualified speed owner)
- cadence                -> RunnerDynamics (authoritative cadence owner)
- walking/running state  -> RunnerDynamics (authoritative motion classification owner)
- validated distance     -> RunnerDynamics (authoritative runner distance owner)

FTMS:
- speed                  -> RunnerDynamics (authoritative runner-qualified speed owner)
- incline                -> InclineSensor (authoritative incline owner)
- ramp angle             -> InclineSensor-derived incline state (FTMS representation)
- distance               -> RunnerDynamics (authoritative runner distance owner)
- heart rate             -> HeartRateClient (authoritative heart rate owner)
- elapsed time           -> WorkoutSession (future authoritative elapsed-time source)
- elevation gain         -> authoritative accumulator (future)
```

---

## 14. GUI Architecture and Interval Coach

### 14.0 Two-Tier GUI Architecture

StrideControl separates workout configuration and post-run analysis from in-run execution into two decoupled web applications:

1. **Session Manager (`index.html`):**
   - Companion interface accessible from phone, tablet, or desktop browser at `/`.
   - Allows switching between existing runner profiles, copying workouts between runners, and building structured interval sessions (warmup, repeating work/rest groups, progressive speed delta, cooldown).
   - Provides post-run history review with speed and heart-rate telemetry charts, summary statistics, and PNG card export.
   - Preserves the active tab (e.g., History) when switching between runner profiles.

2. **Tablet Running Console (`tablet.html`):**
   - In-run console overlay designed for the tablet mounted over the treadmill console at `/tablet.html`.
   - Focuses strictly on live workout execution: high-contrast telemetry readouts, 8+8 speed/incline quick-key arrays, live interval timeline with countdown, auto-prefire alerts, and ramp cancellation.
   - Visual layout, interaction patterns, workflow, terminology and look-and-feel follow the Sportsmaster/COROS styling.

The simulator page is not a UX reference.
`simulator.html` exists solely for:
- physical T610 console emulation
- runner presence stimulation
- diagnostics

All workout UX, presets, interval workflows, and training interactions remain owned by the web presentation tier.

The ESP32 firmware is the authoritative owner of application logic. The GUI is a presentation layer consuming authoritative state from firmware APIs.

### 14.1 Precision UI Principles and Safety Interlocks

The tablet interface is a dedicated web frontend consuming authoritative state from `WebServerManager`. All user actions in the GUI represent dispatch of target orders (speed, incline, mode switch) rather than direct hardware manipulation.

- **Global Activity Interlock:** As soon as physical belt speed is detected (> 0.0 km/h), the entire top navigation bar dims to 20% opacity and pointer events are disabled (`pointer-events: none`). Navigation, profile switching, and configuration modals are physically inaccessible while the belt is moving. The top bar is re-enabled only at complete standstill (0.0 km/h).
- **Dynamic Telemetry Layout:**
  - *Climbed Elevation (m):* Hidden while incline is 0.0%. Total Time, Distance, Heart Rate, and Speed share 25% width each. When incline reaches ≥ 1.0%, the layout compresses and the elevation column slides into the center.
  - *Achromatic Critical Telemetry:* Heart rate and speed numeric readouts remain strictly white/monochrome. Color accents (blue/red) are reserved exclusively for action buttons and progress indicators to avoid visual alarms during exertion.
- **Operational Modes:**
  - *Manuell Modus:* 8+8 quick-key grid with HVILE and DRAG preset order buttons.
  - *Intervall Modus:* Manual grids are hidden in favor of timeline, hero remaining time, phase banner, and contextual action buttons. Focus Mode engages automatically during work intervals, dimming non-essential chrome to ~8–10% opacity.

### 14.2 Top Bar Navigation

Positioned at the very top in Intervall mode and below the incline keys in Manuell mode (accessible only at standstill):

1. **System Clock:** Local RTC time (`HH:MM`). Non-interactive.
2. **User Profile Selector (`KRISTIAN ▾`):** Displays active profile. Tapping returns to Screen 8 to switch user, ensuring correct sensor routing and workout history.
3. **Mode Toggle (`MANUELL` / `INTERVALL`):** Toggles between free running and structured workout modes.
4. **Settings Action (`[ OPPSETT ]`):** Opens Screen 10 (Main Configuration).

### 14.3 Screen State Machine and Operational Flows

#### 14.3.1 Screen 8: Initial Launcher & Profile Selection
Engaged when waking the tablet or when switching profiles at standstill:
- **User Cards (`CHRISTINE`, `JONAS`, `JULIE`, `KRISTIAN`):** Large interactive buttons. Selecting a profile immediately loads the runner's presets, quick-key arrays, and Bluetooth device bindings.
- **Mode Buttons (`MANUELL`, `INTERVALL`):** Tapping either mode switches directly to the corresponding operational view and dismisses the launcher.

#### 14.3.2 Screens 1 & 2: Manual Control Mode
- **Incline Quick Keys (0, 1, 2, 4, 6, 8, 10, 12 %):** Top-anchored buttons with blue bottom border. Tapping dispatches a target whole-percent incline order to `TreadmillController`.
- **Speed Quick Keys (4, 6, 8, 10, 12, 14, 16, 18 km/h):** Bottom-anchored buttons with red bottom border. Tapping dispatches a target speed order to `TreadmillController`.
- **`[ HVILE ]` Preset Button (Blue accent):** Centered above the speed row. Dispatches the profile's configured recovery speed target (e.g., 6.0 km/h).
- **`[ DRAG ]` Preset Button (Red accent):** Centered above the speed row. Dispatches the profile's configured work speed target (e.g., 16.0 km/h).

#### 14.3.3 Screens 3–7: Interval Mode Workflow
- **Screen 3: Program Selection (Standstill):**
  - Bottom program carousel surfaces stored workouts from the active profile (supporting up to 16 defined workouts per user).
  - Tapping previews the timeline and phase breakdown. The selected program displays a blue bottom border.
  - Starts when the runner physically presses `QUICK START` on the treadmill console, transitioning to Screen 4.
- **Screen 4: Work Interval (Drag):**
  - Live countdown timer, phase banner, and active segment timeline.
  - **`[ KUTT DRAG ]` (Blue accent):** Immediate early termination of a work rep. Logs the repetition as partially completed and dispatches a target speed order for the configured recovery pace. Transitions to Screen 5A.
- **Screen 5A: Rest Interval - Step 1 (Speed+ Prompt & Toast):**
  - **Toast Banner:** Slides down from top with previous rep metrics (e.g., "DRAG 4 FULLFØRT · SNITTPULS 168") and dismisses automatically after 4 seconds.
  - **Circular Countdown Timer:** Depletes synchronously with remaining recovery time.
  - **`[ JA ]` / `[ NEI ]` (Speed Adjustment):** Tapping `JA` adds +0.2 km/h (or configured step) to all remaining work intervals in the session. Tapping `NEI` keeps the original planned speed. If countdown expires without input, the original plan is retained. Both choices advance UI to Step 2.
- **Screen 5B: Rest Interval - Step 2 (Manual Rest Override):**
  - Displayed while rest time continues to count down.
  - **`[ UTVID HVILE ]` (Blue accent):** Adds +30 seconds to the active rest countdown per tap.
  - **`[ START DRAG ]` (Red accent):** Immediately truncates remaining rest time and dispatches work interval target speed and incline orders.
- **Screen 6: Motion Interrupted (Physical Belt Stopped):**
  - Triggered automatically if the physical belt stops during an active program.
  - **`[ BYTT TIL MANUELL ]`:** Cancels the interval program, preserves accumulated workout distance and time, and switches to Screen 1.
  - **`[ AVSLUTT ØKT ]`:** Finalizes session record and advances directly to Summary (Screen 9).
  - *Physical Restart:* Starting the belt via physical console Quick Start resumes the interval session directly without GUI prompt per Section 10.11.3.
- **Screen 7: Cooldown and Completion:**
  - Engaged automatically when the final work interval ends. Hero text displays `INTERVALL FULLFØRT`. Target speed drops to cooldown pace. The workout terminates when the runner presses physical Stop on the console, triggering Screen 9.

#### 14.3.4 Screen 9: Post-Workout Summary
- Displays total time, distance, average speed, max speed, and heart-rate session metrics.
- Displays `ØKT SYNKRONISERT OG LAGRET`.
- **`[ FERDIG ]`:** Resets local workout accumulators, closes the summary modal, and returns to launcher state.

#### 14.3.5 Session History Synchronization
- Upon workout completion (duration ≥ 2 minutes), `tablet.html` compiles a session summary and dispatches it to `POST /api/v1/history`.
- Firmware retains up to 5 completed workouts per user in LittleFS (`/hist_<uid>.json`) using atomic temporary-file writes and renames.
- `index.html` fetches summaries via `GET /api/v1/history?userId=<id>` for canvas graph rendering and image export.

#### 14.3.6 Incline Ramp Cancellation
- When an upcoming step targets a new incline and triggers pre-fire (`session.rampPreFireActive == true`), the UI surfaces an "Avbryt" action button while `session.rampCancelable == true`.
- Tapping dispatches `POST /api/v1/control/workout/cancelramp`, aborting the upcoming incline shift without interrupting planned speed or step timing.

### 14.4 Configuration Modals (Screens 10–12)

Accessible only at complete standstill (`speed == 0.0 km/h`):

- **Screen 10: Main Configuration:**
  - Stepper controls (`– / +`) for active profile's `HVILE` and `DRAG` preset speeds.
  - Bluetooth LE device scanning and pairing manager (`[ Koble til ]` / `[ Koble fra ]`) for heart-rate sensors.
  - `[ LAGRE ]` persists settings to `SettingsService`; `[ AVBRYT ]` discards changes.
- **Screen 11: Session Builder (Interval Planner):**
  - Manages up to 16 custom workout definitions per profile.
  - Warmup / Cooldown toggles (`FAST` locks exact target speed; `FRITT` allows free manual speed adjustment).
  - Work duration mode toggle (`TID` vs `METER`).
  - Progressive workout steppers: Adjusting base speed or progressive delta auto-calculates final rep speed `(Base + (Reps - 1) * Delta)`.
  - Advanced structures support multi-stage repeating groups and arbitrary segment sequences.
  - `[ LAGRE ØKT ]` persists workout parameters to the active user profile via `SettingsService`.
- **Screen 12: Tablet Quick-Key Customization:**
  - Customization interface for the 8 speed and 8 incline buttons displayed on the tablet manual screen.
  - Tapping any button opens a value picker to configure that tablet quick key (e.g., setting a button to 15.5 km/h). Purely configures tablet web frontend presentation.
  - `[ LAGRE ENDRINGER ]` persists the custom quick-key array to the user profile.

### 14.5 Interval Coach Execution Rules

Autonomous target dispatch to `TreadmillController` occurs at phase boundaries (speed and incline). Visual feedback uses phase banners, timeline progress, and countdown timers. In V1, audible cues rely exclusively on the console's native buzzer feedback (Web Audio is excluded). Focus mode engages automatically during work intervals.

### 14.6 PC Settings, Diagnostics, and Commissioning UI

This is a separate PC-oriented HTML interface for:

- external speed reference calibration
- SpeedCalibration table inspection
- automatic learning audit log (read-only)
- SpeedLearningTracker commits automatically with no manual approval step; the page surfaces what was applied, for transparency only
- maximum achievable speed commissioning
- configuration validation and rollback
- raw IMU and sensor telemetry
- diagnostics and counters
- data recording and export
- incline calibration and verification

The page distinguishes defaults, stored, temporary, active, externally verified, automatically learned, and last-known-good values. It remains architecturally isolated from the tablet training UI.

### 14.7 System Settings and User Profile Schema

Persistent user configuration, custom workout libraries, and quick-key assignments are serialized atomically to LittleFS at `/config/settings.json` via `SettingsService`. Session history is decoupled from settings and persisted independently in `/hist_<uid>.json` (retaining up to 5 completed sessions per runner) to eliminate configuration bloat and minimize flash write cycles during active workouts.

#### Authoritative `SystemSettings` Schema (`/config/settings.json`)

```json
{
  "schemaVersion": 1,
  "users": [
    {
      "id": 1,
      "name": "Bruker 1",
      "hvileSpeedKmh": 6.0,
      "dragSpeedKmh": 15.0,
      "preferredHrMac": "",
      "speedQuickKeys": [6.0, 8.0, 10.0, 12.0, 14.0, 15.0, 16.0, 18.0],
      "inclineQuickKeys": [0, 1, 2, 3, 4, 5, 6, 8],
      "selectedWorkoutId": 1,
      "recentWorkoutIds": [2, 3],
      "workouts": [
        {
          "id": 1,
          "name": "3-2-1 PYRAMIDE",
          "lastUsedTimestamp": 1000,
          "segments": [
            {
              "id": 1,
              "type": "SINGLE_STEP",
              "repetitions": 1,
              "startSpeedKmh": 9.0,
              "speedProgressionPerRepKmh": 0.0,
              "steps": [
                {
                  "id": 1,
                  "role": "WARMUP",
                  "durationType": "TIME_SECONDS",
                  "durationValue": 600,
                  "speedMode": "FREE",
                  "targetSpeedKmh": 9.0,
                  "targetInclinePct": 0,
                  "setIncline": false
                }
              ]
            },
            {
              "id": 2,
              "type": "REPEATING_GROUP",
              "repetitions": 2,
              "startSpeedKmh": 15.0,
              "speedProgressionPerRepKmh": 0.0,
              "steps": [
                {
                  "id": 2,
                  "role": "WORK",
                  "durationType": "TIME_SECONDS",
                  "durationValue": 180,
                  "speedMode": "FIXED",
                  "targetSpeedKmh": 15.0,
                  "targetInclinePct": 0,
                  "setIncline": false
                },
                {
                  "id": 3,
                  "role": "REST",
                  "durationType": "TIME_SECONDS",
                  "durationValue": 90,
                  "speedMode": "FREE",
                  "targetSpeedKmh": 6.0,
                  "targetInclinePct": 0,
                  "setIncline": false
                }
              ]
            },
            {
              "id": 3,
              "type": "SINGLE_STEP",
              "repetitions": 1,
              "startSpeedKmh": 8.0,
              "speedProgressionPerRepKmh": 0.0,
              "steps": [
                {
                  "id": 7,
                  "role": "COOLDOWN",
                  "durationType": "TIME_SECONDS",
                  "durationValue": 300,
                  "speedMode": "FREE",
                  "targetSpeedKmh": 8.0,
                  "targetInclinePct": 0,
                  "setIncline": false
                }
              ]
            }
          ]
        }
      ]
    }
  ]
}
```

#### Schema Constraints and Capabilities
- **Capacity:** Supports up to 4 runner profiles (`MAX_USERS = 4`), each holding up to 16 workout definitions (`MAX_WORKOUTS_PER_USER = 16`).
- **Segment Hierarchy:** Workouts contain up to 16 segments (`MAX_SEGMENTS_PER_WORKOUT = 16`), defined as either `SINGLE_STEP` or `REPEATING_GROUP` (up to 8 steps per group).
- **Rep-Based Speed Progression:** Work steps support linear speed deltas across rep iterations (`speedProgressionPerRepKmh`), computed deterministically on step entry.
- **Incline Coupling:** Each step can optionally define and command an authoritative target incline (`targetInclinePct`, `setIncline = true`), triggering pre-fire ramp sequences ahead of step transition.

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

### 15.1.1 Simulator Verification

Verified by executed ESP32 tests:
- VirtualTreadmill
- VirtualSpeedSensorAdapter
- VirtualInclineAdapter
- VirtualConsoleAdapter
- VirtualRunnerAdapter
- TreadmillSimulatorComposite

Verified simulator workflows:
- QuickStart -> belt acceleration
- Speed target changes
- Incline target changes
- Runner OnSideRails
- Runner resume
- E-stop handling
- WorkoutSession interaction
- ApplicationSnapshot telemetry publication

### 15.2 Hardware Open Items

- [ ] Validate ambiguous Enter behavior
- [ ] End-to-end speed and incline regression
- [ ] Validate integrated GPIO allocation on final hardware
- [ ] Future solid-state Speed+/- outputs
- [ ] Future active Incline+/- outputs
- [ ] Future deterministic TXS0108E replacement
- [ ] Final PCB/wiring revision
- [ ] Production qualification

### 15.3 Software and Integration Status

- [x] Confirm a known Git baseline and commit hash
- [x] Add executable test infrastructure without permanent hooks
- [x] Execute deterministic RunnerDynamics and interface-contract tests
- [x] Implement and test the separate SpeedCalibration library (`SpeedCalibration` & `SpeedLearningTracker`)
- [x] Define and test automatic learning acceptance rules
- [ ] Implement PC-only external speed commissioning
- [ ] Implement maximum-achievable-speed calibration
- [ ] Define recorded-data format before physical capture
- [x] Implement Diagnostics core and SettingsService core (`SettingsService` with atomic LittleFS persistence)
- [x] Implement InclineVerifier without control authority
- [x] Implement BleManager core and HeartRateClient (`BleManager` & `HeartRateClient`)
- [x] ApplicationOrchestrator and ApplicationSnapshot integrated and verified in simulator pipeline
- [x] Implement SystemManager without duplicating subsystem state
- [x] WorkoutSession integration path verified in simulator environment
- [x] Implement RscService and FtmsService (`BleServices`)
- [ ] Implement recorded-data replay before aggressive tuning
- [x] Implement MaintenanceService with separate mechanical distance
- [x] Implement WorkoutDefinition persistence and structured JSON serialization
- [x] Implement WorkoutExpander
- [x] Testbench WebServer/API contracts established for simulator and telemetry integration
- [x] Serve existing Precision UI from LittleFS (`index.html`, `tablet.html`, `service.html`, `simulator.html`)
- [x] Replace simulator with authoritative firmware state
- [x] Simulator isolated from production build through STRIDECONTROL_TESTBENCH compile-time segregation
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
14. RscService
15. FtmsService
16. Recorded-data replay
17. MaintenanceService
18. WebServerManager and API model
19. PC settings, diagnostics, calibration, and commissioning page
20. Existing Precision UI integration
21. External adapters
22. SafetySupervisor only after separate safety analysis
23. Combined software testing
24. Recorded-data validation
25. Hardware-in-the-loop
26. Physical treadmill testing and tuning

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
