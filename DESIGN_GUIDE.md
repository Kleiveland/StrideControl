# StrideControl Technical Design Guide

This document contains verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system safely and predictably.

> **Primary UI Goal**
>
> StrideControl provides a clean, tablet-based web interface for direct speed and incline control. Because the tablet completely covers the original console, the UI must display **measured belt speed** and the **hardware-tracked incline estimate** derived from physical hardware feedback, not just requested target values. When IMU verification is available and calibrated, the UI may additionally identify the incline estimate as IMU-verified.

---

## 1. Project Goal

The objective is to build a safe, reversible ESP32-S3 interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:

- Hosts a local web interface for a tablet.
- Allows direct user control of speed and incline.
- Measures physical belt speed and tracks incline from homing and calibrated pulse integration.
- Parses machine state from CSAFE.
- Broadcasts telemetry through Bluetooth FTMS.
- Preserves the original console and safety path if the ESP32 loses power.
- Uses qualified buzzer feedback to verify injected commands.
- Retries only responses confidently classified as `NO_RESPONSE`.
- Restores numeric entry to a known state before restarting an ambiguous sequence.

This is a **non-destructive overlay system**, not a replacement for the main motor controller.

---

## 2. Safety & System Integrity Principles

### 2.1 Safety Design Principle

The system assumes failures may occur, including MCU crashes, communication loss, sensor failure, injection failure, missing buzzer feedback, and ambiguous or duplicate numeric registration. The original treadmill control path must remain functional.

### 2.2 Real-Time Constraint Rule

Network, UI, Bluetooth, and storage processing must not delay real-time hardware handling. Timing-critical sensor capture, O1/O2 processing, active baseline generation, buzzer-edge capture, and command injection are isolated from UI and network work.

### 2.3 Fail-Safe Path Requirement

The MitM hardware defaults to passive pass-through when unpowered. Loss of ESP32 power must not interrupt console communication, safety functions, or normal treadmill operation.

### 2.4 ISR Design Rule

ISRs are minimal and non-blocking. They may timestamp and perform minimal validation but must not log, update the UI, use networking, or perform serial I/O.

- The incline ISR may use a documented 10 microsecond validation delay in the hardware profile where that input is enabled.
- The buzzer ISR performs no blocking validation and only stores timestamped edges in a bounded ring buffer.

---

## 3. System Scope & Architecture

- The ESP32 owns UI, telemetry, FTMS, command planning, command injection, qualified buzzer ACK, and recovery.
- The treadmill mainboard retains motor power, incline power, and original safety logic.
- The web UI is stateless and consumes authoritative internal state.
- Instant Speed, Instant Incline, Num0-Num9, and Enter are actively generated with continuous O2 emulation.
- Speed+ and Speed- are actively generated through separate relay outputs.
- Physical Quick Start, Stop, Speed+/-, and Incline+/- are passively recognized from O1/O2 activity.
- No Phase D command is actively injected through O2.
- Incline targets are whole percentages only because active Incline+/- outputs are not installed.

---

## 4. Protocol & Hardware Mapping

### 4.1 Harness Measurements (Verified)

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

The old ROW_A-ROW_E model is superseded. Short intermediate O1 combinations may occur from parallel-bit skew and must not be treated as valid phases.

#### 4.2.3 Idle Response

When O1 enters `0x1F`, output O2 `0x00` for the first 1200 microseconds and `0xFF` afterward. The phase timer restarts on each new valid phase.

### 4.3 GPIO Mapping, O2 Bus, and Isolation

#### 4.3.1 ESP32-S3 GPIO Mapping

| Signal | GPIO | Function / status |
|---|---:|---|
| O1 bits 0-4 | 4, 5, 6, 7, 15 | Read-only phase bus |
| O2 bits 0-7 | 41, 42, 8, 9, 10, 11, 12, 13 | Bidirectional O2 bus |
| Shared MUTE | 21 | Active-low physical-source isolation |
| TXS0108E OE | 2 | Level-shifter output enable |
| Buzzer ACK | 16 | Current console-interface build |
| Speed+ relay | 14 | Current console-interface build |
| Speed- relay | 47 | Current console-interface build |
| Emergency-stop monitor | 18 | Reserved; disabled in current firmware |
| Speed input | 3 | Full architecture |
| Incline input | 14 | Historical assignment; conflicts with Speed+ relay |
| CSAFE RX/TX | 16 / 17 | Historical assignment; RX conflicts with buzzer |
| I2C SDA/SCL | 47 / 48 | Historical assignment; SDA conflicts with Speed- relay |

> **Integration requirement:** The final combined PCB must resolve GPIO16, GPIO14, and GPIO47 conflicts before all subsystems are enabled simultaneously.

#### 4.3.2 O2 Bus

O2 is changed atomically across both GPIO register banks. Sequential `digitalWrite()` calls are not suitable for phase-critical updates.

#### 4.3.3 Atomic Direction Change

The current implementation preloads the complete O2 byte and changes all O2 direction bits through the GPIO-enable registers across both banks. The previous sequential `pinMode()` limitation is superseded.

Ownership transition:

1. Calculate and preload baseline.
2. Assert MUTE LOW.
3. Enable all O2 outputs atomically.

Release transition:

1. Continue valid baseline through the reconnect point.
2. Return O2 to input atomically.
3. Wait 50 microseconds.
4. Release MUTE HIGH.

#### 4.3.4 Physical Panel Isolation

SN74HC4066N bilateral switches isolate the touch-console and side-panel O2 sources through one shared active-low MUTE signal. A 10 kOhm pull-up keeps the default state connected.

| MUTE | Physical sources |
|---|---|
| HIGH | Connected |
| LOW | Isolated |

The emergency-stop path is excluded.

#### 4.3.5 TXS0108E OE and Safe Startup

OE starts LOW through a 10 kOhm pull-down. Configure OE LOW, MUTE HIGH, and O1/O2 as inputs before enabling OE.

#### 4.3.6 Passive State

With injection inactive, O1 and O2 are inputs, MUTE is HIGH, OE remains enabled after safe startup, and the original panels pass through normally.

### 4.4 Verified O2 Emulation Principle

The treadmill is controlled by continuous phase-dependent O2 emulation, not by replaying frames.

#### 4.4.1 Baseline Generation

- `0x0F` -> `0x80`
- `0x17` -> `0x80`
- `0x1B` -> `0x80`
- `0x1D` -> `0xC0`
- `0x1F` -> `0x00`, then `0xFF` after 1200 microseconds

The complete baseline continues before, between, and after steps, during ACK observation, retry cooldown, and recovery. The inverted initial cache value is only a sentinel and is never driven.

#### 4.4.2 Closed-Loop Injection Sequence

1. Observe O1 with physical panels connected.
2. Obtain a valid phase transition and fresh IDLE.
3. Preload baseline.
4. Assert MUTE LOW and take O2 ownership atomically.
5. Generate complete baseline continuously.
6. Wait for a fresh target phase.
7. Generate the full active O2 byte.
8. Hold for the minimum configured time and release after leaving the target phase.
9. Return immediately to baseline.
10. Classify the qualified buzzer response.
11. Continue, retry, or recover according to classification.
12. Keep MUTE and baseline active throughout the O2 macro.
13. Send Enter only after the complete numeric sequence is confirmed.
14. Reconnect in fresh IDLE, return O2 to input, wait 50 microseconds, and release MUTE.
15. Wait before relay adjustment or passive monitoring resumes.

### 4.5 Verified Button Response Matrix

#### Active outputs

- O2: Instant Speed, Instant Incline, Num0-Num9, Enter
- Separate relays: Speed+, Speed-

#### Passively recognized physical inputs

- Quick Start, Stop, Speed+/-, Incline+/-

#### Not actively generated

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

#### 4.5.1 Number 9 Exception

Number 9 outputs `0x81` in Phase B as preload and continues `0x81` through Phase C. This rule is specific to Number 9.

#### 4.5.2 Passive Phase D Recognition

O1/O2 are read atomically. A known Phase D combination must remain stable for approximately 30 ms. One `HardwareEvent` is generated per activation, held buttons are not repeated, and monitoring is suspended during O2 ownership.

### 4.6 Timing, Watchdog, Buzzer ACK, and Recovery

#### 4.6.1 Initial Synchronization

Injection begins only after a valid phase transition and fresh IDLE are obtained within timeout.

#### 4.6.2 Current Console-Interface Timing

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

At 82 ms, screening produced 131/140 `NORMAL_SINGLE`, 9/140 `NO_RESPONSE`, and no `NORMAL_LONG` or multi event. A clear ignored press is recoverable; an unrecognized ambiguous registration is the higher-risk condition.

#### 4.6.3 Qualified Buzzer Feedback

GPIO16 is active LOW and captured on CHANGE. The ISR stores timestamped edges. A task constructs events with sequence, start/end time, envelope, active duration, internal gap, edge count, and segment count.

- `NORMAL_SINGLE`: one normal segment
- `NO_RESPONSE`: no qualified event
- `NORMAL_LONG`: one segment longer than 110 ms, treated as ambiguous
- `MERGED_MULTI`: multiple segments in one envelope
- `DISTINCT_MULTI`: multiple envelopes
- `EDGE_LOSS`: dropped edges
- `INVALID`: unsafe classification

Measured 161 ms and 171 ms envelopes are `NORMAL_LONG`.

#### 4.6.4 ACK Causality

An ACK requires both a newer sequence and `event.startUs >= pressStartUs`. The evaluator rejects edge loss, multiple events, multiple segments, and a second event during the guard interval.

#### 4.6.5 Local Retry Policy

Only an unambiguous `NO_RESPONSE` permits one direct retry. No direct retry follows `NORMAL_LONG`, multi, `EDGE_LOSS`, or `INVALID`.

#### 4.6.6 CLR-Based Numeric Recovery

CLR is the selected authoritative numeric-buffer recovery mechanism, but the physical mapping is still pending. Recovery remains disabled until the target phase, O2 byte, timing, release, and buzzer behavior are measured.

```text
Ambiguous or duplicate result
-> stop numeric entry
-> send and verify CLR
-> resend the complete numeric value
-> send Enter only after all digits are confirmed
```

No placeholder CLR phase or byte is permitted.

Mapping must be verified in Instant Speed and Instant Incline after zero, one, and two digits, including proof that CLR clears without committing a value.

#### 4.6.7 Enter Gating

Enter is allowed only after Instant and every digit are confirmed and any retry or CLR recovery is complete. Ambiguous Enter behavior requires separate validation.

#### 4.6.8 Watchdog and Reconnect

On watchdog or hardware failure, release relays, return O2 to input atomically, wait 50 microseconds, release MUTE, settle, clear ownership, and report failure.

### 4.7 Command and Macro Execution

#### 4.7.1 Speed Target Algorithm

Speed range is 0.8-25.0 km/h. The integer base is entered through O2 and confirmed before Enter. Fractional correction uses Speed+/- relays after O2 reconnect.

- 12.6 -> base 13, then four Speed- relay pulses
- 12.4 -> base 12, then four Speed+ relay pulses

#### 4.7.2 Incline Target Algorithm

Incline range is 0-15%, whole percentages only. Non-integer targets are rejected. There is no active Incline+/- output.

#### 4.7.3 Macro Ownership and Relay Adjustment

Instant, numeric entry, ACK, retry, CLR recovery, sequence restart, and Enter remain under one continuous O2 ownership period. Speed relay pulses occur after controlled reconnect and 150 ms settling.

#### 4.7.4 Command Queue and Status

Requests use bounded FreeRTOS queues and states `queued`, `started`, `step`, `completed`, `rejected`, and `failed`. Status also reports ACK classification, retry, CLR recovery, sequence restart, and recovery failure.

### 4.8 Console Button Behavior

Physical Speed+/- and Incline+/- are ignored unless the belt is moving. Physical Quick Start, Stop, Speed+/-, and Incline+/- are passively recognized and never automatically reinjected.

### 4.9 Mechanical Incline Behavior

Physical Quick Start homes incline to 0%. Measured travel is approximately 49 seconds upward and 48 seconds downward. Board 2 contains two TXS0108E devices, shared MUTE translation, and hardware OE pull-down.

---

## 5. Sensor Interpretation

### 5.1 Data Authority Rules

Speed comes exclusively from Pin 7, incline from the homed pulse-integrated tracker, and system state from CSAFE. The IMU verifies incline but does not replace movement tracking.

### 5.2 Cadence and IMU (LSM6DSOX)

The LSM6DSOX is mounted to the moving deck, derives cadence from footstrike vibration, and provides filtered angle verification. The long I2C cable uses shielding and an LTC4311. A sensor fault returns cadence to 0 without affecting control.

The historical GPIO47 SDA allocation conflicts with the current Speed- relay and must be resolved in final hardware.

### 5.3 Runner Presence Detection

Measured speed above zero with cadence at zero for 2-3 seconds means `No Runner Present`. FTMS speed is forced to zero and distance accumulation is paused, without changing treadmill control state.

---

## 6. Speed Sensor (Pin 7)

### 6.1 PC817 Interface

A PC817 collector uses a 10 kOhm pull-up to 3.3 V. Speed uses a 10 kOhm LED-side resistor at approximately 1.1 mA. Incline uses 1 kOhm at approximately 3.8 mA.

### 6.2 Software Filter

Timestamp-based filtering uses approximately 300 microseconds micro-glitch rejection and 2000 microseconds lockout. No blocking ISR delay is used for speed.

### 6.3 Expected Signal

Approximately 2 Hz at 1 km/h, 9 Hz at 10 km/h, and 22 Hz at 25 km/h, with one accepted falling edge per rotation.

### 6.4 Continuity and Faults

A missing pulse timeout forces speed to zero. RUNNING with no valid pulses for more than 2 seconds logs a mismatch fault.

---

## 7. Incline Sensor (Pin 11)

### 7.1 Interpretation

Pin 11 emits approximately 394 Hz during movement. Incline is calculated from homing and direction-specific pulse integration, with optional IMU verification after movement.

### 7.2 Interface

The chosen isolated concept uses a PC817, 1 kOhm LED resistance, and 10 kOhm pull-up. BSS138 remains a legacy troubleshooting option. GPIO14 conflicts with the current Speed+ relay and must be reassigned in integrated hardware.

### 7.3 Homing

Physical Quick Start homes to 0%. When movement pulses stop, the tracker is set to 0.0%.

### 7.4 Persistence

Store incline position when movement stops or state becomes STOPPED. A `Re-home Incline` action is mandatory because NVS cannot detect untracked movement while powered off.

---

## 8. Calibration & Software Constants

### 8.1 Speed Calibration

- 10 km/h = 8.97 Hz
- 25 km/h = 22.3 Hz
- `km/h = Hz * 1.1148`
- 1 meter = 3.2292 pulses

### 8.2 Speed AutoCal

Use a fixed base factor, correction table, linear interpolation, stable-speed learning in RAM, and persistent writes only when STOPPED.

### 8.3 Incline Calibration

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

HLK-PM01 with mains fusing and isolation. Target continuous use below 70-80% rating. Verify sustained load, radio-induced rail sag, and thermal stability.

### 9.5 CSAFE Interface

RJ45 to MAX3232 at 9600 8-N-1. GPIO16 conflict with buzzer must be resolved.

### 9.6 Future Hardware Improvements

#### 9.6.1 Replace Speed+/- relays

Evaluate a deterministic solid-state solution such as PhotoMOS, isolated transistor, or appropriate open-drain interface, based on measured electrical requirements.

#### 9.6.2 Add active Incline+/- outputs

A future board may add two independent, protected, fail-inactive outputs. Current firmware supports whole percentages only.

#### 9.6.3 Replace TXS0108E

Evaluate explicit-direction translators, separate unidirectional stages, bus switches, Schmitt-trigger receivers, or dedicated drivers with predictable delay and high-impedance behavior.

#### 9.6.4 PCB and Wiring

Use shorter bus paths, continuous ground, controlled connectors, local decoupling, test points, and optional damping footprints.

---

## 10. Software Architecture Rules

### 10.1 Dual-Core Split

Core 0 owns Wi-Fi, web, LittleFS, BLE, logging, and IMU/cadence. Core 1 owns injection, panel monitoring, command execution, and watchdog responsibility. Queues, atomics, and short critical sections protect shared data.

### 10.2 Rate Control and Backpressure

Capture is interrupt-driven. Consumers read at fixed rates. Stale telemetry may be dropped, but pulse counts and homing state may not.

### 10.3 Filesystem, Persistence, and OTA

Use LittleFS, batched writes, atomic configuration replacement, separate firmware/filesystem artifacts, a custom partition table, and dual app partitions if OTA rollback is required.

### 10.4 AP Recovery

After repeated Wi-Fi failure, start AP mode with a portal for credentials, FTMS name, and incline re-home.

---

## 11. CSAFE Rules (RS232)

- 9600 baud, 8-N-1
- Keep-alive: `0xF1 0x85 0x85 0xF2`
- Status: `0xF1 0x80 0x80 0xF2`
- Poll every 200-250 ms
- Do not merge frames
- Never send `0xAA`, `0xA5`, or `0x9C`
- Reset parser on `0xF1`, parse on `0xF2`
- Application code never reads UART directly

---

## 12. Timing & Safety Rules

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
- Positive elevation gain calculated from interval distance and incline
- Cadence included when available
- Dual-role HR client and FTMS server
- HR MAC stored in NVS
- All BLE work on Core 0

---

## 14. GUI Architecture & Interval Coach

### 14.1 Stateless UI and Quick Keys

Vanilla JavaScript, WebSocket telemetry, `pointerdown`, eight-button speed and incline grids, HVILE/DRAG presets, persistent manual controls, measured speed, tracked incline, and optional IMU verification. Physical button events may update UI state through `hardwareEventQueue`.

### 14.2 Interval Coach

Visual and auditory only, never automated control. Includes focus mode, Web Audio cues, ETA, phase banners, and RPE prompt.

### 14.3 User Profile Schema

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
      "work_m": 0, "work_s": 45,
      "rest_m": 0, "rest_s": 15,
      "reps": 10, "series": 2,
      "series_rest_m": 3
    },
    "history": []
  }]
}
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
- [x] Passive Phase D button recognition
- [x] Speed fractional adjustment through relays
- [x] 82 ms general numeric baseline

### 15.2 Open Items

- [ ] Map CLR phase and O2 byte
- [ ] Characterize CLR timing, release, buzzer response, and pause
- [ ] Implement and validate CLR recovery
- [ ] Validate ambiguous Enter behavior
- [ ] End-to-end speed and incline regression
- [ ] Resolve integrated GPIO conflicts
- [ ] Future solid-state Speed+/- outputs
- [ ] Future active Incline+/- outputs
- [ ] Future deterministic TXS0108E replacement
- [ ] Final PCB/wiring revision
- [ ] Production qualification

### 15.3 CLR Verification Plan

Map and validate CLR in Instant Speed and Instant Incline after zero, one, and two digits. Confirm repeated operation, buzzer response, and that CLR does not commit a value.

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

The 10 kOhm pull-up can extend release transitions and cause threshold noise. Timestamp-based filtering replaces historical blanking heuristics.

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

### A.8 Design Decision Summary

```text
Use approximately 82 ms as the general numeric hold baseline.
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
```
