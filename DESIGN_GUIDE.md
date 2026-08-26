# StrideControl Design Guide

## 1. Purpose

StrideControl is an ESP32-S3-based interface for controlling and emulating the physical control panel of a treadmill. The system observes the treadmill console's multiplexed O1 phase bus and drives the corresponding O2 response bus to emulate panel button presses.

The design goals are:

- Deterministic and phase-correct O2 output.
- Continuous valid baseline output while StrideControl owns the O2 bus.
- Closed-loop verification through the treadmill buzzer signal.
- Controlled retry when a button press is confirmed as ignored.
- Recovery to a known input state when the received result is ambiguous or may contain duplicate registrations.
- Fail-safe release of all controlled outputs if the system cannot establish a known state.
- Data-driven characterization of button timing rather than relying on one global timing assumption.

This guide documents the established architecture, verified design decisions, current limitations, open mapping work, and future hardware improvements.

---

## 2. Hardware Platform

### 2.1 Controller

Target controller:

```text
ESP32-S3 DevKitC-1
```

The firmware uses both ESP32-S3 cores:

- Network and web services are separated from the timing-critical panel output task.
- Buzzer edge capture uses a GPIO interrupt and a dedicated processing task.
- The command engine performs tight phase polling while actively controlling O2.

### 2.2 Current pin allocation

```cpp
namespace BoardPins {
constexpr gpio_num_t TXS_OE            = GPIO_NUM_2;
constexpr gpio_num_t MUTE_ALL          = GPIO_NUM_21;
constexpr gpio_num_t ESTOP_IN          = GPIO_NUM_18;
constexpr gpio_num_t BUZZER_IN         = GPIO_NUM_16;
constexpr gpio_num_t SIDE_SPEED_PLUS   = GPIO_NUM_14;
constexpr gpio_num_t SIDE_SPEED_MINUS  = GPIO_NUM_47;

constexpr int O1[5] = {4, 5, 6, 7, 15};
constexpr int O2[8] = {41, 42, 8, 9, 10, 11, 12, 13};
}
```

### 2.3 Electrical isolation and level shifting

The current design uses a TXS0108E-based level-shifting path for the O1/O2 interface. The TXS solution has been useful during development, but it is not considered the preferred final hardware architecture.

Observed short transition combinations on O1 are not solely caused by the TXS0108E. USB logic-analyzer measurements taken directly on the original treadmill bus, with no TXS0108E or ESP32 connected, also showed short parallel-bus transition values. These are consistent with bit skew while multiple bus lines change.

The TXS0108E can still influence:

- Edge shape.
- Rise and fall times.
- Ringing.
- Threshold crossing.
- Behavior with long wiring or capacitive loading.
- Channel-to-channel consistency.

A more deterministic interface is listed as a future hardware improvement in Section 13.

---

## 3. O1 Phase Bus

### 3.1 Valid phases

```cpp
namespace PanelMap {
constexpr uint8_t PHASE_A = 0x0F;
constexpr uint8_t PHASE_B = 0x17;
constexpr uint8_t PHASE_C = 0x1B;
constexpr uint8_t PHASE_D = 0x1D;
constexpr uint8_t IDLE    = 0x1F;
}
```

### 3.2 Measured timing

The original treadmill bus and the ESP32 SmartRecorder measurements agree on the approximate cycle:

```text
Phase A: approximately 1 ms
Phase B: approximately 1 ms
Phase C: approximately 1 ms
Phase D: approximately 1 ms
IDLE:    approximately 3 ms
Full cycle: approximately 7 ms
```

### 3.3 Transition values

Short O1 transition values can occur while parallel lines change at slightly different times. Firmware must not assume that every instantaneous O1 snapshot represents a complete protocol phase.

The phase tracker therefore distinguishes between:

- Raw observed O1 value.
- Last observed value that matches a known phase code.

The current command engine keeps the established low-latency phase-tracking design. A full 50 microsecond qualification delay is not inserted into the timing-critical drive path because that delay would reduce available O2 setup time.

The robust waiting functions must not treat an invalid O1 value as proof that the target phase or IDLE has been left.

---

## 4. O2 Baselines and Button Mapping

### 4.1 Baselines

```cpp
constexpr uint8_t BASE_A      = 0x80;
constexpr uint8_t BASE_B      = 0x80;
constexpr uint8_t BASE_C      = 0x80;
constexpr uint8_t BASE_D      = 0xC0;
constexpr uint8_t IDLE_FIRST  = 0x00;
constexpr uint8_t IDLE_SECOND = 0xFF;
```

During IDLE, the response changes from `0x00` to `0xFF` after the measured IDLE switching interval.

### 4.2 Numeric button mapping

```text
Phase A:
Num1 = 0x82
Num4 = 0x84
Num5 = 0x90
Num7 = 0x88
Num8 = 0xA0

Phase B:
Num0 = 0xA0
Num2 = 0x90
Num3 = 0x84
Num6 = 0x82
Enter = 0x88

Num9:
Observed with 0x81 and handled across the established relevant phase behavior.
```

### 4.3 Other mapped buttons

```text
Instant Incline = 0xC0 in Phase A
Instant Speed   = 0xC0 in Phase B

Speed Plus      = 0xC4 in Phase D
Speed Minus     = 0xC8 in Phase D
Incline Plus    = 0xC1 in Phase D
Incline Minus   = 0xE0 in Phase D
Quick Start     = 0xD0 in Phase D
Stop            = 0xC2 in Phase D
```

---

## 5. Active Baseline Ownership

When StrideControl takes ownership of O2:

1. The physical panel is muted.
2. The current phase-appropriate baseline is written to O2.
3. O2 is changed to output mode.
4. The correct baseline continues to be driven during target-phase waiting, ACK waiting, cooldown, and inter-step pauses.
5. The active button code is only driven during the relevant target phase.
6. Release occurs phase-coherently after the configured minimum hold time has elapsed.
7. At transaction end, O2 is returned to input and the physical panel is released.

The initialization pattern:

```cpp
uint8_t lastDriven = static_cast<uint8_t>(~initialBaseline);
driveO2Atomic(initialBaseline, lastDriven);
```

uses the inverted value only as a cache-invalidating sentinel. It guarantees that the initial baseline is physically written. The inverted value is not driven onto the treadmill bus and is not a software pull-up or pull-down.

---

## 6. Buzzer Feedback

### 6.1 Purpose

The treadmill buzzer is the closed-loop response channel. The buzzer input is connected to:

```text
GPIO16
```

The validated interface is active LOW through the installed protected interface.

A buzzer response confirms that the treadmill console decoded a button event. A visible display change is not required. For example, pressing a number outside Instant Speed or Instant Incline mode can produce a buzzer response without changing the display.

### 6.2 Edge capture

The buzzer signal is captured with a GPIO interrupt. The interrupt handler only timestamps and stores edges in a ring buffer. Event construction and classification occur in a FreeRTOS task.

The implementation must track:

```text
Event start time
Event end time
Envelope duration
Qualified active duration
Qualified segment count
Longest internal inactive gap
Raw edge count
Dropped edge count
```

### 6.3 Qualified response classes

```text
NORMAL_SINGLE
A single qualified buzzer response with no detected internal multi-segment behavior.

NO_RESPONSE
No qualified buzzer response within the complete observation and cooldown period.

NORMAL_LONG
A single-segment response with an envelope outside the established normal profile. This is treated as ambiguous until further characterized.

MERGED_MULTI
Multiple qualified active segments inside one buzzer envelope, separated by an internal dip.

DISTINCT_MULTI
Multiple separate buzzer events associated with one logical button transaction.

INVALID
A response that cannot be classified with sufficient confidence.

EDGE_LOSS
The buzzer edge buffer overflowed or lost edge information during the observation.
```

A known duplicate registration produced a buzzer response with an audible dip. The digital capture method can detect this as multiple qualified active segments with an internal gap.

### 6.4 Causal ACK association

A buzzer event may only acknowledge a button transaction if the event started after the current button press began. A sequence number alone is insufficient because an event from the previous step can still be open when the next step begins.

The ACK logic must qualify both:

```text
Event sequence is newer than the pre-press snapshot.
Event start timestamp belongs to the current press.
```

---

## 7. Retry and Recovery Policy

### 7.1 General principle

The system does not require every button press to succeed on the first attempt. The primary goals are:

1. Detect an ignored press reliably.
2. Retry only when the press is confidently classified as ignored.
3. Detect duplicate or ambiguous registration.
4. Restore the console input buffer to a known state before restarting an uncertain sequence.
5. Never send Enter after an unverified numeric sequence.

### 7.2 Direct digit retry

```text
NORMAL_SINGLE
Continue to the next digit.

NO_RESPONSE
Retry the same digit once after the full observation window and cooldown.

NORMAL_SINGLE after retry
Continue to the next digit.

NO_RESPONSE after retry
Do not continue to Enter. Invoke sequence recovery.
```

A direct retry is permitted only for a qualified `NO_RESPONSE`.

A direct retry must not occur after:

```text
NORMAL_LONG
MERGED_MULTI
DISTINCT_MULTI
INVALID
EDGE_LOSS
```

These states do not prove that the original button was ignored.

### 7.3 Duplicate or ambiguous registration

If the system detects a possible duplicate or cannot determine what the console accepted:

```text
Stop the active numeric sequence.
Clear the console numeric input buffer.
Restart the complete numeric sequence.
Send Enter only after every digit in the restarted sequence is unambiguously confirmed.
```

### 7.4 Recovery limits

Recommended limits:

```text
Maximum direct retry per digit: 1
Maximum complete sequence restarts: 2
Maximum CLR attempts per recovery: 2
```

If the recovery limit is exceeded:

```text
Do not send Enter.
Release controlled outputs.
Return a recovery failure status.
```

---

## 8. CLR Recovery Architecture

### 8.1 Design decision

`CLR` shall be the authoritative recovery mechanism for returning the treadmill numeric-entry buffer to a known empty state.

This replaces the less explicit approach of forcing a buffer reset by sending additional digits.

The planned recovery sequence is:

```text
Ambiguous or duplicate digit response
→ stop current numeric sequence
→ send CLR
→ verify CLR response
→ resend the complete requested numeric value
→ send Enter only after all digits are confirmed
```

The same recovery principle applies when the firmware is uncertain whether the treadmill read a button press.

### 8.2 Current open item: CLR mapping is missing

The physical `CLR` button has not yet been mapped on the O1/O2 bus.

The recovery architecture is established, but implementation must wait until the physical mapping is measured and verified.

The following values are currently unknown:

```text
CLR target phase
CLR active O2 byte
CLR baseline byte
CLR hold time
CLR release behavior
CLR buzzer response
Required post-CLR pause
CLR behavior in Instant Speed mode
CLR behavior in Instant Incline mode
```

No phase, byte value, timing, or buzzer behavior for `CLR` shall be guessed.

### 8.3 CLR mapping method

`CLR` shall be measured in the same way as the other physical panel buttons:

1. Disable ESP32 O2 emulation and allow the original panel to control the bus.
2. Enter Instant Speed mode and perform physical CLR presses in relevant input states.
3. Enter Instant Incline mode and repeat the measurements.
4. Observe O1 to identify the active phase.
5. Observe O2 to determine the stable button byte.
6. Separate stable values from short transition combinations.
7. Record the baseline before the press.
8. Record the active O2 value throughout the target phase.
9. Record the release behavior.
10. Record the buzzer response.
11. Repeat enough times to establish a stable mapping.
12. Verify that CLR empties the input buffer after both one and two entered digits.

USB logic-analyzer measurement is preferred if all required signals can be captured on a common timebase. With an eight-channel analyzer, the measurement should prioritize the five O1 lines, the O2 line or lines needed to resolve CLR, the buzzer response, and one additional reference if available.

### 8.4 Future implementation interface

When the mapping is known, add:

```cpp
ButtonId::Clear
```

and dedicated timing parameters:

```cpp
CLR_HOLD_MS
POST_CLEAR_PAUSE_MS
```

The sequence-level recovery state machine should return a separate status if CLR cannot be confirmed:

```text
CLEAR_FAILED
```

If CLR fails:

```text
Do not send additional digits.
Do not send Enter.
Abort the macro and return O2 to the passive state.
```

---

## 9. Timing Strategy

### 9.1 General digit timing

Existing characterization indicates that a general digit hold around:

```text
82 ms
```

is a reasonable starting point for normal operation when combined with qualified buzzer feedback and controlled retry.

At 82 ms in the scientific screening dataset for the seven retested digits:

```text
131 of 140 trials were NORMAL_SINGLE.
9 of 140 trials were NO_RESPONSE.
No NORMAL_LONG or multi response was observed at that screening point.
```

The design therefore prioritizes:

```text
A conservative general hold time
+ qualified ACK
+ one direct retry for confirmed NO_RESPONSE
+ CLR-based restart for ambiguous or duplicate registration
```

over pursuing a unique millisecond value for every digit before the recovery architecture is complete.

### 9.2 Characterized values

The AutoTune results remain valuable as diagnostic data. Button-specific hold values may be adopted later if they provide measurable improvements after the CLR-based recovery mechanism is implemented and verified.

No AutoTune recommendation shall be considered production-qualified solely because it achieved a short perfect block. Final validation must include complete numeric sequences and actual treadmill behavior.

### 9.3 Enter

Enter must only be sent after the complete numeric sequence is confirmed.

Enter timing must remain separately configurable because Enter has shown different behavior from ordinary digits, including multiple buzzer responses at longer hold times.

---

## 10. Sequence-Level State Machine

The numeric-entry workflow should be implemented at sequence level.

Recommended states:

```text
ACTIVATE_ENTRY_MODE
CLEAR_BUFFER
SEND_DIGIT
WAIT_DIGIT_ACK
RETRY_DIGIT
RECOVERY_CLEAR
RESTART_SEQUENCE
SEND_ENTER
WAIT_ENTER_ACK
COMPLETED
FAILED
```

Recommended response mapping:

```text
NORMAL_SINGLE
→ accept the current step

NO_RESPONSE
→ one direct retry of the same digit

NO_RESPONSE after retry
→ CLR and restart the complete numeric sequence

NORMAL_LONG
MERGED_MULTI
DISTINCT_MULTI
INVALID
EDGE_LOSS
→ CLR and restart the complete numeric sequence

CLR failure
→ abort without Enter
```

This recovery logic belongs above the low-level function that transmits one button. The low-level button function should report an outcome and must not silently restart the complete sequence.

---

## 11. Fail-Safe Behavior

On command-engine failure:

```text
Release Speed Plus output.
Release Speed Minus output.
Return O2 pins to input mode.
Release MUTE_ALL.
Stop the current macro.
Do not execute remaining adjustment steps.
Report the exact failing state.
```

This is a controlled release to a passive state. The firmware does not automatically activate the treadmill emergency stop unless a separate, explicitly verified design is implemented for that purpose.

The system must not send Enter when:

```text
A digit is unconfirmed.
The buzzer measurement is ambiguous.
Edge loss occurred.
CLR recovery failed.
The maximum retry or restart count was exceeded.
```

---

## 12. Web Interface and Logging

The existing web architecture shall be retained:

```text
Wi-Fi station connection with fallback access point.
HTTP server on port 80.
WebSocket server on port 81.
Hostname: tredemolle.local
```

The GUI should continue to provide:

- Manual speed and incline requests.
- Virtual panel buttons.
- Timing configuration.
- Live command status.
- Buzzer and recovery diagnostics.
- Downloadable CSV for characterization and validation tests.

Future recovery logging should include:

```text
Requested value
Entry mode
Current digit index
Digit value
Configured hold time
Initial response classification
Whether direct retry was used
Retry response classification
Whether CLR recovery was used
CLR response classification
Sequence restart count
Whether Enter was sent
Final macro result
Total completion time
```

---

## 13. Future Hardware Improvements

The following improvements are outside the current software-only update and shall be treated as future design work.

### 13.1 Replace relay-based Speed Plus and Speed Minus control

The current side Speed Plus and Speed Minus controls use relay-style outputs.

A future revision should evaluate a solid-state alternative with:

- Deterministic switching time.
- No contact bounce.
- No mechanical wear.
- Lower power consumption.
- Smaller physical size.
- Well-defined fail-open behavior.
- Adequate galvanic isolation where required.

Possible implementation classes include:

```text
Optically isolated transistor output
PhotoMOS or solid-state relay
Open-drain transistor interface
Optocoupler with a suitable output transistor
```

The selected circuit must be verified against the treadmill signal voltage, current, polarity, contact topology, and fail-safe requirements. No replacement topology shall be chosen solely from logic-level assumptions.

### 13.2 Add controlled Incline Plus and Incline Minus

Incline Plus and Incline Minus are already mapped on the panel protocol, but the current design treats the physical incline controls as unavailable for direct automated control.

A future hardware revision should add independent, protected control paths for:

```text
Incline Plus
Incline Minus
```

The implementation should follow the same design requirements as the improved Speed Plus and Speed Minus outputs:

- Defined passive state.
- Electrical isolation where necessary.
- No unintended activation during ESP32 reset or boot.
- Hardware pull-state that guarantees release.
- Independent verification of each output.
- Recovery and timeout behavior in firmware.

### 13.3 Replace TXS0108E with a more deterministic interface

The TXS0108E auto-direction architecture is not the preferred long-term solution for a multiplexed parallel control bus with long wiring and potentially asymmetric loading.

A future revision should evaluate a more deterministic solution with explicit signal direction and stronger control of the electrical interface.

Candidate architectural approaches include:

```text
Explicit-direction level translators
Separate unidirectional buffers for O1 and O2
Open-drain or open-collector drivers where electrically compatible
Bus switches or analog switches selected for the measured voltage and bandwidth
Optically isolated digital channels where timing permits
Dedicated input receivers with hysteresis and dedicated output drivers
```

Preferred design characteristics:

- Explicit direction control.
- Defined high-impedance state.
- Predictable propagation delay.
- Adequate drive strength.
- Better tolerance of cable capacitance.
- Reduced ringing and threshold ambiguity.
- Per-channel observability and test points.
- Fail-passive behavior during reset or power loss.

The final choice must be based on measured bus voltages, current requirements, edge rates, loading, directionality, and required high-impedance behavior.

### 13.4 Improve physical wiring and PCB layout

A future hardware revision should reduce reliance on long jumper wiring and include:

- Shorter O1 and O2 paths.
- Continuous ground reference.
- Ground conductors adjacent to sensitive signals where practical.
- Separation between fast-changing lines.
- Clearly labeled test points on both sides of each interface stage.
- Controlled connector pinout.
- Decoupling local to each interface IC.
- Optional series damping footprints.
- Optional pull-up or pull-down footprints based on measured requirements.

GPIO8 and GPIO9, and the associated O2 bit paths, should receive specific attention because earlier testing suggested channel-dependent behavior.

### 13.5 Persistent validated configuration

After the CLR mapping and recovery mechanism are verified, a future version may store validated timing and recovery parameters in NVS or another controlled configuration store.

Persistent values should include:

```text
Firmware/configuration version
General digit hold time
Optional per-button hold times
Enter timing
CLR timing
Retry limits
Recovery limits
Buzzer qualification thresholds
Validation date and test run identifier
```

The system should retain a known-good default profile and reject incomplete or incompatible stored configurations.

---

## 14. Verification Plan

### 14.1 CLR verification

Before enabling CLR-based automatic recovery:

```text
Map CLR electrically.
Confirm CLR in Instant Speed mode.
Confirm CLR in Instant Incline mode.
Confirm CLR after one entered digit.
Confirm CLR after two entered digits.
Confirm buzzer response.
Confirm repeated CLR behavior.
Confirm that CLR does not unintentionally commit a value.
```

### 14.2 Closed-loop recovery verification

Test complete sequences rather than only isolated digits.

Recommended values:

```text
10
11
12
15
20
22
25
```

These cover:

- Zero as second digit.
- Repeated digits.
- Different first and second digits.
- Digits with historically different response rates.

Each validation run should record:

```text
First-pass success
Direct digit retries
CLR recoveries
Complete sequence restarts
Ambiguous or multi responses
Blocked Enter events
Completed correct values
Failed sequences
Average completion time
```

### 14.3 Acceptance priorities

The acceptance priorities are:

```text
1. No Enter after an unverified digit sequence.
2. No direct retry after an ambiguous or multi response.
3. Reliable CLR recovery to a known input state.
4. Correct final value after recovery.
5. High success after at most one direct digit retry.
6. Low average completion time.
7. High first-pass success rate.
```

---

## 15. Current Design Status

### Verified or strongly supported

```text
O1 phase sequence and approximate timing.
O2 baseline model.
Numeric button mapping.
Active baseline ownership.
Phase-coherent release.
GPIO16 buzzer capture.
Qualified detection of normal, missing, long, merged, and separate buzzer responses.
Controlled direct retry after qualified NO_RESPONSE.
Fail-safe release of O2 and physical side outputs.
Web-based operation and logging.
General digit hold around 82 ms as a practical baseline with closed-loop handling.
```

### Open items

```text
Physical CLR mapping.
CLR timing and buzzer behavior.
CLR recovery implementation.
Full sequence-level recovery validation.
Solid-state replacement for Speed Plus and Speed Minus relays.
Automated Incline Plus and Incline Minus hardware control.
Replacement of TXS0108E with a more deterministic interface.
Final PCB and wiring revision.
Production qualification of the complete system.
```

---

## 16. Design Decision Summary

The current direction is:

```text
Use approximately 82 ms as the general numeric hold baseline.
Treat qualified NO_RESPONSE as a recoverable ignored press.
Allow one direct retry only after qualified NO_RESPONSE.
Treat long, multiple, invalid, or edge-loss responses as an unknown input-buffer state.
Use CLR to restore a known empty input buffer.
Restart the complete numeric sequence after CLR.
Send Enter only after every digit is unambiguously confirmed.
Do not guess the CLR bus mapping.
Map and verify CLR from the physical panel before implementation.
Retain the established active-baseline and qualified-ACK architecture.
Plan future hardware improvements for side-button control, incline control, and deterministic level translation.
```
