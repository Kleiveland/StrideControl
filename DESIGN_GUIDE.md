Klart — her er **hele filen oppdatert**, med de forbedringene vi diskuterte:

*   prosjektmål er med
*   verifiserte funn er med
*   valgte designregler er med
*   CSAFE-delen er oppdatert mot koden som faktisk fungerte
*   intern CSAFE-FSM er tatt med
*   fail-safe MitM-kravet er lagt tilbake
*   planlagt injeksjonssekvens er beholdt som **working rule**, ikke som ferdig verifisert sannhet
*   ingen unødvendig spekulasjon

Du kan lime dette rett inn i GitHub:

````md
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
````

**Reason:**  
Anchored to the measured `10 km/h` reference point, which is closest to the primary operating range.

### Derived odometer constant

```text
1 meter = 3.2292 pulses
```

This is the current **working odometer constant**, derived from the chosen speed calibration.

### 5.2 Incline Calibration

### Verified facts

*   Pin 11 emits `394 Hz` while incline is moving
*   `QUICK START` always homes incline to `0%`
*   Full travel is approximately `49 s` upward and `48 s` downward

### Chosen software rule

The software does **not** hard-code a final pulses-per-percent incline constant yet.

Instead:

1.  `QUICK START` is used to home incline to `0%`
2.  Pin 11 pulse counting starts from that baseline
3.  A full `0% -> 15%` pulse count is stored as the calibration span
4.  Actual incline is calculated dynamically from counted pulses relative to that span

A final fixed pulses-per-percent constant is intentionally **not** declared until a full measured `0% -> 15%` pulse count has been captured and validated.

***

## 6. Hardware Interface Choices

### 6.1 MCU and Isolation

*   **Controller:** ESP32-S3
*   **Speed isolation (Pin 7):** PC817 optocoupler with `1 kOhm` series resistor on the input
*   **Incline interface (Pin 11):** BSS138 level shifter to convert the `5V` logic pulse to a safe `3.3V` input for the ESP32

### 6.2 Power Supply

*   **Hardware:** HLK-PM01 AC-to-5V module
*   provides permanent internal power without relying on external USB

**Note:**  
Physical isolation and appropriate mains-side fusing are required.

### 6.3 Front Panel Strategy

Because the front panel is capacitive, raw touch lines will not be spoofed directly.

Future button injection is planned on the decoded front-panel bus, with the `SN74HC4066` retained as the current candidate switching element.

### 6.4 Fail-Safe MitM Requirement

The command-injection wiring must be designed so that loss of ESP32 power does not interrupt normal treadmill operation.

The original console path must remain functional in the absence of ESP32 activity. This fail-safe behavior must be achieved by the surrounding wiring topology and must not be assumed from the `SN74HC4066` alone.

***

## 7. Software Architecture Rules

### 7.1 Dual-Core Task Split

*   **Core 0:**
    *   WiFi
    *   LittleFS web server
    *   WebSocket communication
    *   Bluetooth FTMS
    *   system logging

*   **Core 1:**
    *   pulse counting interrupts for Pin 7 and Pin 11
    *   real-time GPIO injection tasks
    *   safety watchdog tasks

### 7.2 Safety and Logging

*   **Watchdog:** A `1.0-second` hardware watchdog timer (WDT) must be enabled. If the firmware stops responding, the controller reboots automatically.
*   **Logging:** The firmware must log treadmill state transitions, FTMS connection events, CSAFE framing anomalies, checksum anomalies, and safety timeouts to a local buffer for debugging.

***

## 8. CSAFE Rules (RS232)

### Connection

*   **Interface:** RJ45 to MAX3232
*   **Serial settings:** `9600 baud, 8-N-1`

### Polling Structure

The treadmill drops multi-command frames. Polling is therefore split into two separate frames:

1.  Keep-alive  
    `0xF1 0x85 0x85 0xF2`

2.  Status request  
    `0xF1 0x80 0x80 0xF2`

### Polling interval

*   `200 to 250 ms`

### Inter-frame timing

The working browser prototype sent the `0x85` and `0x80` frames back-to-back with no enforced delay.

A short inter-frame delay (for example around `40 ms`) may still be used as a defensive firmware option if later testing shows buffer sensitivity on the treadmill controller.

### Allowed and forbidden commands

Due to firmware behavior on the DK-City mainboard:

*   polling is restricted to:
    *   `0x80`
    *   `0x85`

The following commands are not used:

*   `0xAA` (Extended Telemetry)
*   `0xA5` (Speed)
*   `0x9C` (Error Codes)

These have been observed to corrupt the treadmill serial buffer or return invalid data.

### Serial Fragmentation and Buffer Reassembly

The ESP32 UART receives fragmented packets.

The code must:

*   assemble incoming bytes into a buffer
*   clear the buffer on `0xF1` (start frame)
*   only parse the state byte once `0xF2` (end frame) is received

### State decoding

The treadmill state is extracted from the received status byte using:

```text
state = raw_byte & 0x0F
```

### Observed raw state values

*   `5` = InUse
*   `6` = Paused
*   `8` = Starting
*   `0`, `1`, `2`, `3`, `15` = Stopped / Ready

### Internal CSAFE-Derived UI States

The firmware does not use raw CSAFE values directly as UI states.

Instead, it derives the following internal states:

*   `STOPPED`
*   `PAUSED`
*   `STARTING`
*   `RUNNING`
*   `IDLE`

### Rules used by the working prototype

*   Raw state `8` transitions the system into `STARTING`
*   Raw state `6` transitions the system into `PAUSED`
*   Raw state `5` completes the transition to `RUNNING` after the `3.5-second` startup delay
*   Raw state `5` may also be interpreted as `IDLE` if no valid startup sequence has been established
*   Raw states `0`, `1`, `2`, `3`, and `15` force `STOPPED`, except for temporary `15` jitter during `STARTING`

### Data Integrity

The working CSAFE prototype did **not** validate the incoming checksum byte.

Instead, frame validity was determined by:

*   resetting the receive buffer on `0xF1`
*   accumulating bytes until `0xF2`
*   extracting the first payload byte
*   masking with `0x0F`

Because of this, checksum mismatch is not currently treated as a hard reject condition.

***

## 9. Timing and Safety Rules

### Incline Timeout

*   maximum continuous incline motion: `60 seconds`
*   if Pin 11 remains active longer than this, trigger an emergency fault state

### Start-Up Synchronization

When raw CSAFE enters State `8` (Starting), the ESP32 ignores speed and distance accumulation for exactly `3.5 seconds` to align with the physical countdown.

### Start-Up Jitter Filter

When the treadmill is in internal state `STARTING`, brief spikes reporting raw State `15` (Ready) must be ignored to prevent resetting the `3.5-second` countdown.

### Resume Debounce

When transitioning from raw Paused (State `6`) back to raw InUse (State `5`), a `700 ms` debounce filter is applied before re-entering `STARTING`, preventing double-triggering.

### Ghost-Running Detection

If CSAFE reports raw state `5` (`InUse`) but Pin 7 reads `0 Hz` for more than `2.0 seconds`:

*   the UI is reset to a non-running state
*   the mismatch fault is logged

### CSAFE RX Watchdog

If no valid `0xF2` frame is received for `1500 ms`:

*   internal treadmill state is forced to `STOPPED`

This is the primary method for detecting:

*   physical E-Stop activation
*   serial cable disconnection
*   CSAFE link loss

***

## 10. Planned Command Injection Timing

The current planned command-injection sequence is:

1.  Isolate the decoded front-panel bus from the original console path
2.  Wait `1 ms`
3.  Inject the command sequence
4.  Wait `2 ms`
5.  Restore the original console path

This timing model is retained as the current working rule and must be validated against the final decoded front-panel bus implementation.

***

## 11. UI and Data Storage Rules

### Tablet UI

The tablet UI stores no treadmill state locally.

The ESP32 sends live JSON via WebSockets at `5-10 Hz`, and the browser only renders incoming data.

### Actual vs Target Values

*   actual speed is derived purely from Pin 7 pulses
*   actual incline is derived purely from Pin 11 pulses relative to the homed baseline
*   target values are displayed only when explicitly commanded by the user via the tablet

### Flash Wear Policy

The odometer is only written to flash/NVS when the treadmill state changes to `Stopped`, minimizing write cycles.

***

## 12. Bluetooth FTMS Rules

### BLE Stack

*   **Library:** `NimBLE-Arduino`

Chosen for:

*   lower memory footprint
*   better long-session stability

### Broadcast Rate

*   exactly `1 Hz`

### Data Formatting

*   speed is sent as an integer with `0.01` resolution  
    Example: `12.50 km/h -> 1250`

*   incline is sent as an integer with `0.1` resolution  
    Example: `3.5% -> 35`

### Control Points

Fitness Machine Control Point requests from training apps must be acknowledged with valid `ACK` responses.

***

## 13. Current Open Items

### Unmapped harness lines

*   Pin 3
*   Pin 4
*   Pin 5
*   Pin 8
*   Pin 9

### Still open in the project

*   full mapping of the post-touch front-panel bus
*   final command injection method on that bus
*   final incline span calibration from measured `0% -> 15%` pulse counting
*   synchronization with physical console key presses

***

## 14. Future Development

### Physical Console Key Synchronization

Once the decoded front-panel bus is mapped, the ESP32 will listen for physical button activity using interrupts and debounce filtering, allowing the tablet UI to remain synchronized with manual console input.

***

## 15. Current Project Status

### Verified and Usable

*   actual speed reading from Pin 7
*   incline movement reading from Pin 11
*   incline homing to `0%` via `QUICK START`
*   CSAFE state polling logic
*   CSAFE fragmentation handling
*   stateless tablet UI architecture
*   FTMS architecture

### Chosen and Fixed for Implementation

*   ESP32-S3 as controller
*   dual-core task split
*   PC817 + 1k resistor on speed input
*   BSS138 3.3V-safe interface on incline input
*   HLK-PM01 for permanent power
*   local web UI on tablet using WebSockets
*   FTMS via NimBLE
*   no direct spoofing of raw capacitive panel
*   Pin 10 ignored

### Planned but Still Open

*   decoded front-panel bus mapping
*   final command injection method
*   synchronization with physical console key presses

```

Hvis du vil, kan jeg også lage en **kort README-versjon** av denne i neste svar, så du får:
- én full teknisk spesifikasjon
- én kort prosjektoversikt til repo-forsiden.
```
