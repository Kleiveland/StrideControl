StrideControl Technical Design Guide
This document contains all verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system safely and predictably.

1. Project Goal
The goal of the project is to build a safe and reversible ESP32-S3 based interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:

hosts a local web interface for a tablet

allows the user to set treadmill speed and incline directly from that tablet interface

reads actual speed and incline movement directly from treadmill hardware

reads treadmill state from the CSAFE port

broadcasts treadmill telemetry over Bluetooth FTMS

preserves the original console and treadmill safety behavior

keeps the original treadmill usable even if the ESP32 is off or rebooting

The project is designed as a non-destructive overlay system, not a full treadmill controller replacement.

Primary User-Facing Goal
The primary user-facing goal of StrideControl is to provide a clean, tablet-based web interface mounted on the treadmill for direct speed and incline control.

Because the tablet will cover the original console’s speed and incline presentation, the web UI must also show the treadmill’s actual speed and actual incline as derived from hardware feedback, not just target values.

2. System Scope and Main Decisions
Chosen system role
The ESP32-S3 is responsible for:

web UI

telemetry processing

FTMS broadcasting

local logging

future command injection

The treadmill itself remains responsible for:

belt drive control (3.5 HP AC motor)

incline motor power

original safety logic (safety key, emergency stops)

original console behavior

Important design choices
The tablet UI is stateless and only renders live data sent from the ESP32.

The original treadmill must continue to operate if the ESP32 loses power.

Raw capacitive touch sensing in the front panel will not be spoofed directly.

Future command injection will target the front-panel bus after the touch electronics, not the raw capacitive panel itself.

3. Verified Physical Findings
3.1 Console and Wiring
The console uses a main control PCB marked APM 900T / APS469B.

The front panel is capacitive.

The RJ45/RS232 interface is the CSAFE port.

The console-to-treadmill harness has 12 signal conductors, plus 2 separate fan power conductors.

3.2 Harness Measurements (Verified)
Black wire: +11.75V DC, constant

Brown wire: 0V, constant (measurement reference / GND)

Pin 6: 5.0V DC, constant

Pin 12: 5.0V DC, constant

Pins with dynamic behavior
Pin 7 (Speed)
11.4V DC at rest

At low speed, DC reading fluctuates rapidly between 5V and 13V

8.97 Hz at 10 km/h

22.3 Hz at 25 km/h

Pin 11 (Incline)
4.682V DC at rest

averages 2.2V and outputs a constant 394 Hz while the incline is physically moving

returns to approximately 0V or 4.682V when motion stops

pulses only while the incline is moving

Pin 10
0.09V DC at rest

not used in the current design

Unmapped lines
Pins 3, 4, 5, 8, and 9 measure at approximately 4.3V DC in standby

these are not yet mapped to named functions

3.3 Mechanical Incline Behavior
A QUICK START command always drives the incline physically down to 0%

Measured incline travel time under user load:

0% -> 15% = 49 seconds

15% -> 0% = 48 seconds

This behavior is used as the incline homing baseline in software.

3.4 Console Button Behavior (Verified)
The physical Speed (+/-) and Incline (+/-) buttons are ignored by the treadmill controller unless the belt is actively moving.

It is physically impossible to pre-set a target speed or incline via the physical console before starting the treadmill.

4. Sensor Interpretation Used by the Project
4.1 Speed Input
Verified: Pin 7 is used as the actual speed feedback input.

Reason:
It has a stable rest voltage and provides a frequency output directly proportional to belt movement.

Implementation:
Pin 7 is treated as the single source of truth for treadmill actual belt speed.

4.2 Incline Input
Verified: Pin 11 is used as the incline movement feedback input.

Reason:
It has a stable rest state and emits a pulse train only while the incline motor is physically moving, stopping immediately when movement ceases.

Implementation:
Pin 11 is not treated as a direct analog angle value. Actual incline is calculated in software by homing to 0% and then counting movement pulses.

5. Calibration Data and Software Constants
5.1 Speed Calibration
Verified reference points
10 km/h = 8.97 Hz

25 km/h = 22.3 Hz

Derived factors
10 / 8.97 = 1.1148

25 / 22.3 = 1.1211

Chosen working constant
Plaintext
Speed (km/h) = Frequency (Hz) * 1.1148
Reason:
Anchored to the measured 10 km/h reference point, which is closest to the primary operating range.

Derived odometer constant
1 meter = 3.2292 pulses
This is the current working odometer constant, derived from the chosen speed calibration.

5.2 Incline Calibration
Verified facts

Pin 11 emits 394 Hz while incline is moving

QUICK START always homes incline to 0%

Full travel is approximately 49 s upward and 48 s downward

Chosen software rule
The software does not hard-code a final pulses-per-percent incline constant yet.
Instead:

QUICK START is used to home incline to 0%

Pin 11 pulse counting starts from that baseline

A full 0% -> 15% pulse count is stored as the calibration span

Actual incline is calculated dynamically from counted pulses relative to that span

A final fixed pulses-per-percent constant is intentionally not declared until a full measured 0% -> 15% pulse count has been captured and validated.

6. Hardware Interface Choices
6.1 MCU and Isolation
Controller: ESP32-S3

Speed isolation (Pin 7): PC817 optocoupler with 1 kOhm series resistor on the input

Incline interface (Pin 11): BSS138 level shifter to convert the 5V logic pulse to a safe 3.3V input for the ESP32

6.2 Power Supply
Hardware: HLK-PM01 AC-to-5V module

provides permanent internal power without relying on external USB

Note: Physical isolation and appropriate mains-side fusing are required.

6.3 Front Panel Strategy
Because the front panel is capacitive, raw touch lines will not be spoofed directly.
Future button injection is planned on the decoded front-panel bus, with the SN74HC4066 retained as the current candidate switching element.

6.4 Fail-Safe MitM Requirement
The command-injection wiring must be designed so that loss of ESP32 power does not interrupt normal treadmill operation.
The original console path must remain functional in the absence of ESP32 activity. This fail-safe behavior must be achieved by the surrounding wiring topology and must not be assumed from the SN74HC4066 alone.

6.5 Bootstrap, Recovery and Local Configuration
The device must be deployable and recoverable without requiring a USB cable or development PC.

First-boot behavior
On first boot, or whenever no valid WiFi credentials are available, the ESP32 shall start in Access Point mode.
A local configuration page shall be exposed for entering WiFi credentials and core device settings.
If WiFi association fails repeatedly, the device shall automatically fall back to Access Point mode instead of remaining offline indefinitely.
The recovery interface shall remain available even if FTMS, CSAFE, or the main UI stack is disabled.

Minimum configuration scope
The local configuration page should allow:

WiFi SSID and password

treadmill profile / machine profile selection

advertised device name

debug mode enable / disable

FTMS enable / disable

optional CSAFE polling interval override

Safety rule
Configuration mode must not interfere with treadmill telemetry capture once the device is already configured and operating normally.

7. Software Architecture Rules
7.1 Dual-Core Task Split
Core 0:

WiFi

LittleFS web server

WebSocket communication

Bluetooth FTMS

system logging

Core 1:

pulse counting interrupts for Pin 7 and Pin 11

real-time GPIO injection tasks

safety watchdog tasks

7.2 Safety and Logging
Watchdog: A 1.0-second hardware watchdog timer (WDT) must be enabled. If the firmware stops responding, the controller reboots automatically.
Logging: The firmware must log treadmill state transitions, FTMS connection events, CSAFE framing anomalies, checksum anomalies, and safety timeouts to a local buffer for debugging.

7.3 Configuration Classes and Persistent Settings
Hardware calibration, user preferences, and transient runtime state must be stored separately.

Configuration classes
The firmware shall separate persistent values into at least these classes:

Hardware calibration (speed factor, pulses per meter, incline span, homing-related parameters)

Device preferences (WiFi settings, FTMS device name, debug mode, feature flags)

User profile settings (drag target defaults, pause target defaults, preset values)

Runtime-only state (raw counters, current treadmill state, current FTMS session state, current UI state)

Safety rule
Runtime-only state must never be treated as durable calibration data.

7.4 Telemetry Rate Control and Backpressure
Real-time pulse capture, WebSocket rendering, CSAFE parsing, and FTMS broadcasting must not compete directly for timing or buffer space.

Data path separation
Telemetry must pass through the following layers:

hardware capture layer

derived metrics layer

internal state layer

output adapters: WebSocket UI, FTMS, local logging, optional future export layers

Required rate limits

Hardware pulse counting is interrupt-driven. No UI work is allowed inside interrupt handlers.

Derived metric updates must run at a fixed internal cadence.

WebSocket UI pushes must be bounded to a configured rate.

FTMS broadcast uses a separate fixed rate.

Logging must be queued and non-blocking.

Backpressure rule
If a downstream consumer cannot keep up:

the newest valid state should replace the previous pending state

stale telemetry frames may be dropped

pulse counters, odometer, and homing-critical state must never be dropped

Safety rule
No output layer may block the sensor capture layer.

8. CSAFE Rules (RS232)
Connection
Interface: RJ45 to MAX3232

Serial settings: 9600 baud, 8-N-1

Polling Structure
The treadmill drops multi-command frames. Polling is therefore split into two separate frames:

Keep-alive: 0xF1 0x85 0x85 0xF2

Status request: 0xF1 0x80 0x80 0xF2

Polling interval: 200 to 250 ms

Inter-frame timing:
The working browser prototype sent the 0x85 and 0x80 frames back-to-back with no enforced delay.
A short inter-frame delay (for example around 40 ms) may still be used as a defensive firmware option if later testing shows buffer sensitivity on the treadmill controller.

Allowed and forbidden commands
Due to firmware behavior on the DK-City mainboard, polling is restricted to:

0x80

0x85

The following commands are not used:

0xAA (Extended Telemetry)

0xA5 (Speed)

0x9C (Error Codes)
These have been observed to corrupt the treadmill serial buffer or return invalid data.

Serial Fragmentation and Buffer Reassembly
The ESP32 UART receives fragmented packets.
The code must:

assemble incoming bytes into a buffer

clear the buffer on 0xF1 (start frame)

only parse the state byte once 0xF2 (end frame) is received

State decoding
The treadmill state is extracted from the received status byte using:
state = raw_byte & 0x0F

Observed raw state values:
5 = InUse, 6 = Paused, 8 = Starting, 0, 1, 2, 3, 15 = Stopped / Ready

Internal CSAFE-Derived UI States
The firmware does not use raw CSAFE values directly as UI states.
Instead, it derives the following internal states:

STOPPED

PAUSED

STARTING

RUNNING

IDLE

Rules used by the working prototype:

Raw state 8 transitions the system into STARTING

Raw state 6 transitions the system into PAUSED

Raw state 5 completes the transition to RUNNING after the 3.5-second startup delay

Raw state 5 may also be interpreted as IDLE if no valid startup sequence has been established

Raw states 0, 1, 2, 3, and 15 force STOPPED, except for temporary 15 jitter during STARTING

Data Integrity
The working CSAFE prototype did not validate the incoming checksum byte.
Instead, frame validity was determined by:

resetting the receive buffer on 0xF1

accumulating bytes until 0xF2

extracting the first payload byte

masking with 0x0F
Because of this, checksum mismatch is not currently treated as a hard reject condition.

CSAFE Transport-Layer Contract
CSAFE handling shall be implemented as a dedicated transport/parser layer, not as ad-hoc serial reads inside application logic.

Required separation
The CSAFE stack shall be divided into:

UART transport

frame detection / reassembly

command scheduler

response matcher

raw state extractor

treadmill-specific state translator

Scheduler rule
Only a controlled set of commands may be sent by the scheduler.
The command layer must not allow random feature polling from multiple callers.

Response-matching rule
The parser must assume: fragmented frames, delayed frames, missing frames, and treadmill-specific response irregularities.

Safety rule
Application code must never read directly from UART. All CSAFE input must pass through the framed parser and treadmill-specific translator.

9. Timing and Safety Rules
Incline Timeout: maximum continuous incline motion: 60 seconds. If Pin 11 remains active longer than this, trigger an emergency fault state.

Start-Up Synchronization: When raw CSAFE enters State 8 (Starting), the ESP32 ignores speed and distance accumulation for exactly 3.5 seconds to align with the physical countdown.

Start-Up Jitter Filter: When the treadmill is in internal state STARTING, brief spikes reporting raw State 15 (Ready) must be ignored to prevent resetting the 3.5-second countdown.

Resume Debounce: When transitioning from raw Paused (State 6) back to raw InUse (State 5), a 700 ms debounce filter is applied before re-entering STARTING, preventing double-triggering.

Ghost-Running Detection: If CSAFE reports raw state 5 (InUse) but Pin 7 reads 0 Hz for more than 2.0 seconds: the UI is reset to a non-running state and the mismatch fault is logged.

CSAFE RX Watchdog: If no valid 0xF2 frame is received for 1500 ms: internal treadmill state is forced to STOPPED. This is the primary method for detecting physical E-Stop activation, serial cable disconnection, or CSAFE link loss.

Command Injection Lockout: The command injector must discard any requests to adjust speed or incline unless the internal system state is currently RUNNING.

Command Queuing (5-Second Rule): Because the treadmill hardware ignores speed/incline inputs until the belt is moving, UI commands issued while the internal state is STARTING (or immediately after pressing start) must be placed in a queue. Once the state transitions to RUNNING, the queued commands are injected. If the treadmill does not reach RUNNING within 5 seconds of the user's input, the queued command expires and is discarded.

10. Planned Command Injection Timing
The current planned command-injection sequence is:

Isolate the decoded front-panel bus from the original console path

Wait 1 ms

Inject the command sequence

Wait 2 ms

Restore the original console path

This timing model is retained as the current working rule and must be validated against the final decoded front-panel bus implementation.

11. UI and Data Storage Rules
Tablet UI
The tablet UI stores no treadmill state locally. The ESP32 sends live JSON via WebSockets at 5-10 Hz, and the browser only renders incoming data.

Primary UI purpose
The web UI is the primary control surface for the user while running.
It must support:

direct adjustment of treadmill speed

direct adjustment of treadmill incline

display of actual speed

display of actual incline

display of treadmill state

display of session time and related live workout data

Because the tablet covers the original console view, actual speed and actual incline must always be available in the web UI whenever the treadmill is operating.

UI Command Queuing Feedback: The web UI allows users to tap speed, incline, or preset buttons during the STARTING phase. These requested target values should be visually indicated as "pending" until the 5-second queue executes them upon reaching the RUNNING state.

Actual vs Target Values
actual speed is derived purely from Pin 7 pulses; actual incline is derived purely from Pin 11 pulses relative to the homed baseline. Target values are displayed only when explicitly commanded by the user via the tablet.

Filesystem, Web UI and OTA Policy
The web UI and filesystem data are product assets and must be versioned with firmware.

Filesystem rules

HTML, CSS, JavaScript, and configuration files shall be stored in LittleFS.

Filesystem contents shall be versioned together with firmware.

A schema version shall be stored with configuration data.

On boot, configuration schema mismatch shall trigger migration or safe fallback to defaults.

OTA rules

firmware OTA and filesystem OTA shall be treated as separate artifacts.

the design guide must define when a filesystem update is mandatory together with firmware.

a failed OTA must not leave the system without a recovery path.

Write discipline

high-frequency runtime values shall not be written synchronously on every update.

persistent writes shall be minimized and batched where possible.

configuration changes should be committed atomically.

Flash Wear Policy
The odometer is only written to flash/NVS when the treadmill state changes to Stopped, minimizing write cycles.

12. Bluetooth FTMS Rules
BLE Stack
Library: NimBLE-Arduino

Chosen for: lower memory footprint and better long-session stability.

Broadcast Rate: exactly 1 Hz

BLE / FTMS Connection Policy
BLE stability is more important than feature count.

Rules

The firmware shall use NimBLE as the BLE stack.

The device shall advertise a stable and unique name (including a short device suffix).

FTMS shall be exposed as a dedicated server role.

The FTMS implementation shall explicitly define readable, notifiable, and writable characteristics.

Any characteristic requiring notify or indicate must include proper CCCD handling.

Connection policy

Only the minimum required number of BLE connections shall be enabled.

If multi-client behavior is not explicitly tested, assume a single primary FTMS consumer.

Reconnect logic must return the device to a clean advertising state after disconnect.

Disconnect events must reset any per-client transient state.

Safety rule
Loss of a BLE client must not stop treadmill telemetry capture or corrupt treadmill state tracking.

Data Formatting
speed is sent as an integer with 0.01 resolution (Example: 12.50 km/h -> 1250)

incline is sent as an integer with 0.1 resolution (Example: 3.5% -> 35)

Control Points
Fitness Machine Control Point requests from training apps must be acknowledged with valid ACK responses.

13. GUI Architecture and Frontend Logic
This section defines the mathematical and functional rules for the tablet interface.

13.1 Layout Engine: The 5-Row Symmetrical Grid
To achieve perfect visual balance regardless of screen resolution, a fixed CSS Grid is utilized. This forces primary telemetry to the mathematical center of the display.

Structure: grid-template-rows: 140px 1fr auto 1fr 140px;

Row 1 (140px): Top Quick Keys (Incline).

Row 2 (1fr): Upper flexible zone (Workout Time).

Row 3 (auto): Dead-center (Telemetry values).

Row 4 (1fr): Lower flexible zone (Speed Presets).

Row 5 (140px): Bottom Quick Keys (Speed).

Visual Offset Adjustment: To compensate for visual weight, Workout Time (Row 2) is offset exactly 15px upwards, and Presets (Row 4) exactly 15px downwards using transform: translateY(). This maximizes the "air" surrounding the centered Row 3 data.

13.2 Smart Telemetry Presentation
Distance Resolution: Distance must be displayed exclusively in kilometers (km) with two decimal places (e.g., 12.57).

Logic: This represents 10-meter resolution. Using three decimal places (single meters) is forbidden as it causes high-frequency digit flickering at high speeds, creating visual stress.

Tabular Numbers: All telemetry fonts must enforce the CSS rule font-variant-numeric: tabular-nums;. This ensures fixed-width digits, preventing telemetry columns from "jittering" horizontally as values change.

Monochrome Heart Rate: The heart rate value must remain white (not red). Colors are reserved for active commands (Blue for incline, Red for speed).

13.3 Interaction Model (Zero-Lag)
Event Handling: Use pointerdown instead of click. This eliminates the built-in delay found in mobile browsers and provides instantaneous response upon touch.

Haptic Feedback (Simulated): Buttons must scale down to 0.92 on press to provide immediate visual confirmation of the command.

Floating Utilities: The System Clock and Setup button reside in an absolute positioned row (approx. 195px from top). This removes them from the grid calculation so they do not interfere with the primary data centering.

13.4 Heart Rate Belt & Bluetooth LE (BLE) Discovery
Device Management: The interface shall not use a simple toggle. It must implement a discovery engine.

Workflow: 1. User initiates "SCAN FOR DEVICES".
2. ESP32 returns a JSON list of available BLE device names and IDs.
3. UI populates a selection list (e.g., "Polar H10", "Garmin HRM").
4. Connection status is persisted in the ESP32 NVS for automatic reconnection in future sessions.

13.5 Technical Frontend Stack
Vanilla JavaScript: Frameworks (React/Vue/etc.) are prohibited to keep memory footprint minimal and ensure the LittleFS partition remains lean.

Communication: Bi-directional data exchange via a single WebSocket stream (5-10 Hz) to avoid HTTP request overhead.

CSS Variable System: All colors and dimensions (e.g., --accent, --danger) are defined as :root variables for rapid global adjustments without logic modification.

13.6 Contextual Logic
P A U S E Status: Status overlays must be absolute positioned below the workout time, ensuring they float without displacing other grid elements.

Incline Homing Display: During the homing process triggered by STARTING, the GUI must lock the incline display to 0.0% until the homing sequence is confirmed complete.

14. Current Open Items
Unmapped harness lines:

Pin 3, Pin 4, Pin 5, Pin 8, Pin 9

Still open in the project:

full mapping of the post-touch front-panel bus

final command injection method on that bus

final incline span calibration from measured 0% -> 15% pulse counting

synchronization with physical console key presses

15. Future Development
Physical Console Key Synchronization
Once the decoded front-panel bus is mapped, the ESP32 will listen for physical button activity using interrupts and debounce filtering.

Observability and Remote Debug
The device should be diagnosable over the network.
The firmware should expose structured logs for:

boot sequence, WiFi state, CSAFE transport state

treadmill and internal UI state transitions

FTMS events, watchdog events, and safety timeouts

command injection attempts/failures

16. Current Project Status
Verified and Usable
actual speed reading from Pin 7

incline movement reading from Pin 11

incline homing to 0% via QUICK START

Console lockout behavior verified (speed/incline keys inactive until running)

CSAFE state polling logic and fragmentation handling

stateless tablet UI architecture

FTMS architecture

Locked 5-row mathematical grid GUI

Chosen and Fixed for Implementation
ESP32-S3 as controller (dual-core task split)

PC817 + 1k resistor on speed input

BSS138 3.3V-safe interface on incline input

HLK-PM01 for permanent power

local web UI on tablet using WebSockets

FTMS via NimBLE

no direct spoofing of raw capacitive panel

Command queuing logic defined for UI inputs during STARTING state

Pin 10 ignored

Planned but Still Open
decoded front-panel bus mapping

final command injection method

synchronization with physical console key presses
