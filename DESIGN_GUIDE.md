# Technical Design Guide

This document contains all verified physical measurements, protocol specifications, and software architecture rules required to operate the StrideControl system safely.

## 1. System Architecture
* **Dual-Core Processing:** To ensure network latency does not interfere with sensor readings, the tasks are strictly divided across the ESP32-S3 cores:
  * **Core 0:** Handles WiFi, the LittleFS web server, WebSocket communication, Bluetooth FTMS, and system logging.
  * **Core 1:** Dedicated entirely to hardware interrupts (Pin 7 and Pin 11 pulse counting), GPIO toggling for the keyboard bus, and safety watchdogs.
* **Hardware Watchdog Timer (WDT):** A 1.0-second hardware watchdog must be enabled. If the main program loop stops responding, the microcontroller will reboot automatically.
* **Fail-Safe MitM Wiring:** The SN74HC4066 analog switch must be wired so its unpowered (default) state allows the original console signals to pass through uninterrupted.
* **System Logging:** Comprehensive logging is required. All state transitions, FTMS connection events, and serial checksum errors must be written to a local buffer for debugging purposes.

## 2. Sensor Data and Calibration
All values are based on physical logic analysis and load testing.

### Speed (Pin 7 - Motor Controller)
* **Signal Type:** 12V pulse (Isolated via PC817).
* **Conversion Factor:** 1.1148.
* **Calculation:** `Speed (km/h) = Frequency (Hz) * 1.1148`.
* **Distance:** 3.2292 pulses correspond to exactly 1 meter.

### Incline (Pin 11 - Elevation Motor)
* **Signal Type:** 5V pulse (Shifted to 3.3V via BSS138).
* **Active Frequency:** Emits a constant **394 Hz** whenever the physical motor is moving.
* **Resolution:** Approximately 1313 pulses equal 1% of incline travel.
* **Mechanical Travel Time:** Measured under user load at 49 seconds (0% to 15%) and 48 seconds (15% to 0%).
* **Homing Sequence:** Initiating a `QUICK START` command forces the incline motor to its physical baseline. The software must monitor the 394 Hz signal; once it drops to 0 Hz, the system calibrates its internal incline tracker to `0.0%`.

## 3. CSAFE Protocol (RS232 Port)
The treadmill mainboard broadcasts its state via an RJ45 port.
* **Connection:** 9600 Baud, 8-N-1 (Using a MAX3232 module).
* **Polling Rate:** 200 to 250 milliseconds.
* **Frame Structure:** The internal controller will drop multi-command frames. You must send two individual frames per cycle:
  1. GoInUse (Keep-alive): `0xF1 0x85 0x85 0xF2`
  2. GetStatus: `0xF1 0x80 0x80 0xF2`
* **Status Decoding:** The active state is found in the first byte following the `0xF1` start byte. Apply a bitmask (`raw_byte & 0x0F`) to extract the value.
* **State Definitions:**
  * `5`: InUse (Motor is running)
  * `6`: Paused
  * `8`: Starting (Countdown sequence)
  * `1, 2, 15`: Stopped / Ready
* **Data Integrity:** The 3.5 HP AC motor generates electromagnetic interference (EMI). Incoming RS232 payloads must be discarded unless the checksum byte is fully verified.

## 4. Timing Rules and Safety Watchdogs
* **Incline Hardware Timeout:** Based on the 49-second mechanical travel time, a hard software limit of **60 seconds** is enforced. If the 394 Hz signal remains active continuously for 60 seconds, the system triggers an emergency stop to prevent motor damage.
* **Start-Up Sync (3.5s Rule):** When the machine enters State 8 (Starting), the console displays a "3-2-1" countdown before the belt moves. The ESP32 must ignore speed and distance accumulation for exactly 3.5 seconds to remain synchronized.
* **Ghost Running Detection:** If the CSAFE state reports `5` (InUse), but the speed sensor on Pin 7 reads `0 Hz` for more than 2.0 seconds, the software assumes a mechanical jam or safety key removal and resets the UI.
* **Injection Sequence:** To simulate a keypress on the 16-pin bus:
  1. Pull SN74HC4066 HIGH (Disconnect console).
  2. Wait 1 millisecond.
  3. Transmit Hex sequence.
  4. Wait 2 milliseconds.
  5. Pull SN74HC4066 LOW (Reconnect console).

## 5. UI and Data Storage
* **Stateless Interface:** The tablet UI does not store variables locally. The ESP32 sends JSON payloads via WebSockets at 5-10 Hz. The UI simply renders the incoming data.
* **Actual vs. Target Values:** "Actual Incline" is calculated strictly by integrating the 394 Hz pulses from Pin 11. "Target Incline" is only displayed when the user inputs a specific command via the tablet.
* **Flash Wear Leveling:** To preserve the ESP32 Flash memory, the total accumulated distance (Odometer) must only be written to NVS/LittleFS when the treadmill transitions into the `Stopped` state. 

## 6. Bluetooth FTMS Integration
* **Library Selection:** The project uses the `NimBLE-Arduino` library. The standard ESP32 BLE stack consumes too much memory and may cause crashes during workouts lasting longer than 40 minutes.
* **Broadcast Rate:** Telemetry data is broadcast exactly once per second (1 Hz). Higher rates will cause buffer overflows in applications like Zwift, resulting in disconnections.
* **Data Formatting:** * Speed: Formatted as an integer with 0.01 resolution (e.g., 12.50 km/h is sent as `1250`).
  * Incline: Formatted as an integer with 0.1 resolution (e.g., 3.5% is sent as `35`).
* **Control Point Acknowledgment:** The ESP32 must listen to the Fitness Machine Control Point and return standard `ACK` responses to all incoming requests from the training application.

## 7. Future Development
* **Physical Button Sniffing:** Once the 16-pin keyboard matrix is mapped with a logic analyzer, the ESP32 will use hardware interrupts and debounce filtering to listen for physical button presses. This will keep the tablet UI synchronized if the user presses the physical +/- keys on the console.
