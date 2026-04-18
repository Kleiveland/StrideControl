# Technical Design Guide

This document defines the physical laws, timing delays, and protocol rules required to interface with the treadmill hardware. These values are based on physical multimeter testing and logic analysis.

## 1. Sensor Calibration & Physics

### Speed (Pin 7)
* Signal: 12V pulse.
* Conversion Factor: 1.1148.
* Formula: Speed (km/h) = Hz * 1.1148.
* Distance: 3.2292 pulses = 1 meter.
* Reference: 10 km/h = 8.97 Hz | 25 km/h = 22.3 Hz.

### Incline (Pin 11)
* Signal: 5V pulse.
* Active Frequency: A constant 394 Hz is emitted when the incline motor is moving.
* Homing Logic: Triggered automatically by the QUICK START command. The system forces the incline to the physical bottom. The ESP32 must wait for the 394 Hz signal to stop before locking the internal calibration to 0.0%.

## 2. CSAFE Protocol (RS232)
The treadmill broadcasts state data via the RJ45 port.
* Parameters: 9600 Baud, 8-N-1.
* Polling Interval: 200 - 250 ms.
* Frame Structure: The treadmill controller drops multi-command frames. Always send two separate frames per cycle:
  1. GoInUse (Keep-alive): 0xF1 0x85 0x85 0xF2
  2. GetStatus: 0xF1 0x80 0x80 0xF2
* Decoding: The status byte is the first byte after 0xF1. Apply bitmask (raw_byte & 0x0F) to filter noise.
* States:
  * 5: InUse (Running)
  * 6: Paused
  * 8: Starting (Countdown)
  * 1, 2, 15: Stopped/Ready

## 3. Timing Rules
* Start-Delay (3.5s Rule): When the machine enters State 8 (Starting), it runs a 3-2-1 countdown on the main display. The ESP32 must ignore speed/distance accumulation for exactly 3.5 seconds.
* Resume Filter: When moving from Paused (6) to InUse (5), wait 700 ms to confirm stability before triggering resume logic.
* MitM Injection Sequence:
  1. Pull SN74HC4066 HIGH (Disconnect physical panel).
  2. Wait 1 ms.
  3. Send Hex sequence to bus.
  4. Wait 2 ms.
  5. Pull SN74HC4066 LOW (Reconnect physical panel).

## 4. Safety Watchdogs
* Ghost Running (C7): If CSAFE reports State 5 (InUse) but the speed sensor on Pin 7 reads 0 Hz for more than 2.0 seconds, trigger a safety reset on the UI.
* Serial Timeout: If no valid 0xF2 byte is received on the RS232 line for 1.5 seconds, force system state to Stopped.

## 5. Software Architecture
To maintain stable hardware interrupts, tasks are strictly divided across the ESP32-S3 dual-core processor:
* Core 0: WiFi, WebSocket JSON broadcasting (every 250ms), and LittleFS web server.
* Core 1: Hardware interrupts (Pin 7 & 11 pulse counting), MitM GPIO toggling, CSAFE polling, and watchdogs.

## Pending Research
* USB Logic Sniffing: Connect logic analyzer to the 16-pin keyboard flat cable to map Hex codes for digits 0-9, ENTER, and instant speed/incline keys.
* Motor Travel Time: Measure total mechanical travel time from 0% to 15% incline to define a maximum hardware timeout limit.
