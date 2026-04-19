# Hardware & Protocol Master Reference

This is the single source of truth for the StrideControl project (Sportsmaster T610 / Runfit 99). It contains every verified timing, pinout, protocol quirk, and logic rule required to make the ESP32-S3 interface work safely with the treadmill's original hardware.

## 1. System Architecture & Safety
Treadmills are dangerous if software fails. The core architecture relies on these strict rules:

* **Dual-Core Isolation:** Network lag must never delay a hardware interrupt. 
  * `Core 0`: Handles WiFi, WebSockets (tablet UI), Bluetooth FTMS, and system logging.
  * `Core 1`: Dedicated entirely to Pin 7/11 hardware interrupts, MitM GPIO toggling, and safety watchdogs.
* **Fail-Safe MitM:** The SN74HC4066 analog switch must be wired so that if the ESP32 loses power, the default state allows the original keyboard signals to pass through. The original panel remains the hard-wired emergency backup.
* **Hardware Watchdog (WDT):** Enabled by default. If the main loop hangs for > 1.0 seconds, the MCU forces a hard reboot.
* **Flash Wear Leveling:** Do not write odometer data to Flash/NVS while running. Save to memory ONLY when the treadmill transitions to the `Stopped` state.
* **System Logging:** Comprehensive logging is a mandatory design principle. All state changes, FTMS drops, and serial checksum failures must be logged to a local buffer for debugging.

## 2. Sensor Math & Logic (Verified)

### Speed (Pin 7 - Motor Controller)
* **Hardware:** 12V pulse. Must be stepped down using a PC817 optocoupler + 1k resistor to protect the ESP32.
* **Math:** `Speed (km/h) = Hz * 1.1148`
* **Distance:** `3.2292` pulses equals exactly 1 meter.

### Incline (Pin 11 - Elevation Motor)
* **Hardware:** 5V pulse. Step down to 3.3V using a BSS138 level shifter.
* **Signal:** Emits a constant **394 Hz** whenever the physical motor is moving.
* **Resolution:** 1313 pulses = 1% of incline travel.
* **Homing Sequence:** Pressing `QUICK START` forces the incline motor to the physical bottom. The ESP32 must wait for the 394 Hz signal to drop to 0, then lock its internal tracker to `0.0%`.
* **Actual vs. Target:** "Actual Incline" is calculated strictly by integrating the 394 Hz pulses. "Target Incline" is only used as a UI reference when a command is injected.

## 3. The CSAFE Protocol (RS232)
The treadmill broadcasts system state via the RJ45 port.
* **Hardware:** 9600 Baud, 8-N-1. Requires a MAX3232 RS232-to-TTL module.
* **Polling:** 200 - 250 ms intervals.
* **Frame Rules:** The treadmill controller is unstable and drops multi-command frames. You must send two separate frames per cycle:
  1. GoInUse (Keep-alive): `0xF1 0x85 0x85 0xF2`
  2. GetStatus: `0xF1 0x80 0x80 0xF2`
* **Decoding:** Apply bitmask (`raw_byte & 0x0F`) to the status byte (first byte after `0xF1`).
* **Known States:**
  * `5`: InUse (Motor running)
  * `6`: Paused
  * `8`: Starting (3-2-1 countdown)
  * `1, 2, 15`: Stopped/Ready
* **EMI Warning:** The massive DC motor creates heavy electromagnetic interference. Treat all incoming RS232 payloads as garbage unless the Checksum/CRC byte is 100% verified.

## 4. Watchdogs & Timing Delays
* **Incline Hardware Timeout:** We measured full incline travel at 49s (up) and 48s (down) under load. The software enforces a hard **60-second limit**. If Pin 11 reads 394 Hz for >60s, trigger emergency stop.
* **The 3.5s Blind Spot:** When transitioning to `State 8` (Starting), the machine does a "3-2-1" countdown before the belt moves. The ESP32 must ignore speed/distance logic for exactly 3.5 seconds to maintain sync.
* **Ghost Running (C7):** If CSAFE reports `State 5` (InUse), but Pin 7 reads `0 Hz` for > 2.0 seconds, the belt is jammed or the safety key was pulled. Trigger UI reset.
* **MitM Injection Timing:** 1. Pull SN74HC4066 HIGH (Disconnect physical panel).
  2. Wait 1 ms.
  3. Inject Hex sequence into bus.
  4. Wait 2 ms.
  5. Pull SN74HC4066 LOW (Reconnect physical panel).

## 5. Bluetooth FTMS (Zwift/Apps)
Zwift is notoriously strict about the FTMS standard.
* **Library:** Use `NimBLE-Arduino`. The standard ESP32 BLE stack leaks memory and will crash after 40+ minutes of running.
* **1 Hz Broadcast Limit:** Send telemetry (Speed/Incline) exactly once per second. Broadcasting faster will overflow the receiver buffer on the tablet/Apple TV and cause disconnects.
* **Data Formatting:** Speed must be sent as an integer with 0.01 resolution (`1250` = 12.50 km/h). Incline uses 0.1 resolution (`35` = 3.5%).
* **Control Points:** The ESP32 must actively listen to the Fitness Machine Control Point and return standard `ACK` responses to app requests, or the app will drop the connection.

## 6. Extended Features (Pending Logic Analyzer)
* **Physical Button Sniffing:** Connect the ESP32 to the 16-pin keyboard bus to listen for physical button presses (using hardware interrupts and software debounce). This will allow the tablet UI to update in real-time when the user presses the physical +/- keys on the treadmill console.
