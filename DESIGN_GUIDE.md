# Hardware & Protocol Notes

This is the master reference for the reverse-engineered hardware. It contains the exact timings, pinouts, and logic rules needed to interface with the Sportsmaster T610 and Runfit 99 without blowing anything up.

## 1. Safety & Architecture
If the ESP crashes while you are running at 20 km/h, someone gets hurt. The system is built around these rules:
* **Dual-Core Split:** Core 1 handles hardware interrupts and MitM injection. Core 0 handles WiFi, WebSockets, and Bluetooth. Network lag must never delay a motor interrupt.
* **Fail-Safe MitM:** The SN74HC4066 switch must be wired so its unpowered state lets the original keyboard signals pass through. The original panel is the ultimate hardware backup.
* **Hardware Watchdog:** Enabled by default. If the main loop hangs for > 1 second, the MCU reboots.
* **Incline Timeout:** We measured full incline travel at 49 seconds under load. The software enforces a hard 60-second limit. If the incline motor runs longer than that, the system triggers an emergency stop.

## 2. Sensor Math (Verified)
Tested with a multimeter and logic analyzer directly on the controller board.

### Speed (Pin 7)
* **Signal:** 12V pulse (Use PC817).
* **Math:** `Speed (km/h) = Hz * 1.1148`.
* **Distance:** 3.2292 pulses = 1 meter.

### Incline (Pin 11)
* **Signal:** 5V pulse (Use BSS138).
* **Frequency:** Outputs a constant **394 Hz** whenever the motor is moving.
* **Resolution:** roughly 1313 pulses per 1% of incline.
* **Homing:** Pressing `QUICK START` forces the incline to the bottom. The ESP32 waits for the 394 Hz signal to drop to 0, then locks its internal tracker to `0.0%`.

## 3. Protocol Quirks & Delays
* **The 3.5s Blind Spot:** When the machine enters `Starting` state, the original display runs a "3-2-1" countdown. The ESP32 must ignore all speed/distance data for exactly 3.5 seconds to stay synced with the hardware.
* **Ghost Running:** If the CSAFE port reports the machine is running, but Pin 7 reads `0 Hz` for more than 2 seconds, the belt is jammed or the safety key was pulled. Trigger UI reset.
* **Injection Timing:** To simulate a button press: Pull SN74HC4066 HIGH (1ms wait) -> Send Hex sequence -> (2ms wait) -> Pull LOW.

## 4. Bluetooth FTMS (Zwift)
Zwift is extremely picky about FTMS data. 
* **Use NimBLE:** The standard Arduino BLE stack leaks memory and drops connections after 40 minutes. `NimBLE-Arduino` is mandatory.
* **1 Hz Limit:** Do not broadcast data faster than once a second. Zwift's buffer will overflow and disconnect you.
* **Formatting:** Speed is an integer (`1250` = 12.50 km/h). Incline is `35` for 3.5%.

## 5. CSAFE (RS232 Port)
The RJ45 port spits out standard CSAFE at 9600 Baud, 8-N-1.
* The DC motor creates massive EMI. Treat all incoming RS232 data as garbage unless the checksum is 100% verified.
* Send two separate frames every 250ms (GoInUse and GetStatus). Merging them into one frame crashes the treadmill's serial controller.
