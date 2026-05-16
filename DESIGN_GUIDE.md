# StrideControl Technical Design Guide (V3.0)

This document contains all verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system (V3.0 Architecture) safely and predictably.

> **Primary UI Goal**
> StrideControl provides a clean, tablet-based web interface for direct speed and incline control. Because the tablet completely covers the original console, the UI must display the treadmill's **actual speed** and **actual incline** derived from post-EMA hardware feedback, not just requested target values.

---

## 1. Project Goal

The objective is to build a safe, reversible ESP32-S3 interface layer for the Sportsmaster T610 / Runfit 99 treadmill that:
* Hosts a local web interface for a tablet.
* Allows direct user control of speed and incline.
* Reads actual physical state directly from the hardware.
* Parses machine state from the CSAFE port.
* Broadcasts telemetry via Bluetooth FTMS.
* Preserves original hardware safety features (the console must work if the ESP32 loses power).

This is a **non-destructive overlay system**, not a replacement for the main motor controller.

---

## 2. System Scope & Architecture

* **Role Split:** The ESP32 handles the UI, telemetry, FTMS, and command injection. The treadmill's mainboard retains full control over the 3.5 HP AC drive motor, incline motor power, and safety logic (e-stop key).
* **Core Design:** The web UI is stateless. Physical console buttons are not spoofed at the capacitive level; instead, command injection happens on the decoded 5V logic bus after the touch-controller IC.

---

## 3. Protocol & Hardware Mapping

### 3.1 Harness Measurements (Verified)
* **Black Wire:** +11.75V DC (Constant)
* **Brown Wire:** 0V (GND / Reference)
* **Pin 6:** 5.0V DC (Constant)
* **Pin 12:** 5.0V DC (Constant)

**Pins with dynamic behavior (Telemetry):**
* **Pin 7 (Speed):** 11.4V DC at rest. Emits 8.97 Hz at 10 km/h and 22.3 Hz at 25 km/h.
* **Pin 11 (Incline):** 4.682V DC at rest. Outputs a constant 394 Hz while the incline motor is physically moving. Returns to rest voltage when motion stops.
* **Pin 10:** 0.09V DC at rest. Not used in the current design.

### 3.2 Front-Panel Bus Mapping
Console communication uses time-division multiplexing across two 5V logic buses:
* **O1 Bus (Scanner):** Pins 3, 4, 5, 6, 7, 8. Line 4 acts as a shared active line, pulled LOW during ROW_A through ROW_D scans. Scan rate: ~167 Hz per row, full cycle ~6 ms.
* **O2 Bus (Data/Inject):** Pins 9 through 16. An 8-bit parallel data bus used to return key presses during specific O1 scan windows. Frame rate: ~330 Hz.

**O1 Row Scan Sequence:** ROW_B → ROW_C → ROW_D → ROW_E → ROW_A → IDLE → repeat

| Row | Lines LOW | O1 Pattern |
|---|---|---|
| ROW_A | 3 + 4 | 0x3C |
| ROW_B | 4 + 5 | 0x39 |
| ROW_C | 4 + 6 | 0x35 |
| ROW_D | 4 + 7 | 0x2D |
| ROW_E | 8 only | 0x1F |
| IDLE | none | 0x3F |

### 3.3 GPIO Mapping (ESP32-S3 N16R8)

| Signal | ESP32 GPIO | Connection Target |
|---|---|---|
| **PIN_ROW_A** | GPIO 4 | O1 Line 3+4 (ROW_A) |
| **PIN_ROW_B** | GPIO 5 | O1 Line 4+5 (ROW_B) |
| **PIN_ROW_C** | GPIO 6 | O1 Line 4+6 (ROW_C) |
| **PIN_ROW_D** | GPIO 7 | O1 Line 4+7 (ROW_D) |
| **PIN_ROW_E** | GPIO 15 | O1 Line 8 (ROW_E) |
| **O2_BUS (0-7)** | 41, 42, 8, 9, 10, 11, 12, 13 | O2 Lines 9-16 |
| **MUTE_PIN** | GPIO 21 | Controls 74HC4066 gates |
| **TXS_OE (×2)** | 3.3V or dedicated GPIO | TXS0108E Output Enable — must be HIGH at all times |
| **CSAFE_RX/TX** | GPIO 16 / 17 | MAX3232 Interface |
| **SPEED_IN** | GPIO 3 | Pin 7 (via PC817) |
| **INCLINE_IN** | GPIO 14 | Pin 11 (via BSS138) |

> **Note:** `GPIO.out_w1ts` / `GPIO.out_w1tc` control GPIO 0–31 only. GPIO 41 and 42 (O2 bits 0–1) must be written via `GPIO.out1_w1ts.val` / `GPIO.out1_w1tc.val`. This split must be handled in `writeO2Fast()`.

> **TXS0108E OE pin:** The TXS0108E is passive (all channels high-impedance) until OE is driven HIGH. If OE is left floating or pulled LOW, O1 and O2 communication will silently fail with no error indication. Tie OE directly to 3.3V, or drive via a GPIO set HIGH in `setup()` before any interrupt is attached.

### 3.4 Frame Injection Protocol

Each button press is uniquely identified by the combination of its KEY byte, the specific frame sequence (O2), and the active O1 row pattern.

* **Sync Pulse:** 0xFF (all O2 lines HIGH) terminates every frame at ~330 Hz.
* **Injection Window:** Injection occurs strictly within the ~3 ms window triggered by a hardware interrupt detecting the correct O1 row FALLING edge.
* **Mute Duration:** The 74HC4066 gate is opened (MUTE_PIN HIGH) for the duration of one frame only (~3 ms), then immediately released.

### 3.5 Console Button Behavior (Verified)

Physical Speed (+/−) and Incline (+/−) buttons are **ignored** by the treadmill mainboard unless the belt is actively moving. It is not possible to pre-set a target speed or incline via the console before starting. This constraint also applies to injected commands — do not queue Speed/Incline injections before the RUNNING state is confirmed.

### 3.6 Mechanical Incline Behavior

* A QUICK START command always drives the incline physically to 0% as a homing sequence.
* Measured travel time under user load: 0% → 15% = 49 seconds, 15% → 0% = 48 seconds.
* This is used as the incline homing baseline in software.

---

## 4. Sensor Interpretation

### 4.1 Speed (Pin 7)

For å lese farten feilfritt fra tredemølla, bruker vi en kombinasjon av optisk maskinvare-isolasjon og et "Confirm and Discard"-filter i programvaren.

#### 1. Maskinvaren og problemet ("The Slow Rise")
Vi bruker en PC817 optokobler for å skille ESP32-ens 3.3V-logikk fra tredemøllas 5V-logikk og motorstøy.
* Når magneten treffer sensoren, slår PC817 seg på og trekker pinnen knallhardt ned til jord (0V). Dette gir et perfekt, rent signal inn.
* Når magneten forlater sensoren, slår PC817 seg av. Nå må en 10k pull-up motstand trekke spenningen tilbake opp til 3.3V. Fordi spenningen må dras opp gjennom en motstand, stiger den litt sakte.

**Problemet:** Akkurat i det denne sakte stigende spenningen krysser ESP32-ens grense for hva som er HØY og LAV, oppstår det mikroskopisk elektrisk dirring. ESP32-en er så rask at den leser denne dirringen som helt nye, falske pulser.

#### 2. Programvaren ("Confirm and Discard")
I stedet for å bygge komplisert matematikk for å ignorere disse falske pulsene, løser vi det med en brutal og enkel sjekk i selve avlesningen (ISR):
Når ESP32-en kjenner at signalet går LAVT, vet vi at en ekte magnet vil holde signalet nede i minst 40 millisekunder. Elektrisk dirring og støy fra motoren varer bare i noen få mikrosekunder. Derfor gjør vi følgende:
1. Pinnen går LAV → Avlesningen starter.
2. Vi tvinger koden til å vente i 500 mikrosekunder (`delayMicroseconds`).
3. Sjekk pinnen på nytt: Er den fortsatt LAV? Da er det den ekte magneten. Har den sprettet tilbake til HØY? Da var det bare støy/dirring fra pull-up motstanden, og vi kaster pulsen i søpla.

**Verifisert kode (Speed ISR):**

```cpp
void IRAM_ATTR isrSpeed() {
    // 1. Svelger dirring fra 10k pull-up motstanden og motorstøy
    delayMicroseconds(500); 
    if (digitalRead(PIN_SPEED) == HIGH) return; // Var bare støy, avbryt!

    // 2. Tidsregistrering for den ekte pulsen
    unsigned long now = micros();
    unsigned long delta = now - isr_lastPulseUs;

    // 3. Tillater maks 50 pulser i sekundet (20ms sperre)
    if (delta > 20000UL) {
        isr_intervalUs  = delta;
        isr_lastPulseUs = now;
        isr_newPulse    = true;
    }
}
```

### 4.2 Incline (Pin 11)

Read via a BSS138 level shifter. Calculated by counting pulses relative to a 0% homed baseline. The pulse count is committed to **NVS** every time physical movement stops, ensuring accurate position recovery across reboots.

> **NVS validity assumption:** The NVS pulse count is trusted on boot without physical verification. If the user observes incline position drift over time (e.g., after manual adjustment while the system was off), a manual re-home via QUICK START resets the count to 0. A "Re-home incline" button should be exposed in the configuration UI.

### 4.3 Cadence (LSM6DSOX)

An LSM6DSOX IMU is mounted to the lower treadmill frame via I2C. It uses the built-in hardware step counter to derive cadence from footstrike shockwaves. The I2C bus is actively terminated via an LTC4311 extender to handle the 2-meter cable run from the frame to the motor compartment.

* **Cabling:** Shielded repurposed USB cable to protect against 3.5 HP motor EMI.
* **Connectors:** Wago 221 vibration-proof connectors at the frame splice; GX12 Aviation plug at the ESP32 enclosure for modularity.
* **Graceful Degradation:** If the sensor disconnects or the I2C bus faults, cadence defaults to 0 SPM. This must not trigger a kernel panic or interrupt Core 1 motor pulse counting.
* Polled asynchronously to prevent blocking Core 1 interrupts.

---

## 5. Calibration Constants

* **Speed:** `km/h = Hz * 1.1148` (Anchored at 10 km/h = 8.97 Hz).
* **Odometer:** `1 meter = 3.2292 pulses`.
* **Incline:** **[BLOCKER]** Final pulses-per-percent constant is pending a physical 0–15% sweep validation. A calibration screen in AP-mode should be planned to allow re-calibration without reflashing.

---

## 6. Hardware Interface Choices

### 6.1 Mute Circuit (74HC4066)

* **Normal State:** MUTE_PIN is **LOW**. The bilateral switches are closed, allowing native console communication.
* **Injection State:** MUTE_PIN is **HIGH**. The switches open, isolating the console while the ESP32 drives the O2 bus.
* **Fail-Safe:** A 10 kΩ **pull-down** resistor on MUTE_PIN ensures the hardware defaults to normal console operation if the ESP32 crashes or loses power.
* **Settle Time:** Allow a minimum of 100 µs after asserting MUTE_PIN HIGH before writing to the O2 bus. Measure actual settle time with a logic analyzer on the first hardware bring-up and reduce if margins allow.

### 6.2 Modular PCB Architecture

* **Board 1 (Interceptor Shield):** Inline with the 16-pin ribbon cable. Houses the SN74HC4066N switches and fail-safe pull-down logic. Kept strictly separate from high-voltage/motor noise.
* **Board 2 (Core Brain):** Houses the ESP32-S3, HLK-PM01 PSU (AC–5V, fused on mains side), TXS0108E logic level converters (×2, OE pins tied HIGH), and optical isolators (PC817, BSS138).
* Boards are connected via soft stranded silicone wire to withstand mechanical vibration.

### 6.3 Power Supply

* HLK-PM01 AC-to-5V module provides permanent internal power without relying on external USB.
* Total draw budget: ESP32-S3 with active WiFi + BLE peaks at ~500 mA. HLK-PM01 is rated at 600 mA. Margin is tight — verify under full operational load before final assembly.

### 6.4 Bootstrap, Recovery and Local Configuration

The device must be deployable and recoverable without requiring a USB cable. First boot, or missing WiFi credentials, forces Access Point (AP) mode to expose a local configuration page for network setup, machine profile, FTMS name toggles, and incline calibration.

---

## 7. Software Architecture

### 7.1 Dual-Core Task Split

* **Core 0:** WiFi, Async Web Server, WebSockets, Bluetooth FTMS, LittleFS.
* **Core 1:** Hard real-time tasks: pulse counting ISRs (Pin 7, Pin 11), hardware command injection, safety watchdogs.
* All shared memory structures must be protected by a `portMUX_TYPE` spinlock (`portENTER_CRITICAL` / `portEXIT_CRITICAL`).

> **Known conflict:** NimBLE and AsyncWebServer share Core 0 WiFi resources. During peak load (simultaneous WebSocket burst + BLE notify), heap fragmentation spikes are observed. Mitigation: enforce a strict WebSocket send queue with backpressure — never send a new frame before the previous ACK is confirmed.

### 7.2 Memory & Storage

* **Hardware:** The N16R8 profile (16MB Flash, 8MB PSRAM) is strictly required to prevent heap fragmentation from the dual-role BLE stack, WebSockets, and ArduinoJson over time.
* **Serialization:** Handled deterministically using ArduinoJson with statically allocated buffers. Dynamic allocation for telemetry payloads is prohibited.
* **Partitions:** A custom CSV partition table is required, allocating at minimum 1.8 MB for `app0` and a dedicated LittleFS partition for web assets and `profiles.json`.
* **OTA Rules:** Firmware OTA and filesystem (LittleFS) OTA must be handled as separate artifacts to prevent overwriting `profiles.json` and calibration data during logic updates.
* **Flash Wear Policy:** Odometer and incline NVS pulse count are written to flash only when treadmill state transitions to STOPPED. Never write on every pulse.
* **Atomic write pattern for profiles.json:** Write to `/profiles.json.tmp`, verify JSON parse succeeds, then rename to `/profiles.json`. On boot, if `.tmp` exists but `.json` is absent or corrupt, restore from `.tmp`. This prevents data loss on power failure mid-write.

### 7.3 Injection Logic

Injection strictly occurs within a ~3 ms window triggered by a hardware interrupt (FALLING edge) on the correct O1 row pin. The ISR wakes the `injectionTask` on Core 1 via `vTaskNotifyGiveFromISR`. The task then:

1. Asserts MUTE_PIN HIGH and waits 100 µs settle time.
2. Writes the frame byte sequence to O2 via `writeO2Fast()`.
3. Waits 50 µs, then releases MUTE_PIN LOW.

**Fractional Speed Targeting (Nearest Integer Optimization):**
To minimize total injection time and UI latency, the system uses `round()` to find the nearest whole-number base speed, then adjusts by discrete Speed +/− increments.

* Example: Target 12.6 km/h → inject 13 via InstantSpeed macro, then 4× Speed−.
* Example: Target 12.4 km/h → inject 12 via InstantSpeed macro, then 4× Speed+.
* Boundary clamping is required: `round()` can produce values outside 1–20 km/h range at the extremes. Clamp after rounding before injecting.

**FTMS Speed Target Rate Limiting:**
A full InstantSpeed sequence (7–8 frames × ~3 ms ≈ 25 ms total) must not be re-triggered while already in progress. Enforce a minimum 500 ms cooldown between complete InstantSpeed sequences when driven by FTMS Control Point inputs. Ignore new targets received during an active injection sequence.

**FTMS Elevation Gain:**
The FTMS spec requires Positive Elevation Gain, which the hardware does not natively provide. Accumulate algorithmically per tick:
```text
elevation_gain += distance_interval_km * 1000 * (incline_pct / 100)
```

**Injection Debug Log:**
Maintain a circular log buffer of 64 entries. Each entry records: timestamp, KEY byte, O1 row ID, frame type, and injection result (success/timeout). Essential for diagnosing timing issues during hardware bring-up.

### 7.4 Telemetry Rate Control and Backpressure

Real-time pulse capture, WebSocket rendering, CSAFE parsing, and FTMS broadcasting must not compete for timing. Hardware pulse counting is interrupt-driven; no UI work occurs inside ISRs. Downstream consumers read from internal state at fixed rates: **WebSocket at 5–10 Hz, FTMS at 1 Hz.**

### 7.5 Configuration Classes

Persistent values are separated into four classes to prevent runtime state from being accidentally treated as calibration data:

1. **Hardware calibration:** Speed factor, pulses per meter, incline span, homing parameters.
2. **Device preferences:** WiFi credentials, FTMS device name, debug mode toggle.
3. **User profile settings:** HVILE/DRAG speed presets, last-used interval configuration (work/rest durations, reps, series).
4. **Runtime-only state:** Raw counters, current treadmill state machine value. Must never be written to NVS.

---

## 8. CSAFE Rules (RS232)

* **Connection:** RJ45 to MAX3232 (9600 baud, 8-N-1).
* **Polling:** Split into Keep-alive (`0xF1 0x85 0x85 0xF2`) and Status request (`0xF1 0x80 0x80 0xF2`) at 200–250 ms intervals.
* **Forbidden Commands:** `0xAA`, `0xA5`, and `0x9C` must never be sent. These commands cause confirmed buffer corruption on the DK-City mainboard and the behavior is unrecoverable without a power cycle.
* **State Decoding:** Raw CSAFE states map strictly to internal `SystemState` enums: `STOPPED`, `PAUSED`, `STARTING`, `RUNNING`, `IDLE`.
* **Transport Contract:** Application code must never read directly from UART. CSAFE is handled as a dedicated transport/parser layer.

---

## 9. Timing & Safety Rules

* **3.5s Startup Sync:** The ESP32 ignores speed/distance telemetry for 3.5 seconds after transitioning to `STARTING` to align with the console's physical countdown beeps. This prevents false odometer accumulation during motor spin-up.
* **700 ms Resume Debounce:** A 700 ms debounce filter applies when transitioning from `PAUSED` to `RUNNING` to prevent double-triggering from brief state fluctuations.
* **1500 ms CSAFE Watchdog:** If no valid `0xF2` CSAFE frame is received for 1500 ms, internal state is forced to `STOPPED`. This is the primary detection method for physical e-stop key removal.
* **5-Second Command Queue Expiry:** Commands issued during the `STARTING` state are queued and injected once `RUNNING` is confirmed. If `RUNNING` is not reached within 5 seconds, the queued commands are discarded.
* **Incline Timeout:** Maximum continuous incline motion is capped at 60 seconds. Exceeding this triggers a fault state.
* **Ghost-Running Detection:** If CSAFE reports `InUse` but Pin 7 reads 0 Hz for 2.0 seconds continuously, the UI resets and logs a mismatch fault. This handles the case where the safety key is removed while the belt is coasting.
* **Hardware Watchdog:** A 1.0-second hardware WDT must be enabled. The injection task feeds the WDT via `esp_task_wdt_reset()` after each successful frame injection. During `IDLE`/`STOPPED` (no O1 interrupts firing), a dedicated keepalive task on Core 1 feeds the WDT every 500 ms to prevent spurious reboots during standby.

---

## 10. Bluetooth FTMS Rules

* **BLE Stack:** NimBLE-Arduino (Dual-Role operation required).
* **Broadcast Rate:** Exactly **1 Hz**.
* **MTU Negotiation:** Request MTU of 64 bytes during BLE connection setup (`NimBLEDevice::setMTU(64)`). The full FTMS `treadmill_data` characteristic is 34 bytes — without MTU negotiation it splits across two BLE packets, adding 40–60 ms latency per broadcast.
* **Data Formatting:** Telemetry payloads strictly adhere to the Bluetooth SIG FTMS specifications (`org.bluetooth.characteristic.treadmill_data`).
  * Speed is sent as an integer with 0.01 resolution (e.g., 12.50 km/h = 1250).
  * Incline is sent as an integer with 0.1 resolution (e.g., 3.5% = 35).
  * Cadence (from the LSM6DSOX) is baked natively into the FTMS payload.
* **Connection Policy:** FTMS exposed as a dedicated server. Disconnect events reset transient state. Loss of a BLE client must not corrupt treadmill state tracking.
* **Control Points:** External target requests (e.g., from Zwift) route through the internal macro queue system, functioning identically to local UI inputs. A minimum 500 ms cooldown between complete InstantSpeed sequences applies — see Section 7.3.

---

## 11. GUI Architecture & Frontend Rules

* **Stack:** Vanilla JavaScript only. Frameworks (React, Vue, etc.) are prohibited — files must fit within the LittleFS partition and minimize browser memory footprint.
* **Layout:** 5-row symmetrical CSS Grid. Primary telemetry is anchored to the mathematical center.
* **Interaction:** `pointerdown` events are used instead of `click` to bypass mobile browser tap delay (~300 ms). CSS brightness filters provide haptic visual feedback.
* **Telemetry Update Rate:** WebSocket push rate from ESP32 is strictly regulated to **5–10 Hz**.
* **Rendering Constraints:** Non-ASCII characters (e.g., 'Ø', 'Å') are strictly avoided in all UI copy to prevent rendering artifacts in older Android browsers. Use "AVBRYT" instead of "KLARGJØR ØKT", etc.
* **Contextual Locking:** Setup selectors and user profile pickers are disabled during `STARTING` and `RUNNING` states.
* **WebSocket Reconnect Policy:** The tablet UI must implement automatic reconnect with exponential backoff (starting at 1s, max 10s). A visible "DISCONNECTED" overlay must be displayed whenever the WebSocket is not in `OPEN` state. All displayed telemetry values must be grayed out or cleared when disconnected to prevent stale data being interpreted as live.
* **BLE Heart Rate Discovery:** The UI implements a BLE device discovery flow. The ESP32 scans for nearby HR monitors, returns a JSON list, and upon selection acts as an **HR Proxy** — forwarding pulse data natively within the FTMS broadcast. The selected device MAC is persisted in NVS for automatic reconnection.

### 11.1 Quick Keys and Macro Presets
To ensure zero-latency control, eliminate the need for virtual keyboards, and avoid continuous physical button holding, the UI relies exclusively on large, static touch targets strategically placed at the outer edges of the screen.

* **Incline Grid (Top Row):** The top edge of the screen consists of an 8-button symmetrical grid mapping to the most frequently used incline levels (0 through 12). To provide immediate visual grouping, these buttons feature a distinct **blue bottom border** (`var(--accent)`).
* **Speed Grid (Bottom Row):** The bottom edge of the screen mirrors this layout with an 8-button grid for target speeds (4 through 18 km/h). These buttons feature a **red bottom border** (`var(--danger)`).
* **Personalized Macro Presets (HVILE / DRAG):** Positioned directly below the central telemetry zone, these two dominant action buttons allow single-tap transitions between recovery and target intensity. The absolute speed values are dynamically populated from the active user's `presets` in `profiles.json`. 
  * The **HVILE** (Rest) button is styled with a **neutral gray border** (`var(--text-dim)`).
  * The **DRAG** (Work) button is styled with a **red border** (`var(--danger)`), matching the main speed grid.
* **Persistent Availability:** To guarantee user safety and predictable manual override capabilities, the Quick Key grids and Macro Presets remain active and fully functional at all times, including when the Interval Coach is actively running in `focus-mode`. Pressing any of these keys queues an immediate hardware injection sequence for that specific target.
  
---

## 12. Pro Interval Coach (Visual Engine)

* **No Automated Control:** The interval engine is strictly visual and auditory. It tracks phases (Work, Rest, Cooldown) and prompts the user but **never** injects speed or incline commands automatically. The user remains in full control via the persistent HVILE and DRAG macro buttons.
* **Focus Mode:** When a workout begins, the DOM transitions to `focus-mode` via CSS class toggle. Primary telemetry shrinks to the periphery; the countdown timer scales to the mathematical center.
* **Dynamic Workout Graph:** A CSS flexbox graph generates a visual timeline of the full session (including nested series and series-rests) for peripheral progression feedback.
* **Web Audio API Cues:** Synthetic oscillator beeps (3-2-1 countdowns, phase-shift alerts) are generated via the native browser Web Audio API. No external `.mp3` dependencies.
* **Emergency Controls During Active Intervals:**
  * **Lifebuoy (+30s):** Dynamically extends the current rest phase without breaking the workout structure.
  * **Skip Phase (⏭):** Truncates the current phase and jumps immediately to the next block.
* **Signature Workouts:** Predefined hardcoded templates (e.g., "Olympiatoppen 4x4", "Pyramid") populate the workout schedule array dynamically, overriding manual step inputs.
* **Profile Binding:** Interval targets are stored in `profiles.json` and dynamically loaded based on the active user. When a user is selected, the interval modal pre-populates with their last-used settings.
* **Psychological & Motivational Overlays:** To reduce cognitive load and provide mental anchoring during high-intensity phases, the engine implements the following deterministic micro-interactions:
  * **ETA Projection:** The UI must continuously calculate and display the absolute finish time (Estimated Time of Arrival, e.g., "Ferdig kl. 19:42") adjacent to the primary timer. This relies on the system clock and the remaining sequence duration, shifting user focus from elapsed duration to a fixed completion milestone.
  * **Phase Split-Banner:** Upon the successful completion of a `work` phase, a UI banner is briefly displayed for exactly 4 seconds (e.g., "Drag 3 fullført"). This provides immediate positive reinforcement without requiring the user to parse the main telemetry grid.
  * **The Halfway Hump:** The engine pre-calculates the 50% threshold of the total scheduled `work` phases. Upon initiating the phase that crosses this median, the UI triggers a distinct visual cue (e.g., "Halvveis!") to shift the user's mental model from counting completed phases to counting remaining phases.
  * **Post-Workout RPE Logging:** Upon entering the `cooldown` phase, the UI intercepts the session termination by prompting the user for a Rate of Perceived Exertion (RPE) score (scale 1–10). This subjective metric is paired with the session's execution data and written to a local log buffer before the UI fully reverts to the standard running state.

---

## 13. Interval Data Structures

### 13.1 User Profile Schema (profiles.json)

To ensure consistent parsing between the web UI and the LittleFS backend, configurations follow a strict JSON schema. The `presets` object dictates the target speeds injected by the HVILE and DRAG macro buttons, while the `quick_keys` arrays dynamically populate the 8-button speed and incline grids. The `history` array functions as a circular buffer (e.g., max 10 entries) to persist post-workout RPE evaluations.

```json
{
  "users": [
    {
      "name": "Kristian",
      "presets": { 
        "hvile": 6.0, 
        "drag": 16.0 
      },
      "quick_keys": {
        "speed": [4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0],
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
      "history": [
        {
          "timestamp": 1715512800,
          "signature": "4515",
          "rpe_score": 8,
          "completed": true
        }
      ]
    }
  ]
}
```

### 13.2 Dynamic Schedule Array (In-Memory Engine)

Regardless of whether the user inputs manual fields or selects a Signature Workout, the UI compiles the workout into a flat sequential array before execution:

```json
[
  { "type": "work", "time": 60,  "name": "DRAG 1 MIN" },
  { "type": "rest", "time": 60,  "name": "PAUSE" },
  { "type": "work", "time": 120, "name": "DRAG 2 MIN" },
  { "type": "rest", "time": 60,  "name": "PAUSE" }
]
```

`time` is always total seconds. `type` dictates both CSS graph coloring (red = work, green = rest) and the Web Audio cue triggered upon entering the phase.

---

## 14. Open Items

| Item | Status | Priority | Owner |
|---|---|---|---|
| Final incline span calibration (pulses-per-percent) | Pending physical 0–15% sweep | **HIGH — blocks accurate incline reporting** | User |
| ~~Fan command verification~~ | ✅ Verified & mapped in Appendix A | Closed | — |
| Num 6 O1 row confirmation | Pending simultaneous O1/O2 capture | Low | User |
| Incline re-home UI in AP config page | Not yet implemented | Medium | User |

---

## 15. Future Development

* **Physical Console Key Synchronization:** Listen for physical button activity on O1/O2 to synchronize the web UI with console inputs in real time.
* **Auto-Incline via FTMS (Two-Way):** Map FTMS Control Point elevation data (e.g., from Zwift) to automatic Incline +/− injections.
* **Observability:** Expose the injection debug log (Section 7.3) and structured boot/transport/FTMS event logs over the network for remote diagnostics.

---

## 16. Current Project Status

### Verified and Implemented
* Actual speed reading from Pin 7 (filtered via EMA/Blanking) and incline reading from Pin 11 (NVS persistent).
* Console lockout behavior verified.
* CSAFE state polling logic and fragmentation handling.
* Stateless tablet UI architecture (locked 5-row mathematical grid).
* HR Proxy functionality and Bluetooth FTMS via NimBLE (Dual-Role).
* Decoded front-panel post-touch bus mapping (O1/O2 buses).
* Final command injection method via 74HC4066N Data Gate and Core 1 ISRs.
* Fractional target math using nearest-integer optimization logic.

### Chosen and Fixed for Implementation
* ESP32-S3 as controller (dual-core task split). Required hardware profile: **N16R8**.
* 100% Optical Isolation for Speed (PC817) and Incline (BSS138).
* I2C Active Termination (LTC4311) for Cadence/Presence tracking (LSM6DSOX).
* 2x TXS0108E logic level translators for O1/O2 buses (OE pins tied HIGH).
* HLK-PM01 for permanent power.
* Deterministic memory handling via statically allocated ArduinoJson buffers.
* Custom Partition Table for dual-role BLE and Web Server stability.
* Algorithmic Positive Elevation Gain calculation for FTMS compliance.
* Atomic write pattern for `profiles.json` to protect against power-loss corruption.
* 64-entry circular injection debug log for hardware bring-up diagnostics.

---

## Appendix A: Verified Button Map

A button is uniquely identified by its **KEY Byte**, **Frame Sequence**, and active **O1 Row**. No two buttons share all three parameters simultaneously.

**Frame sequences** always terminate with 0xFF (sync pulse). Function keys use shorter frames (3–4 bytes). Digit and control keys use longer frames (5–6 bytes). Fan keys use a distinct variant with 0x03 as the third byte, not found in any other group.

| Function | KEY Byte | Frame Sequence (O2) | O1 Row | Pattern | Line |
|---|---|---|---|---|---|
| **Speed +** | 0x23 | 0x01→0x23→0x00→0xFF | ROW_D | 0x2D | 4+7 |
| **Speed −** | 0x13 | 0x01→0x13→0x00→0xFF | ROW_E | 0x1F | 8 |
| **Incline +** | 0x83 | 0x01→0x83→0x00→0xFF | ROW_A | 0x3C | 3+4 |
| **Incline −** | 0x07 | 0x01→0x07→0x00→0xFF | ROW_C | 0x35 | 4+6 |
| **Quick Start** | 0x0B | 0x01→0x0B→0x00→0xFF | ROW_C | 0x35 | 4+6 |
| **Stop** | 0x43 | 0x01→0x43→0x00→0xFF | ROW_C | 0x35 | 4+6 |
| **Enter** | 0x11 | 0x01→0x11→0x01→0x03→0x00→0xFF | ROW_C | 0x35 | 4+6 |
| **Clear** | 0x81 | 0x81→0x01→0x03→0x00→0xFF | ROW_D | 0x2D | 4+7 |
| **Instant Speed** | 0x03 | 0x01→0x03→0x01→0x03→0x00→0xFF | ROW_E | 0x1F | 8 |
| **Instant Incline** | 0x03 | 0x03→0x01→0x03→0x00→0xFF | ROW_D | 0x2D | 4+7 |
| **Num 0** | 0x05 | 0x01→0x05→0x01→0x03→0x00→0xFF | ROW_E | 0x1F | 8 |
| **Num 1** | 0x41 | 0x41→0x01→0x03→0x00→0xFF | ROW_E | 0x1F | 8 |
| **Num 2** | 0x09 | 0x01→0x09→0x01→0x03→0x00→0xFF | ROW_C | 0x35 | 4+6 |
| **Num 3** | 0x21 | 0x01→0x21→0x01→0x03→0x00→0xFF | ROW_C | 0x35 | 4+6 |
| **Num 4** | 0x21 | 0x21→0x01→0x03→0x00→0xFF | ROW_D | 0x2D | 4+7 |
| **Num 5** | 0x09 | 0x09→0x01→0x03→0x00→0xFF | ROW_A | 0x3C | 3+4 |
| **Num 6** | 0x41 | 0x01→0x41→0x01→0x03→0x00→0xFF | ROW_C | 0x35 | 4+6 |
| **Num 7** | 0x11 | 0x11→0x01→0x03→0x00→0xFF | ROW_A | 0x3C | 3+4 |
| **Num 8** | 0x05 | 0x05→0x01→0x03→0x00→0xFF | ROW_E | 0x1F | 8 |
| **Num 9** | 0x81 | 0x01→0x81→0x01→0x03→0x00→0xFF | ROW_D | 0x2D | 4+7 |
| **Fan High** | 0x09 | 0x01→0x09→0x03→0x00→0xFF | IDLE | 0x3F | — |
| **Fan Low** | 0x03 | 0x01→0x03→0x00→0xFF | IDLE | 0x3F | — |
| **Fan On/Off** | 0x05 | 0x01→0x05→0x03→0x00→0xFF | ROW_D | 0x2D | 4+7 |

> *Num 5 (formerly ROW_B) and Num 7 (formerly ROW_D) were revised to ROW_A based on raw log re-analysis: 111/114 and 107/111 synchronized O1 hits respectively.*
>
> *Num 6: O1 row assignment (ROW_C) is based on legacy fasit analysis. Fresh raw analysis was inconclusive due to insufficient recording overlap between O1 and O2 captures. Verify with a new simultaneous capture.*
>
> *Fan Low uses the shortest frame in the entire table (3 bytes before sync). KEY=0x03 is safe because the sequence 0x01→0x03→0x00→0xFF is unique — no other button uses this exact sequence.*
