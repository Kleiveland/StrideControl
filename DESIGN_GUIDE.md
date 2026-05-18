# StrideControl Technical Design Guide (V3.1)

This document contains all verified physical measurements, protocol specifications, architecture rules, and implementation decisions required to operate the StrideControl system (V3.1 Architecture) safely and predictably.

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
| **INCLINE_IN** | GPIO 14 | Pin 11 (via PC817) |

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

To read the speed flawlessly from the treadmill, we use a combination of optical hardware isolation and a non-blocking mathematical software filter to reject signal bounce.

---

## 4.1.1 Hardware Configuration — PC817 Output Stage

The PC817 output is a transistor switch — it cannot produce voltage or current on its own. It can only connect or disconnect two wires. This determines the entire pull-up architecture.

---

### Wiring (Collector Side)

~~~text
3.3V ──── 10kΩ ──┬──── GPIO 3 (SPEED_IN)
                 │
             PC817 Pin 4 (Collector)
             PC817 Pin 3 (Emitter) ──── GND
~~~

---

### Why the Pull-up Resistor is Mandatory

Without it, GPIO 3 is connected to nothing when the PC817 is off. A floating pin acts as an antenna — it picks up WiFi signals, static electricity, and AC motor EMI, producing hundreds of false readings per second.

The 10 kΩ pull-up resistor solves this by acting as a stiff spring holding the door closed:

| PC817 State         | Transistor            | GPIO Voltage                  | ESP32 Reads |
|-------------------|----------------------|-------------------------------|-------------|
| OFF (no magnet)   | Open                 | Pulled to 3.3V by resistor   | Stable HIGH |
| ON (magnet present)| Closed (path to GND) | Collapses to 0V              | LOW         |

**Why GND always wins:**  
Current takes the path of least resistance. GND has 0 Ω, the pull-up has 10,000 Ω. All voltage collapses to 0V the instant the transistor closes.

---

### The Slow-Rise Side Effect & Bouncing

When the PC817 switches off, the 10 kΩ resistor must pull the voltage back up to 3.3V. This rise is not instant — during the transition, the signal bounces across the ESP32's logic threshold.

This slow-rise behavior can cause the signal to cross the ESP32 logic threshold multiple times. These threshold crossings produce additional digital edges ("bounce") in the microsecond range, especially at low speeds where the signal remains near the threshold for longer periods.

~~~text
Voltage
 3.3V ─────────────╮          ╭─ bouncing ─╮          ╭──────────
                   │          │            │          │
                   ╰──────────╯            ╰──────────╯
                   ↑                       ↑
               Magnet in               Magnet out
               (clean LOW)         (slow rise + bounce)
~~~

---

### Verified Circuit Parameters

| Sensor | Source Voltage | Series Resistor (LED Side) | Forward Current | Pull-up (Collector Side) |
|--------|--------------|-----------------------------|----------------|--------------------------|
| Speed (Pin 7) | 11.4V | 10 kΩ | ~1.1 mA | 10 kΩ to 3.3V |
| Incline (Pin 11) | 4.68V | 1 kΩ | ~3.8 mA | 10 kΩ to 3.3V |

---

### Complete Circuit (Both Sensors)

**Speed (Pin 7 → GPIO 3):**

~~~text
Pin 7 (11.4V) ──── 10kΩ ──── PC817-1 Anode
                             PC817-1 Katode ──── GND

3.3V ──── 10kΩ ──┬──── GPIO 3
                 │
             PC817-1 Collector
             PC817-1 Emitter ──── GND
~~~

---

**Incline (Pin 11 → GPIO 14):**

~~~text
Pin 11 (4.68V) ──── 1kΩ ──── PC817-2 Anode
                             PC817-2 Katode ──── GND

3.3V ──── 10kΩ ──┬──── GPIO 14
                 │
             PC817-2 Collector
             PC817-2 Emitter ──── GND
~~~

---

## 4.1.2 Software Filtering (Mathematical Glitch Rejection)

When the ESP32 detects the signal going LOW, a real magnet will keep the signal down for a duration (typically in the millisecond range, ~2–5 ms at 10–25 km/h based on verified measurements).

However, electrical bouncing and motor noise occur on the microsecond scale and may produce multiple rapid transitions around the threshold during both entry and exit of the magnet. Furthermore, at extremely low speeds (e.g., 1 km/h), the exit bounce and threshold oscillation can occur several milliseconds after the initial trigger.

Because the ESP32 must also monitor the fast-paced O1/O2 communication bus, we cannot use blocking delays (`delayMicroseconds()`) inside the Interrupt Service Routine (ISR) to filter this noise.

---

### Two-Stage Non-Blocking Filter

The system applies a two-stage, non-blocking time filter:

#### Micro-glitch rejection (~300 µs)
Extremely short intervals are discarded immediately. These represent electrical noise and threshold bounce, not real rotations.

#### Double-edge lockout (≈2 ms)
To prevent additional edges (including slower exit bounce and threshold oscillation) from being counted as new physical rotations, any edge arriving shortly after a valid pulse is ignored.

The exact value (typically 1–3 ms) is chosen to:
- be significantly larger than observed microsecond-scale bounce
- but much smaller than the minimum physical pulse interval (~45 ms at maximum speed)

---

### Verified Code (Non-Blocking Speed ISR)

~~~cpp
volatile unsigned long isr_lastPulseUs = 0;
volatile unsigned long isr_intervalUs  = 0;
volatile bool isr_newPulse = false;

// Stage 1: Reject threshold bounce and motor noise
const unsigned long GLITCH_REJECT_US = 300;

// Stage 2: Prevent threshold oscillation and slow exit-bounce
const unsigned long LOCKOUT_US = 2000;

void IRAM_ATTR isrSpeed() {
    unsigned long now = micros();
    unsigned long delta = now - isr_lastPulseUs;

    // Reject micro-glitches and most exit-bounce edges.
    // Remaining robustness relies on appropriate LOCKOUT_US tuning.
    if (delta < GLITCH_REJECT_US) return;
    if (delta < LOCKOUT_US) return;

    // Valid physical rotation detected
    isr_intervalUs  = delta;
    isr_lastPulseUs = now;
    isr_newPulse    = true;
}
~~~

---

### Result

This mathematical approach ensures that, under all tested conditions, one physical rotation produces a single registered pulse, providing stable readings from **1 km/h to 25 km/h** while leaving the CPU fully available for the O1/O2 bus logic.
**Verified code (Speed ISR):**

```cpp
void IRAM_ATTR isrSpeed() {
    // 1. Swallow bouncing from the 10k pull-up resistor and motor noise
    delayMicroseconds(500); 
    if (digitalRead(PIN_SPEED) == HIGH) return; // Just noise, abort!

    // 2. Timestamp the actual pulse
    unsigned long now = micros();
    unsigned long delta = now - isr_lastPulseUs;

    // 3. Allow max 50 pulses per second (20ms lockout)
    if (delta > 20000UL) {
        isr_intervalUs  = delta;
        isr_lastPulseUs = now;
        isr_newPulse    = true;
    }
}
