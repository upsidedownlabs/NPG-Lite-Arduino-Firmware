# EEG/EMG-Based BCI Control for microbots.io ProtoBot

This folder contains firmware for a **brain–computer / muscle–computer interface** using the **NPG-Lite** device to drive a **stock, unmodified microbots.io ProtoBot** over Bluetooth Low Energy (BLE).

The firmware is designed for **research, demos, and educational neuroscience projects**.

---

## 1. How It Works

ProtoBot ships running the **CodeCell-MicroLink** library, which exposes a BLE GATT "joystick" characteristic that the official MicroLink phone app normally writes to in order to drive it.

This firmware turns the NPG-Lite into a **BLE client** that connects directly to that same characteristic and writes joystick values itself — so from ProtoBot's point of view, it looks exactly like the MicroLink app is driving it.

**No changes to the ProtoBot's firmware are required.** The protocol was reverse-derived from the public [CodeCell-MicroLink source](https://github.com/microbotsio/CodeCell-MicroLink):

| Item | Value |
|---|---|
| Service UUID | `12345678-1234-1234-1234-123456789012` |
| Joystick characteristic (write) | `abcd1234-abcd-1234-abcd-123456789012` |
| Settings characteristic (write) | `dcba4330-dcba-4321-dcba-432123456789` |
| Value format | 2 bytes `[X, Y]`, range 0–200, 100 = center/neutral |
| Advertised name | `CodeCell` |

---

## 2. Hardware Overview

### Recording Device (Signal Acquisition)

* **NPG-Lite (ESP32 based)**
* 3 analog input channels:
  * **CH1 (A0)** – EEG (Beta band + blink detection)
  * **CH2 (A1)** – Right-hand EMG
  * **CH3 (A2)** – Left-hand EMG
* Sampling rate: **512 Hz**
* On-board features:
  * NeoPixel LEDs (6 pixels)
  * BLE connectivity (acting as a **client**, not a server)
  * Shared LED + vibration motor on pin 7

### Controlled Device

* **Stock microbots.io ProtoBot** — no firmware modification needed

---

## 3. First-Time Setup

1. Flash the NPG-Lite firmware
2. Power on your ProtoBot (running its normal, unmodified firmware)
3. Open the Serial Monitor at **115200 baud** — the NPG-Lite will automatically scan for and connect to the first ProtoBot/CodeCell device it finds

That's it — no pairing step, no addresses to copy anywhere.

Check current connection state at any time with:

```
status
```

---

## 4. Electrode Placement

### EEG (Channel 1 – A0)

**Purpose:**
* Beta-band power estimation → forward motion control
* Blink detection → infinity-shape trick trigger

**Recommended placement:**
* Active electrode (AOP): **Center forehead**
* Reference (REF): **Behind the right ear**
* Ground (GND): **Behind the left ear**

---

### EMG – Right Hand (Channel 2 – A1)

* Place electrodes over **right forearm flexor muscles**
* Controls:
  * **Right turn**
  * **Backward motion (when combined with left EMG)**

**EMG placement tips:**
* Place electrodes parallel to muscle fibers
* Ensure good skin contact for clean signals

### EMG – Left Hand (Channel 3 – A2)

* Place electrodes over **left forearm flexor muscles**
* Controls:
  * **Left turn**
  * **Backward motion (when combined with right EMG)**

---

---

## 5. Signal Processing Pipeline

### EEG Processing

* 50 Hz Notch Filter (power-line noise removal)
* Low-pass EEG smoothing filter
* FFT (512-point)
* Bandpower extraction:
  * Delta (0.5–4 Hz)
  * Theta (4–8 Hz)
  * Alpha (8–13 Hz)
  * **Beta (13–30 Hz)**
  * Gamma (30–45 Hz)
* Exponential smoothing of bandpower
* Beta power normalized against total power

### Blink Detection

* Dedicated high-pass filter isolates blink transients from the EEG signal
* 100 ms envelope window tracks blink amplitude against `BlinkThreshold`
* **Triple blink** (three blinks within ~1 second, each debounced) triggers the **infinity-shape trick**

### EMG Processing

* 50 Hz Notch Filter
* High-pass filter (70 Hz cutoff)
* Rectification
* Envelope detection (16-sample moving average)

---

## 6. User Controls & Interactions

### 6.1 Forward Movement (EEG – Beta Power)

* **Beta band power > threshold** (default: 10% of total power) → forward, joystick `Y = 0`
* Below threshold → stop, joystick `Y = X = 100` (center)
* Suppressed while backward is active or the infinity shape is running

### 6.2 EMG-Based Turning

| Input | Condition | Action | Joystick |
|---|---|---|---|
| Right EMG (A1) | Envelope > threshold (default 150) | Turn right | `X = 0, Y = 100` |
| Left EMG (A2) | Envelope > threshold (default 150) | Turn left | `X = 200, Y = 100` |

### 6.3 Backward Motion (Dual EMG Activation)

* Both EMG channels active simultaneously (each > 50% of threshold) → backward, joystick `Y = 200`
* Forward command suppressed until backward state clears

### 6.4 Triple Blink → Infinity Shape

* Three blinks in quick succession triggers ProtoBot's **built-in figure-8 ("infinity") drive routine**, run natively by ProtoBot's own firmware
* All other joystick commands **pause** while it runs
* Detected as complete either via a BLE notify from ProtoBot, or a 15-second safety timeout, after which normal control resumes automatically

---

## 7. BLE Control Summary

The NPG-Lite writes directly to ProtoBot's stock characteristics — there's no custom protocol of our own to document beyond what ProtoBot already exposes:

* **Joystick characteristic** — 2-byte `[X, Y]`, refreshed at ~10 Hz while a command is active (a single write-on-change isn't enough; ProtoBot auto-stops if writes stop arriving)
* **Settings characteristic** — full 10-byte packet, used here mainly to set the eye/display color per current command and to keep speed at full
* **Shape characteristic** — triggers ProtoBot's native infinity-shape routine

| Command | Joystick (X, Y) | Eye Color |
|---|---|---|
| Stop | 100, 100 | White |
| Right turn | 200, 100 | Amber |
| Left turn | 0, 100 | Amber |
| Forward | 100, 0 | Green |
| Backward | 100, 200 | Red |
| Infinity shape | (ProtoBot-controlled) | Purple |

> **Efficiency note:** Commands are only re-sent as new writes when the value *changes*; a lightweight periodic refresh keeps the last command alive so ProtoBot's own failsafe doesn't stop it mid-drive.

---

## 8. LED & Haptic Indicators (NPG-Lite side)

### NeoPixel LEDs

| Pixel | Color | Meaning |
|---|---|---|
| 0 | Red | BLE disconnected / scanning |
| 0 | Green | BLE connected to ProtoBot |
| 0 | Blue (brief flash) | Command just sent |
| 5 | Red / Orange / Green | Battery low / medium / high |

### Vibration / LED (Pin 7)

Currently unused in this build (kept in code for future connect/disconnect haptic feedback).

---

## 9. Serial Commands

```
debug                        - start debug output
exit                         - stop debug output
status                       - show connection info
set betathreshold  <n>       - set EEG beta threshold
set emg1threshold  <n>       - set right EMG threshold
set emg2threshold  <n>       - set left EMG threshold
set blinkthreshold <n>       - set blink detection threshold
```

---

## 10. Tuning Thresholds

| Parameter | Variable | Default | Description |
|---|---|---|---|
| EEG Beta threshold | `betaThreshold` | `10` | % of total power required for forward command |
| EMG envelope threshold | `emg1Threshold` / `emg2Threshold` | `150` | ADC counts for single-hand turn command |
| Backward dual threshold | — | `emgThreshold × 0.5` | Each hand must exceed 50% of its EMG threshold |
| Blink threshold | `BlinkThreshold` | `50` | Envelope amplitude required to register a blink |

All thresholds persist across reboots (stored in NVS via `Preferences`).

---

## 11. Intended Use & Disclaimer

**This firmware is intended for research, education, and controlled demonstrations only.**

* Use the ProtoBot in open, safe environments
* Avoid operation near people or fragile objects
* Signal thresholds require per-user calibration — EEG and EMG vary significantly between individuals
* The joystick X/Y direction mapping is inferred from library comments, not an official spec — confirm on the bench before relying on it

---

> Making neuroscience affordable and accessible for everyone
> — **Upside Down Labs**