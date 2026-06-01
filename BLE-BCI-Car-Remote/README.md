# EEG/EMG-Based Car Control Firmware

This folder contains firmware for **brain–computer / muscle–computer interface** using the **NPG-Lite** device.  
The system enables **hands-free control of a robotic car** using **EEG (brain signals)** and **EMG (muscle activity)** over **Bluetooth Low Energy (BLE)**.

The firmware is designed for **research, demos, and educational neuroscience projects**.

---

## 1. Hardware Overview

### Recording Device (Signal Acquisition)

* **NPG-Lite (ESP32 based)**
* 3 analog input channels:
  * **CH1 (A0)** – EEG (Beta band detection)
  * **CH2 (A1)** – Left-hand EMG
  * **CH3 (A2)** – Right-hand EMG
* Sampling rate: **512 Hz**
* On-board features:
  * NeoPixel LEDs (6 pixels)
  * BLE connectivity
  * Shared LED + vibration motor on pin 7

### Controlled Device

* **BLE-enabled robotic car**
* Communication via **Bluetooth Low Energy (BLE)**

---

## 2. First-Time Setup: MAC Address Pairing

On first power-up, the NPG-Lite prints its **BLE MAC address** to the Serial Monitor.  
You need this address to pair with the car firmware.

**Steps:**
1. Flash the NPG-Lite firmware and open the Serial Monitor at **115200 baud**
2. Copy the MAC address printed between the `===` banners:
   ```
   ============================================
     NPG Lite MAC Address (copy this value)
   ============================================
   AA:BB:CC:DD:EE:FF
   Paste the MAC above into the car firmware
   when prompted, then reset the car.
   ============================================
   ```
3. Paste the MAC address into the car firmware where indicated
4. Reset the car, it will now connect automatically

---

## 3. Electrode Placement

### EEG (Channel 1 – A0)

**Purpose:**
* Beta-band power estimation
* Forward motion control

**Recommended placement:**
* Active electrode (AOP): **Center forehead**
* Reference (REF): **Behind the right ear**
* Ground (GND): **Behind the left ear**

---

### EMG – Left Hand (Channel 2 – A1)

* Place electrodes over **left forearm flexor muscles**
* Controls:
  * **Left turn**
  * **Backward motion (when combined with right EMG)**

---

### EMG – Right Hand (Channel 3 – A2)

* Place electrodes over **right forearm flexor muscles**
* Controls:
  * **Right turn**
  * **Backward motion (when combined with left EMG)**

**EMG placement tips:**
* Place electrodes parallel to muscle fibers
* Ensure good skin contact for clean signals

![Placement Image](./placement.jpeg)

---

## 4. Signal Processing Pipeline

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

---

### EMG Processing

* 50 Hz Notch Filter
* High-pass filter (70 Hz cutoff)
* Rectification
* Envelope detection (16-sample moving average)

---

## 5. User Controls & Interactions

### 5.1 Forward Movement (EEG – Beta Power)

* When **Beta band power > threshold (default: 4% of total power)**:
  * Car moves **forward**
  * BLE value sent: **`3`**
* When beta power falls below threshold:
  * Car stops
  * BLE value sent: **`0`**

> **Note:** Forward motion is automatically suppressed while a backward command is active.

---

### 5.2 EMG-Based Turning

#### Left Hand EMG (Channel A1)

* Envelope > threshold (default: 150 ):
  * Car turns **left**
  * BLE value sent: **`2`**

#### Right Hand EMG (Channel A2)

* Envelope > threshold (default: 150 ):
  * Car turns **right**
  * BLE value sent: **`1`**

---

### 5.3 Backward Motion (Dual EMG Activation)

* **Both EMG channels active simultaneously** (each > 50% of threshold):
  * Car moves **backward**
  * BLE value sent: **`4`**
  * EEG-based forward command is suppressed until backward state clears

---

## 6. BLE Control Protocol

### BLE Device Name

`UDL-BCI-Car`

### BLE Service

* Custom BLE service UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
* Two characteristics:

| Characteristic | UUID | Purpose |
|---|---|---|
| Char 1 | `beb5483e-...` | Motion control commands (Notify) |
| Char 2 | `1c95d5e3-...` | Reserved / extensible (Read, Write, Notify) |

### Control Values

| Value | Action     |
|------:|------------|
| 0     | Stop       |
| 1     | Right turn |
| 2     | Left turn  |
| 3     | Forward    |
| 4     | Backward   |

> **Efficiency note:** Commands are only transmitted over BLE when the value **changes**, reducing unnecessary radio traffic.

---

## 7. LED & Haptic Indicators

### NeoPixel LEDs

| Pixel | Color  | Meaning               |
|-------|--------|-----------------------|
| 0     | Orange | Device running        |
| 5     | Green  | BLE connected         |
| 5     | Red    | BLE disconnected      |
| 2     | Off    | Unused                |

### Vibration / LED (Pin 7)

| State              | Behavior                          |
|--------------------|-----------------------------------|
| BLE just connected | Double blink (120 ms on / 100 ms gap / 120 ms on) |
| BLE disconnected   | Off                               |

---

## 8. Tuning Thresholds

Default thresholds are a starting point. Adjust them in the firmware based on your signal levels:

| Parameter | Variable | Default | Description |
|---|---|---|---|
| EEG Beta threshold | `betaThreshold` | `4` | % of total power required for forward command |
| EMG envelope threshold | `emgThreshold` | `150` | ADC counts for single-hand turn command |
| Backward dual threshold | — | `emgThreshold × 0.5` | Each hand must exceed 50% of EMG threshold |

---

## 9. Intended Use & Disclaimer

**This firmware is intended for research, education, and controlled demonstrations only.**

* Use the car in open, safe environments
* Avoid operation near people or fragile objects
* Signal thresholds require per-user calibration, EEG and EMG vary significantly between individuals

---

> Making neuroscience affordable and accessible for everyone 
> — **Upside Down Labs**