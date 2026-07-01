# NPG-Joystick: Neuro Playground Lite Joystick-style Mouse Control

This project turns the Neuro Playground Lite (NPG) into a hands-free joystick-style mouse controller using a headband. It combines EEG/EOG blink detection and head movement sensing for intuitive computer control.

## **[How to Use](#how-to-use)**

## What It Does
- **Head Movement → Joystick-Style Cursor Movement:**
  - The MPU6050 sensor (gyro + accelerometer) is attached to the headband and connected to NPG via the Qwiic port.
  - Only the accelerometer is used: head tilt angles (pitch and roll) are computed from accelerometer data to drive cursor movement.
  - Tilting your head up/down or left/right moves the cursor continuously. The greater the tilt, the faster it moves. Return your head to neutral to stop.
- **Blink Detection → Mouse Clicks:**
  - NPG reads single-channel EOG data.
  - Jaw clench triggers a left mouse click.
  - Double blinks trigger a left mouse hold, and again double blinking releases the hold.
  - Triple blinks trigger a right mouse click.

## How It Works
- **Sensors Used:**
  - **MPU6050 (accelerometer + gyroscope):** Head tilt angles (pitch and roll) are computed from accelerometer data. The gyroscope is not used.
  - **EEG/EOG Input:** Detects blinks for mouse clicks.
- **Joystick-Style Cursor Control:**
  - The code maps head tilt angle to cursor speed, tilt more to move faster, center your head to stop. This is analogous to a joystick rather than a mouse.
  - Sensitivity, deadzone, and acceleration are adjustable for comfort and precision.
- **Blink Detection:**
  - The EOG signal is filtered and analyzed to detect blinks.
  - Timing logic distinguishes between double and triple blinks for different mouse clicks.
- **Calibration:**
  - The headband calibrates itself for neutral position and axis directions using accelerometer gesture detection and vibration feedback.
- **BLE Connection:**
  - NPG acts as a Bluetooth mouse and keyboard, allowing wireless control.

## How To Use
1. Attach the NPG and MPU6050 to a headband.
2. Connect the MPU6050 to the NPG via the Qwiic port.
3. Install the `BleCombo.h` library following the [Library Installation](#library-installation) section.
4. Compile and upload the sketch to your NPG Lite board.
5. Set up your electrodes and calibrate the sensor as described in the [Connection and Calibration](#connection-and-calibration) section below.
6. Move your head to control the mouse cursor.
7. Jaw clench for a left click, double blink to toggle/release left hold, and triple blink for a right click.

## Connection and Calibration
1. Clean your forehead and the bony area behind your ears with alcohol swabs.
2. Attach the snap cables to the gel electrodes, then peel and stick the electrodes onto the cleaned areas as shown below.

<img src="../assets/NPGConnectionDiagram.jpeg" width=90% alt="NPG Lite connection diagram"/>

3. Turn on the NPG, open Bluetooth on your device, and connect to `NPG Lite BCI Joystick`. You'll feel a short vibration confirming the connection.

4. **Calibration begins automatically.** Follow the vibration cues:

   - **First vibration (~3 seconds):** Look up or tilt your head upward and hold the position until the vibration stops, then return to neutral.

   <img src="../assets/NPGHeadUpwardDiagram.jpeg" width=90% alt="Head upward position"/>

   - **Second vibration (~3 seconds):** Tilt your head to the left and hold until the vibration stops, then return to neutral.

   <img src="../assets/NPGHeadLeftDiagram.jpeg" width=90% alt="Head left position"/>

5. After a brief pause, you'll feel one final short vibration, which means calibration is complete and the joystick is now active.

## Library Installation

> For a full visual guide covering both the ZIP method and Library Manager method, refer to:
> **[Installing Arduino Library](https://docs.upsidedownlabs.tech/guides/usage-guides/arduino-library-from-github/index.html)**

The sketch uses the `BleCombo.h` library from [ESP32-BLE-Combo](https://github.com/upsidedownlabs/ESP32-BLE-Combo), which is not included with Arduino IDE by default. Follow the steps below to install it:

1. Open the [ESP32-BLE-Combo](https://github.com/upsidedownlabs/ESP32-BLE-Combo) GitHub repository.
2. Click the green `<> Code` button. In the dropdown, click `Download ZIP`.
3. Open Arduino IDE. Click on **`Sketch` → `Include Library` → `Add .ZIP Library...`**.
4. Select the downloaded ZIP file and install it.
5. In case of any issue, refer to the library installation guide provided above.

---

**Made by Upside Down Labs.**

Open-source, affordable neuroscience for everyone!
