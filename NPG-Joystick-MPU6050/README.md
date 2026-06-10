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
  - Double blinks trigger a left mouse click.
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
2. Connect the MPU6050 to NPG via the Qwiic port.
3. Install the `BleCombo.h` library using the [Library Installation](#library-installation) section below for instructions.
4. Compile and upload the sketch to your NPG Lite board.
5. Wear the headband and power on NPG.
6. Calibrate by following the vibration feedback.
7. Tilt your head to move the cursor. More tilt means faster movement; center to stop.
8. Blink twice for a left click and three times for a right click.

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
