# NPG-Joystick: Neuro Playground Lite Joystick-style Mouse Control

This project turns the Neuro Playground Lite (NPG) into a hands-free joystick-style mouse controller using a headband. It combines EEG/EOG blink detection and head movement sensing for intuitive computer control.

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

## Usage
1. Attach the NPG and MPU6050 to a headband.
2. Connect the MPU6050 to NPG via the Qwiic port.
3. Wear the headband and power on NPG.
4. Calibrate by following vibration feedback.
5. Tilt your head to move the cursor. More tilt means faster movement; center to stop.
6. Blink twice for left click, three times for right click.

---

**Made by Upside Down Labs.**

Open-source, affordable neuroscience for everyone!
