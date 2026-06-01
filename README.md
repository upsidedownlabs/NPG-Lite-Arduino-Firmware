# NPG Lite Arduino Firmware

**Neuro PlayGround (NPG) Lite** is an open-source neuroscience development board by [Upside Down Labs](https://upsidedownlabs.tech/) that lets you read, process, and act on biosignals like EEG, EMG, and EOG directly from an ESP32-based wearable. This repository contains the complete Arduino firmware library for the NPG Lite hardware.


## What This Repo Contains

Each folder is a self-contained Arduino sketch targeting a specific application. Sketches range from raw signal streaming over serial to full Brain-Computer Interface (BCI) pipelines that control external devices wirelessly via Bluetooth Low Energy (BLE), infrared (IR), or Wi-Fi.

---

## Firmware Sketches

| Example | Code |
| ------- | ---- |
| Default program to show Visual, Audiotory, and Haptic feedback on NPG| [NPG-Default.ino](NPG-Default/NPG-Default.ino) |
| Calculate FFT & Band Power of single channel EEG and print on Serial | [Serial-FFT.ino](Serial-FFT/Serial-FFT.ino) |
| Bluetooh Low Energy (BLE) server to notify client with real-time NPG data | [BLE-Server.ino](BLE-Server/BLE-Server.ino) |
| BLE client to take notification from server and trigger GPIO | [BLE-Client.ino](BLE-Client/BLE-Client.ino) |
| BLE server to nofity client based on EEG band (beta) power triggers | [BLE-BCI-Server-Toggle.ino](BLE-BCI-Server-Toggle/BLE-BCI-Server-Toggle.ino) |
| InfraRed (IR) reciever code to identify LG AC remote button commands | [IR-LG-Receive.ino](IR-LG-Receive/IR-LG-Receive.ino) |
| IR signal send example code to control LG AC to toggle ON/OFF using user button | [IR-LG-Send.ino](IR-LG-Send/IR-LG-Send.ino) |
| Brain Computer Interface (BCI) to toggle LG AC ON/OFF using EEG band (beta) power | [BCI-IR-Send.ino](BCI-IR-Send/BCI-IR-Send.ino) |
| BLE client that receives notifications from the server and triggers GPIO to control the car | [BLE-BCI-Car.ino](BLE-BCI-Car/BLE-BCI-Car.ino) |
| BCI remote (server) to drive the BLE car using EEG band (beta) power and EMG (envelope) data | [BLE-BCI-Car-Remote.ino](BLE-BCI-Car-Remote/BLE-BCI-Car-Remote.ino) |
| Brain Computer Interface example sketch for Double blink and focus detection. | [BCI-Blink-Serial.ino](BCI-Blink-Serial/BCI-Blink-Serial.ino) |
| Brain Computer Interface to control a menu of options using Double blink and focus detection for ALS patients. | [BCI-Blink-BLE.ino](BCI-Blink-BLE/BCI-Blink-BLE.ino) |
| MPU6050 sketch to stream 3-axis accelerometer data and send 4 keystrokes to play video games on laptop. | [Gyro-Motion-Detection.ino](Gyro-Motion-Detection/Gyro-Motion-Detection.ino) |
| Detects double and triple blinks from EOG signals using high‑pass and notch IIR filters with envelope detection. | [Blinky-Keys-Serial.ino](Blinky-Keys-Serial/Blinky-Keys-Serial.ino) |
| Implements a BLE HID keyboard that sends right‑arrow on double blinks and left‑arrow on triple blinks to control slides in a presentation.| [Blinky-Keys-BLE.ino](Blinky-Keys-BLE/Blinky-Keys-BLE.ino) |
| Implements a BLE gamepad that reads EEG and EMG signals to control games on Windows, using focus and muscle contractions.| [BCI-BLE-Gamepad.ino](BCI-BLE-Gamepad/BCI-BLE-Gamepad.ino) |

---

## Standardized NeoPixel LED Layout

All BLE-enabled or peripheral-using sketches follow a shared 6-LED NeoPixel convention so the board behaves predictably across firmware variants.

| LED | Standard Role | Colors |
|-----|--------------|--------|
| **LED 1** | BLE / Wi-Fi connection status | GREEN = connected · RED = disconnected · BLUE = actively streaming |
| **LED 2** | General purpose | Sketch-specific |
| **LED 3** | I²C peripheral status | GREEN = device found · RED = device missing |
| **LED 4** | General purpose | Sketch-specific |
| **LED 5** | General purpose | Sketch-specific |
| **LED 6** | Battery level | GREEN > 70 % · ORANGE 20–70 % · RED < 20 % |
 ---
## For more information
- [Upside Down Labs](https://upsidedownlabs.tech)
- [NPG Lite Documentation](https://docs.upsidedownlabs.tech/hardware/bioamp/neuro-play-ground-lite/index.html)
- [Contact Support](mailto:contact@upsidedownlabs.tech)
---
