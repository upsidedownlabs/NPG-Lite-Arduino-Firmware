# EMG Air Mouse

Use hand movement to control the mouse cursor and one-channel forearm EMG to control the left mouse button.

## What You Need

- NPG Lite with the `EMG-AirMouse.ino` firmware
- MPU6050 and Qwiic cable
- Three gel electrodes and snap cables
- Muscle BioAmp Band (optional, for a wearable setup)
- A computer or phone with Bluetooth

## Connections and Electrode Placement

1. Prepare the electrode areas with alcohol swabs and let the skin dry completely.
2. Connect the MPU6050 to the Qwiic port of NPG Lite.
3. Connect the EMG electrodes to channel A0:
   - Place the positive and negative electrodes a few centimetres apart over a forearm flexor muscle.
   - Keep both electrodes parallel to the muscle fibres.
   - Place the reference electrode on a nearby bony or less muscular area.

The image below shows the NPG Lite channel and cable connections. For this project, use the forearm placement described above, not the face placement shown in the image.

![NPG Lite electrode connections](../Wheelchair-Sim/electrode-placement.png)

You can secure the NPG Lite and electrodes with a Muscle BioAmp Band to make the setup wearable. For a simple handheld setup, hold the NPG Lite and MPU6050 in one hand and place the EMG electrodes on the other forearm.

## Getting Started and Calibration

1. Upload the firmware, connect the MPU6050 and electrodes, then turn on or reset NPG Lite.
2. Follow the vibration cues:
   - **First double vibration:** raise the hand holding or wearing the NPG Lite upward.
   - **Next single vibration:** move the hand back to its original position.
   - **Second double vibration:** move the hand to the left.
   - **Next single vibration:** move the hand back to its original position.
3. Keep the hand still while the gyro offsets are calculated.
4. Open Bluetooth settings and connect to **NPG Air Mouse**.

## Controls

- Move the calibrated hand to move the cursor.
- Flex the muscle under the electrodes for a left click.
- Keep the muscle flexed to hold the left mouse button; relax it to release.
