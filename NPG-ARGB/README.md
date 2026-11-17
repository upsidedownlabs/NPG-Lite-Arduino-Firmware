# NPG ARGB

Control an addressable RGB (NeoPixel / WS2812) LED strip with one-channel EMG. Flex to fade the LEDs in, relax to fade them out.

## What You Need

- NPG Lite with the `NPG-ARGB.ino` firmware
- Any addressable RGB LED strip (NeoPixel / WS2812)
- STEMMA QT / Qwiic cable
- Three gel electrodes and snap cables
- Alcohol swabs for cleaning the skin

## Connections and Electrode Placement

1. Connect the strip to the Qwiic port of NPG Lite using a STEMMA QT / Qwiic cable. The SDA pin (GPIO 22) is used as a GPIO to drive the LEDs:
   - Blue (SDA) to the strip's data-in (DIN)
   - Red (3V3) to the strip's power
   - Black (GND) to the strip's GND
   - Yellow (SCL) is not used

   You can use a STEMMA QT to male jumper cable, or cut one end of the cable and solder the wires directly to the strip.
2. Prepare the electrode areas with alcohol swabs and let the skin dry completely.
3. Connect the EMG electrodes to channel A0:
   - Place the positive and negative electrodes a few centimetres apart over a muscle, for example a forearm flexor.
   - Keep both electrodes parallel to the muscle fibres.
   - Place the reference electrode on a nearby bony or less muscular area.

## Getting Started

1. Set `NUM_PIXELS` at the top of the sketch to match your strip.
2. Upload the firmware and turn on or reset NPG Lite.
3. Flex the muscle to fade the LEDs in, relax to fade them out.

## Customization

All settings are at the top of the sketch.

| Setting         | What it does                                       |
|-----------------|----------------------------------------------------|
| `LED_PIN`       | Pin driving the strip's data-in (default 22, SDA on the Qwiic port) |
| `NUM_PIXELS`    | Number of LEDs on your strip                       |
| `EMG_THRESHOLD` | Envelope level above which the LEDs fade in        |
| `FADE_MS`       | Time in ms for a full fade from off to full, and back |

### Setting the Threshold

1. Open the Serial Plotter or Serial Monitor at 115200 baud. The sketch prints the EMG envelope.
2. Note the value when the muscle is relaxed and when it is flexed.
3. Set `EMG_THRESHOLD` somewhere between the two.
   - LEDs turn on without flexing: increase it.
   - LEDs do not turn on when flexing: decrease it.

### Setting the Fade Speed

- Lower `FADE_MS`: faster fade.
- Higher `FADE_MS`: slower, smoother fade. For example, `2000` gives a 2 second fade.