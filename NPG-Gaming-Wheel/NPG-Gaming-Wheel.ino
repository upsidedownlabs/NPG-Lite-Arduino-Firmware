// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Krishnanshu Mittal - [krishnanshu@upsidedownlabs.tech]
// Copyright (c) 2025 Deepak Khatri - [deepak@upsidedownlabs.tech]
// Copyright (c) 2025 Upside Down Labs - [contact@upsidedownlabs.tech]

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
   Neuro Playground Lite - Gaming Wheel (XInput / Xbox controller emulation)
   
   MPU6050 on the NPG Lite          -> LEFT ANALOG STICK X  (proportional steering)
   EMG env1 (leg 1)                 -> RIGHT TRIGGER  (accelerator)
   EMG env2 (leg 2)                 -> LEFT TRIGGER   (brake)
   Boot button                      -> A button
   Game rumble                      -> vibration motor
    
   Required libraries:
     ESP32-BLE-CompositeHID  https://github.com/upsidedownlabs/ESP32-BLE-CompositeHID
     NimBLE-Arduino (2.1.2)  https://github.com/h2zero/NimBLE-Arduino
     Callback                https://github.com/tomstewart89/Callback
     Adafruit MPU6050        https://github.com/adafruit/Adafruit_MPU6050
     Adafruit NeoPixel       https://github.com/adafruit/Adafruit_NeoPixel
*/

// Core includes
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <vector>
#include <BleCompositeHID.h>
#include <XboxGamepadDevice.h>

// ---- MPU6050 ----
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---- Turn features on or off (comment out to disable) ----
#define ENABLE_STEERING            // wheel -> left stick
#define ENABLE_EMG_ACCELERATOR     // EMG 1 -> right trigger
#define ENABLE_EMG_BRAKE           // EMG 2 -> left trigger
#define ENABLE_BOOT_BUTTON         // boot button -> A button
#define ENABLE_RUMBLE              // game vibration -> vibration motor

// ---- Controller type ----
// Comment this out to look like an older Xbox One S controller instead.
#define USE_XBOX_SERIES_X

#define BLE_DEVICE_NAME     "NPG Gaming Wheel"
#define BLE_MANUFACTURER    "Upside Down Labs"

#define BOOT_BUTTON_XBOX    XBOX_BUTTON_A    // button sent by the boot button

// ---- Pins ----
#define VIBRATION_PIN       7                // vibration motor
#define BOOT_BUTTON_PIN     9                // boot button

// ---- Status LEDs ----
#define PIN_NEOPIXEL        15               // NeoPixel data pin
#define BLUE_LED_DURATION   100              // how long the BLE led stays blue

Adafruit_NeoPixel pixel(6, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5
#define IMU_LED 3

// ---- Steering settings ----
#define STEER_DEADZONE_DEG    0.75f          // degrees near the center that stay at 0
#define STEER_SMOOTHING       0.69f          // higher value is smoother but slower
#define STEER_MIN_RANGE_DEG   10.0f          // calibration smaller than this is ignored
#define STEER_FALLBACK_DEG    45.0f          // range used when calibration is ignored
#define STEER_EXPO            0.0f           // 0.0 is linear, 0.3 to 0.5 is finer near center
#define STEER_RESPONSE_FLOOR  0.15f          // stick value right after the deadzone, 0.0 to disable
#define MPU_UPDATE_MS         10             // read the sensor every 10 ms

// ---- EMG trigger settings ----
#define EMG_ANALOG_TRIGGERS                  // comment out to make the triggers on/off only
#define EMG_ENV_MIN         75.0f            // envelope where the trigger starts to move
#define EMG_ENV_MAX         150.0f           // envelope for a fully pressed trigger

// ---- Report settings ----
#define REPORT_INTERVAL_MS  10               // send one report every 10 ms

// ---- Rumble settings ----
#define RUMBLE_MIN_MAGNITUDE 16              // motor turns on above this strength
#define RUMBLE_TIMEOUT_MS    1000            // turn the motor off if the game stops sending

// ---- Calibration timing ----
#define CAL_HOLD_MS          3000            // hold still after connecting, before the first buzz
#define CAL_TURN_MS          3000            // how long each buzz lasts
#define CAL_RETURN_MS        3000            // time given to come back to the center
#define CAL_SETTLE_MS        2000            // settle time before measuring the center
#define CAL_DONE_BUZZ_MS     300             // short buzz that means calibration is done
#define CAL_NEUTRAL_SAMPLES  100             // readings averaged for the center angle

#define IMU_CHECK_MS         500             // how often to check the MPU6050 is still there

// ---- Gamepad ----
XboxGamepadDevice *gamepad = nullptr;
BleCompositeHID compositeHID(BLE_DEVICE_NAME, BLE_MANUFACTURER, 100);
unsigned long lastReportMs = 0;

// ---- MPU6050 ----
Adafruit_MPU6050 mpu;
uint32_t mpuAddress = 0x68;
float neutralRoll = 0;                       // wheel angle at the center
float smoothedDelta = 0;                     // smoothed angle away from the center
float wheelAngle = 0;                        // running wheel angle, keeps counting past 180
float lastRawRoll = 0;                       // last raw sensor angle, used to unwrap
bool rollTrackerReady = false;
bool isCalibrated = false;
int rollDirection = 1;                       // flips the sign if the sensor is mounted the other way
unsigned long lastMPUUpdateMs = 0;

// how far the wheel turns to each side, in degrees
float steerLeftRange = STEER_FALLBACK_DEG;
float steerRightRange = STEER_FALLBACK_DEG;

enum CalibrationState {
  CAL_IDLE,
  CAL_INIT_WAIT,
  CAL_LEFT_VIBRATE,
  CAL_LEFT_WAIT,
  CAL_RIGHT_VIBRATE,
  CAL_RIGHT_WAIT,
  CAL_NEUTRAL_SAMPLE,
  CAL_DONE_BUZZ,
  CAL_COMPLETE
};

CalibrationState calState = CAL_IDLE;
unsigned long calStateStartTime = 0;
float calRestRoll = 0;                       // angle at rest
float maxLeftRoll = 0;                       // angle at full left
float maxRightRoll = 0;                      // angle at full right
float maxLeftDev = 0;                        // biggest movement seen while turning left
float maxRightDev = 0;                       // biggest movement seen while turning right
int neutralSampleCount = 0;
float neutralRollSum = 0;

// ---- Boot button ----
bool bootButtonPressed = false;

// ---- Rumble ----
// set by the BLE callback, used in loop()
volatile bool rumbleRequested = false;
volatile unsigned long lastRumbleMs = 0;
bool rumbleActive = false;

// ---- EMG Signal processing config ----
#define SAMPLE_RATE   512
#define INPUT_PIN1    A0    // EMG channel 1 -> accelerator
#define INPUT_PIN2    A1    // EMG channel 2 -> brake

// ----Filter Classes----

// Band-Stop Butterworth IIR digital filter
// Sampling rate: 512.0 Hz, frequency: [48.0, 52.0] Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// Reference: https://github.com/upsidedownlabs/BioAmp-Filter-Designer
class NotchFilter {
private:
  struct BiquadState { float z1 = 0, z2 = 0; };
  BiquadState state0;
  BiquadState state1;

public:
  float process(float input) {
    float output = input;
    float x0 = output - (-1.58696045f * state0.z1) - (0.96505858f * state0.z2);
    output = 0.96588529f * x0 + -1.57986211f * state0.z1 + 0.96588529f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    float x1 = output - (-1.62761184f * state1.z1) - (0.96671306f * state1.z2);
    output = 1.00000000f * x1 + -1.63566226f * state1.z1 + 1.00000000f * state1.z2;
    state1.z2 = state1.z1;
    state1.z1 = x1;

    return output;
  }

  void reset() {
    state0.z1 = state0.z2 = 0;
    state1.z1 = state1.z2 = 0;
  }
} filters[2];  // Only 2 notch filters

// High-Pass Butterworth IIR digital filter
// Sampling rate: 512.0 Hz, frequency: 70.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// Reference: https://github.com/upsidedownlabs/BioAmp-Filter-Designer
class EMGFilter {
private:
  struct BiquadState { float z1 = 0, z2 = 0; };
  BiquadState state0;

public:
  float process(float input) {
    float output = input;
    float x0 = output - (-0.85080258f * state0.z1) - (0.30256882f * state0.z2);
    output = 0.53834285f * x0 + -1.07668570f * state0.z1 + 0.53834285f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;
    return output;
  }

  void reset() {
    state0.z1 = state0.z2 = 0;
  }
} emgfilters[2];  // Only 2 EMG high-pass filters

class EnvelopeFilter {
private:
  std::vector<double> buf;
  double sum = 0.0;
  int idx = 0;
  const int N;
public:
  EnvelopeFilter(int n) : N(n) { buf.resize(N, 0.0); }
  double getEnvelope(double v) {
    sum -= buf[idx];
    sum += v;
    buf[idx] = v;
    idx = (idx + 1) % N;
    return sum / N;
  }
} env1(16), env2(16);  // Only 2 envelope filters

// ---- Status LED colors ----
// The three leds are always written together and shown once, and only when a
// color changed. One show() per change keeps the leds out of the way of the
// sensor reads and the report timing.
unsigned long lastCmdSentMs = 0;             // last time an input actually moved
uint32_t bleColor = 0;
uint32_t batteryColor = 0;
uint32_t imuColor = 0;
uint32_t shownBleColor = 0xFFFFFFFF;
uint32_t shownBatteryColor = 0xFFFFFFFF;
uint32_t shownImuColor = 0xFFFFFFFF;

// ---- Battery ----
#define BATTERY_VOLTAGE_PIN A6
#define BATTERY_CHECK_MS    30000            // read the battery every 30 s

unsigned long lastBatteryCheck = 0;
uint32_t batteryWinSum = 0;
uint16_t batteryWinCount = 0;
int lastBatteryPct = -1;
uint8_t risingCount = 0;
const uint8_t RISING_THRESHOLD = 3;

const float voltageLUT[] = {
  3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
  3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20
};

const int percentLUT[] = {
  0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
  50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
};

const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

float interpolatePercentage(float voltage) {
  if (voltage <= voltageLUT[0])
    return 0;
  if (voltage >= voltageLUT[lutSize - 1])
    return 100;
  int i = 0;
  while (i < lutSize - 1 && voltage > voltageLUT[i + 1])
    i++;
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}

// Averages the readings taken since the last call, then only lets the
// percentage rise after a few higher readings in a row so it does not jump about.
int getCurrentBatteryPercentage() {
  float avgRaw = (batteryWinCount > 0) ? (batteryWinSum / batteryWinCount) : analogRead(BATTERY_VOLTAGE_PIN);
  batteryWinSum = 0;
  batteryWinCount = 0;
  float voltage = (avgRaw / 1000.0) * 2;
  voltage += 0.022;
  float percentage = interpolatePercentage(voltage);
  if (lastBatteryPct == -1) {
    lastBatteryPct = (int)percentage;
  } else if ((int)percentage < lastBatteryPct) {
    lastBatteryPct = (int)percentage;
    risingCount = 0;
  } else if ((int)percentage > lastBatteryPct) {
    risingCount++;
    if (risingCount >= RISING_THRESHOLD) {
      lastBatteryPct = (int)percentage;
      risingCount = 0;
    }
  } else {
    risingCount = 0;
  }
  return lastBatteryPct;
}

// Turns a battery percentage into the color for the battery led.
uint32_t batteryPercentToColor(int percent) {
  if (percent <= 20) return pixel.Color(20, 0, 0);
  if (percent <= 70) return pixel.Color(35, 7, 0);
  return pixel.Color(0, 20, 0);
}

// Writes all three leds and shows them, but only when a color changed.
void updateStatusLeds(bool connected, unsigned long nowMs) {
  // Bluetooth led: red until connected, blue while an input is moving,
  // green when connected but idle.
  if (!connected) {
    bleColor = pixel.Color(20, 0, 0);
  } else if (nowMs - lastCmdSentMs < BLUE_LED_DURATION) {
    bleColor = pixel.Color(0, 0, 30);
  } else {
    bleColor = pixel.Color(0, 20, 0);
  }

  if (bleColor == shownBleColor &&
      batteryColor == shownBatteryColor &&
      imuColor == shownImuColor) {
    return;
  }

  shownBleColor = bleColor;
  shownBatteryColor = batteryColor;
  shownImuColor = imuColor;

  pixel.setPixelColor(BLE_LED, bleColor);
  pixel.setPixelColor(BATTERY_LED, batteryColor);
  pixel.setPixelColor(IMU_LED, imuColor);
  pixel.show();
}

void startVibration() {
  digitalWrite(VIBRATION_PIN, HIGH);
}

void stopVibration() {
  digitalWrite(VIBRATION_PIN, LOW);
}

// Keeps an angle difference between -180 and +180 degrees.
float wrapAngle180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

// Reads the current wheel angle from the MPU6050.
// This value only covers -180 to +180 and jumps to the other end past that.
float readRoll() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
}

// Keeps a running wheel angle that does not jump at 180 degrees.
// The sensor angle wraps from +180 straight to -180, so instead of using it
// directly we only add the small change since the last read. The running angle
// can then pass 180 and keep counting, which is what a 180 degree turn needs.
// Returns true when a new reading was taken.
bool updateWheelAngle(unsigned long nowMs) {
  if (nowMs - lastMPUUpdateMs < MPU_UPDATE_MS) return false;
  lastMPUUpdateMs = nowMs;

  float raw = readRoll();

  if (!rollTrackerReady) {
    rollTrackerReady = true;
    lastRawRoll = raw;
    wheelAngle = raw;
    return true;
  }

  wheelAngle += wrapAngle180(raw - lastRawRoll);
  lastRawRoll = raw;
  return true;
}

// Starts the calibration sequence from the beginning.
// The motor is always stopped first, so a restart part way through can never
// leave it running into the next buzz.
void startCalibration(unsigned long nowMs) {
  stopVibration();
  rumbleRequested = false;
  rumbleActive = false;

  calState = CAL_INIT_WAIT;
  calStateStartTime = nowMs;

  neutralSampleCount = 0;
  neutralRollSum = 0;
  maxLeftDev = 0;
  maxRightDev = 0;

  isCalibrated = false;
  smoothedDelta = 0;

  if (gamepad) gamepad->setLeftThumb(0, 0);
}

// Works out the turn direction and how far the wheel turns to each side.
void finishCalibration() {
  // No wrapping here. These are running angles, so the difference is already
  // the real amount turned even when it goes past 180 degrees.
  float leftSpan  = maxLeftRoll - neutralRoll;
  float rightSpan = maxRightRoll - neutralRoll;

  // After this, turning right gives a positive value and left gives a negative one.
  rollDirection = ((rightSpan - leftSpan) >= 0) ? 1 : -1;

  steerLeftRange  = fabs(leftSpan);
  steerRightRange = fabs(rightSpan);

  // If the wheel barely moved the reading is bad, so use a default range instead.
  // Otherwise the stick would become far too sensitive.
  if (steerLeftRange  < STEER_MIN_RANGE_DEG) steerLeftRange  = STEER_FALLBACK_DEG;
  if (steerRightRange < STEER_MIN_RANGE_DEG) steerRightRange = STEER_FALLBACK_DEG;

  smoothedDelta = 0;
  isCalibrated = true;
}

// Calibration steps: rest, turn left, rest, turn right, rest.
// newSample is true only when a fresh sensor reading was taken this loop.
void updateCalibrationStateMachine(unsigned long nowMs, bool newSample) {
  if (calState == CAL_IDLE || calState == CAL_COMPLETE) return;

  unsigned long elapsed = nowMs - calStateStartTime;

  switch (calState) {
    case CAL_INIT_WAIT:
      // Hold the wheel at rest. The motor buzzes when it is time to turn left.
      if (elapsed >= CAL_HOLD_MS) {
        calRestRoll = wheelAngle;
        maxLeftRoll = calRestRoll;
        maxLeftDev = 0;
        calState = CAL_LEFT_VIBRATE;
        calStateStartTime = nowMs;
        startVibration();
      }
      break;

    case CAL_LEFT_VIBRATE: {
      // Keep the angle that is furthest from rest. That is the full left position.
      float dev = fabs(wheelAngle - calRestRoll);
      if (dev > maxLeftDev) {
        maxLeftDev = dev;
        maxLeftRoll = wheelAngle;
      }
      if (elapsed >= CAL_TURN_MS) {
        stopVibration();
        calState = CAL_LEFT_WAIT;
        calStateStartTime = nowMs;
      }
      break;
    }

    case CAL_LEFT_WAIT:
      // Go back to rest. The motor buzzes when it is time to turn right.
      if (elapsed >= CAL_RETURN_MS) {
        maxRightRoll = calRestRoll;
        maxRightDev = 0;
        calState = CAL_RIGHT_VIBRATE;
        calStateStartTime = nowMs;
        startVibration();
      }
      break;

    case CAL_RIGHT_VIBRATE: {
      // Keep the angle that is furthest from rest. That is the full right position.
      float dev = fabs(wheelAngle - calRestRoll);
      if (dev > maxRightDev) {
        maxRightDev = dev;
        maxRightRoll = wheelAngle;
      }
      if (elapsed >= CAL_TURN_MS) {
        stopVibration();
        calState = CAL_RIGHT_WAIT;
        calStateStartTime = nowMs;
      }
      break;
    }

    case CAL_RIGHT_WAIT:
      // Let the wheel settle at rest before measuring the center.
      if (elapsed >= CAL_SETTLE_MS) {
        calState = CAL_NEUTRAL_SAMPLE;
        calStateStartTime = nowMs;
        neutralSampleCount = 0;
        neutralRollSum = 0;
      }
      break;

    case CAL_NEUTRAL_SAMPLE:
      // Average the readings to get the center angle. Only count fresh ones,
      // otherwise the same reading would be added over and over.
      if (neutralSampleCount < CAL_NEUTRAL_SAMPLES) {
        if (newSample) {
          neutralRollSum += wheelAngle;
          neutralSampleCount++;
        }
      } else {
        neutralRoll = neutralRollSum / CAL_NEUTRAL_SAMPLES;
        finishCalibration();
        calState = CAL_DONE_BUZZ;
        calStateStartTime = nowMs;
        startVibration();
      }
      break;

    case CAL_DONE_BUZZ:
      // One short buzz means calibration is done and the wheel is ready.
      if (elapsed >= CAL_DONE_BUZZ_MS) {
        stopVibration();
        calState = CAL_COMPLETE;
      }
      break;

    default:
      break;
  }
}

// Turns the wheel angle into a left stick value.
void updateSteering(unsigned long nowMs) {
#ifdef ENABLE_STEERING
  if (!isCalibrated) return;

  // The running angle never wraps, so turning past 180 degrees just keeps
  // counting up and the value below is clamped to full lock instead of flipping.
  float rawDelta = (wheelAngle - neutralRoll) * rollDirection;
  smoothedDelta = STEER_SMOOTHING * smoothedDelta + (1.0f - STEER_SMOOTHING) * rawDelta;

  float magnitude = fabs(smoothedDelta) - STEER_DEADZONE_DEG;
  int16_t steerAxis = 0;

  if (magnitude > 0.0f) {
    float range = ((smoothedDelta > 0) ? steerRightRange : steerLeftRange) - STEER_DEADZONE_DEG;
    if (range < 1.0f) range = 1.0f;

    float norm = magnitude / range;          // 0 at the edge of the deadzone, 1 at full turn
    if (norm > 1.0f) norm = 1.0f;

    // Optional curve for finer control near the center
    if (STEER_EXPO > 0.0f) {
      norm = (1.0f - STEER_EXPO) * norm + STEER_EXPO * norm * norm * norm;
    }

    // Many games have their own deadzone on the stick and ignore small values.
    // Jump straight to STEER_RESPONSE_FLOOR after our deadzone instead of
    // ramping up from zero, so the game actually sees the movement.
    if (STEER_RESPONSE_FLOOR > 0.0f) {
      norm = STEER_RESPONSE_FLOOR + (1.0f - STEER_RESPONSE_FLOOR) * norm;
    }

    float value = norm * (float)XBOX_STICK_MAX;
    steerAxis = (int16_t)((smoothedDelta > 0) ? value : -value);
  }

  // the led goes blue while the wheel is being turned
  if (steerAxis != 0) lastCmdSentMs = nowMs;

  gamepad->setLeftThumb(steerAxis, 0);
#endif
}

// Turns an EMG envelope value into a trigger value.
uint16_t envelopeToTrigger(float envelope) {
#ifdef EMG_ANALOG_TRIGGERS
  if (envelope <= EMG_ENV_MIN) return 0;
  float norm = (envelope - EMG_ENV_MIN) / (EMG_ENV_MAX - EMG_ENV_MIN);
  if (norm > 1.0f) norm = 1.0f;
  return (uint16_t)(norm * XBOX_TRIGGER_MAX);
#else
  return (envelope > EMG_ENV_MIN) ? XBOX_TRIGGER_MAX : 0;
#endif
}

// Sends the A button when the boot button is pressed.
void updateBootButton(unsigned long nowMs) {
#ifdef ENABLE_BOOT_BUTTON
  bool currentButtonState = digitalRead(BOOT_BUTTON_PIN);

  // the button reads LOW when it is pressed
  if (currentButtonState == LOW && !bootButtonPressed) {
    bootButtonPressed = true;
    gamepad->press(BOOT_BUTTON_XBOX);
    lastCmdSentMs = nowMs;
  } else if (currentButtonState == HIGH && bootButtonPressed) {
    bootButtonPressed = false;
    gamepad->release(BOOT_BUTTON_XBOX);
  }
#endif
}

// Called by the game when it wants the controller to vibrate.
void OnVibrateEvent(XboxGamepadOutputReportData data) {
  rumbleRequested = (data.weakMotorMagnitude >= RUMBLE_MIN_MAGNITUDE ||
                     data.strongMotorMagnitude >= RUMBLE_MIN_MAGNITUDE);
  lastRumbleMs = millis();
}

FunctionSlot<XboxGamepadOutputReportData> vibrationSlot(OnVibrateEvent);

// Runs the vibration motor when the game asks for it.
void updateRumble(unsigned long nowMs, bool connected) {
#ifdef ENABLE_RUMBLE
  // Do not touch the motor while calibration is still using it.
  if (calState != CAL_COMPLETE) return;

  bool shouldRumble = rumbleRequested &&
                      (nowMs - lastRumbleMs < RUMBLE_TIMEOUT_MS) &&
                      connected;

  if (shouldRumble != rumbleActive) {
    rumbleActive = shouldRumble;
    if (rumbleActive) startVibration(); else stopVibration();
  }
#endif
}

// Centers every input so a new connection starts clean.
void resetGamepadState() {
  gamepad->resetInputs();
  gamepad->setLeftThumb(0, 0);
  gamepad->setRightThumb(0, 0);
  gamepad->setTriggers(0, 0);
  gamepad->sendGamepadReport();
  bootButtonPressed = false;
}

// True when the MPU6050 answers on the I2C bus.
bool isMPUConnected() {
  Wire.beginTransmission(mpuAddress);
  return (Wire.endTransmission() == 0);
}

void setup() {
  pixel.begin();
  pixel.clear();
  pixel.show();

  Wire.begin(22, 23);

  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);

#ifdef ENABLE_BOOT_BUTTON
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  // internal pullup for the boot button
#endif

  batteryColor = batteryPercentToColor(getCurrentBatteryPercentage());
  imuColor = pixel.Color(20, 0, 0);        // red until the MPU6050 answers
  updateStatusLeds(false, millis());       // battery on, IMU red, bluetooth red

  // Wait here until the MPU6050 answers. Bluetooth is not started yet, so the
  // wheel does not show up for pairing until it can actually steer.
  while (!mpu.begin()) {
    static uint16_t fader = 100;
    static bool decreasing = true;
    pixel.setPixelColor(IMU_LED, pixel.Color(fader, 0, 0));
    pixel.show();
    delay(20);
    if (decreasing) {
      fader = fader - 2;
      if (fader < 10) {
        decreasing = false;
      }
    } else {
      fader = fader + 2;
      if (fader > 100) {
        decreasing = true;
      }
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  imuColor = pixel.Color(0, 20, 0);
  shownImuColor = 0xFFFFFFFF;              // force a refresh after the fade loop
  updateStatusLeds(false, millis());

  // ---- Start the BLE Xbox controller ----
#ifdef USE_XBOX_SERIES_X
  XboxGamepadDeviceConfiguration *config = new XboxSeriesXControllerDeviceConfiguration();
#else
  XboxGamepadDeviceConfiguration *config = new XboxOneSControllerDeviceConfiguration();
#endif

  // The real Xbox VID and PID make Windows load the Xbox driver
  // instead of treating this as a plain BLE HID device.
  BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();

  // Send one report holding all the values, instead of one report per value.
  config->setAutoReport(false);

  gamepad = new XboxGamepadDevice(config);
#ifdef ENABLE_RUMBLE
  gamepad->onVibrate.attach(vibrationSlot);
#endif

  compositeHID.addDevice(gamepad);
  compositeHID.begin(hostConfig);

  lastBatteryCheck = millis();             // the battery was just read above
}

void loop() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  // ---- Check the MPU6050 is still there ----
  // Bluetooth is only useful while the wheel can steer, so if the MPU6050 goes
  // away we restart. That drops the Bluetooth connection, and setup() then waits
  // for the MPU6050 again before it starts advertising.
  static unsigned long lastImuCheckMs = 0;
  if (nowMs - lastImuCheckMs >= IMU_CHECK_MS) {
    lastImuCheckMs = nowMs;
    if (!isMPUConnected()) {
      stopVibration();
      ESP.restart();
    }
  }

  // ---- Bluetooth connection ----
  bool connected = compositeHID.isConnected();
  static bool lastConnected = false;
  if (connected != lastConnected) {
    lastConnected = connected;
    if (connected) {
      // Calibrate every time a host connects. The 3 second hold happens inside
      // CAL_INIT_WAIT, so the first buzz comes 3 seconds after connecting.
      resetGamepadState();
      startCalibration(nowMs);
    } else {
      stopVibration();
      calState = CAL_IDLE;
      isCalibrated = false;
    }
  }

  // ---- Battery ----
  if (nowMs - lastBatteryCheck >= BATTERY_CHECK_MS) {
    lastBatteryCheck = nowMs;
    int currentBattery = getCurrentBatteryPercentage();
    batteryColor = batteryPercentToColor(currentBattery);
    // the same reading also goes to the controller battery indicator
    if (connected) {
      compositeHID.setBatteryLevel((uint8_t)currentBattery);
    }
  }

  // ---- Wheel angle ----
  // Tracked first and always, even during calibration. If reads are skipped the
  // wheel can pass 180 degrees unnoticed and the running angle would be wrong
  // from then on.
  bool newWheelSample = updateWheelAngle(nowMs);

  updateCalibrationStateMachine(nowMs, newWheelSample);
  if (newWheelSample) {
    updateSteering(nowMs);
  }
  updateBootButton(nowMs);
  updateRumble(nowMs, connected);

  // ---- EMG sampling at a fixed rate ----
  static unsigned long lastSampleUs = 0;
  const unsigned long samplePeriodUs = 1000000UL / SAMPLE_RATE;
  if (nowUs - lastSampleUs >= samplePeriodUs) {
    lastSampleUs += samplePeriodUs;

    // If something held the loop up, do not catch up with a burst of samples.
    // Start counting again from now instead.
    if (nowUs - lastSampleUs > samplePeriodUs * 4) {
      lastSampleUs = nowUs;
    }

    // Read only 2 EMG channels
    int raw1 = analogRead(INPUT_PIN1);
    int raw2 = analogRead(INPUT_PIN2);

    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

    // Filter and envelope extraction for 2 channels
    float filtemg1 = emgfilters[0].process(filters[0].process(raw1));
    float filtemg2 = emgfilters[1].process(filters[1].process(raw2));

    float envelope1 = env1.getEnvelope(fabs(filtemg1));
    float envelope2 = env2.getEnvelope(fabs(filtemg2));

#ifdef ENABLE_EMG_ACCELERATOR
    // EMG 1 -> accelerator (right trigger)
    uint16_t accelTrigger = envelopeToTrigger(envelope1);
    gamepad->setRightTrigger(accelTrigger);
    if (accelTrigger > 0) lastCmdSentMs = nowMs;
#endif

#ifdef ENABLE_EMG_BRAKE
    // EMG 2 -> brake (left trigger)
    uint16_t brakeTrigger = envelopeToTrigger(envelope2);
    gamepad->setLeftTrigger(brakeTrigger);
    if (brakeTrigger > 0) lastCmdSentMs = nowMs;
#endif
  }

  // ---- Send one report at a fixed rate ----
  if (connected && (nowMs - lastReportMs >= REPORT_INTERVAL_MS)) {
    lastReportMs = nowMs;
    gamepad->sendGamepadReport();
  }

  // ---- Status leds last, so they show the state from this loop ----
  updateStatusLeds(connected, nowMs);
}
