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

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech
// Copyright (c) 2026 Varun Patil - vap05072006@gmail.com

// Core includes
#include <Arduino.h>
#include <Wire.h>
#include <BleCombo.h>

// ── MPU6050 Includes ──
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ══════════════════════════════════════════════════════════════════════════════
// ─── CONTROL MAPPING CONFIGURATION ───
// ══════════════════════════════════════════════════════════════════════════════
// Head movement (accelerometer angles) -> Mouse cursor movement
// Double blink -> Left mouse click
// Triple blink -> Right mouse click

// ══════════════════════════════════════════════════════════════════════════════
// ─── EASY-TO-ADJUST MOUSE CONTROL SETTINGS ───
// ══════════════════════════════════════════════════════════════════════════════

//  BASIC SETTINGS (ADJUST THESE TO FINE-TUNE)
#define MOUSE_UPDATE_RATE 12  // Update frequency: LOWER = faster updates (8-20)
#define DEADZONE 4.0          // Rest zone: HIGHER = easier to stop (3.0-8.0) degrees
#define MIN_SENSITIVITY 0.15  // Slowest speed: LOWER = more precise (0.1-0.5)
#define MAX_SENSITIVITY 8.0   // Fastest speed: LOWER = more controlled (4.0-15.0)

//  CALIBRATION SETTINGS
#define ACC_BIAS_SAMPLES 200  // samples averaged for bias at rest

//  SMOOTHING SETTINGS (FOR RESPONSIVENESS)
#define MOVEMENT_SMOOTHING 0.70  // Movement filter: LOWER = more responsive (0.5-0.85)
#define VELOCITY_DECAY 0.80      // Stop speed: LOWER = stops faster (0.7-0.9)
#define STOP_THRESHOLD 0.2       // Complete stop point: LOWER = stops sooner (0.1-0.5)

//  ACCELERATION SETTINGS
#define ACCEL_CURVE 2.5       // Acceleration curve: HIGHER = faster acceleration (1.5-4.0)
#define ACCEL_MULTIPLIER 2.8  // Acceleration strength: HIGHER = more acceleration (2.0-4.0)

//  RANGE SETTINGS
#define MAX_TILT_ANGLE 20.0  // Maximum head tilt angle (15.0-30.0) degrees

// ══════════════════════════════════════════════════════════════════════════════

// ── VIBRATION MOTOR PIN ──
#define VIBRATION_PIN 7  // Vibration motor for calibration feedback

// ─── MPU6050 ───
Adafruit_MPU6050 mpu;

float neutralPitch = 0;
float neutralRoll = 0;
float mouseVelocityX = 0, mouseVelocityY = 0;
bool isMPUCalibrated = false;
unsigned long lastMouseUpdate = 0;
bool axisCalibrated = false;

// ── NON-BLOCKING CALIBRATION STATE MACHINE ──
enum CalibrationState {
  CAL_IDLE,
  CAL_INIT_WAIT,
  CAL_UP_VIBRATE,
  CAL_UP_WAIT,
  CAL_LEFT_VIBRATE,
  CAL_LEFT_WAIT,
  CAL_NEUTRAL_SAMPLE,
  CAL_COMPLETE
};

CalibrationState calState = CAL_IDLE;
unsigned long calStateStartTime = 0;

// ── LEARNED AXIS MAPPING ──
int xAxisIndex = 0;
int xAxisSign = 1;  // side axis (left gesture resolves this)
int yAxisIndex = 1;
int yAxisSign = 1;  // forward axis (up gesture resolves this)

// ── BIAS SAMPLING ──
int biasSampleCount = 0;
float biasSumAcc[3] = { 0, 0, 0 };
float calStartAcc[3] = { 0, 0, 0 };

// ── CACHED SENSOR DATA ──
float readingsAcc[3] = { 0, 0, 0 };

// ── SUB-PIXEL ACCUMULATION ──
float mouseAccumX = 0, mouseAccumY = 0;

// ─── EEG Signal processing config ───
#define SAMPLE_RATE 512
#define INPUT_PIN1 A0  // EEG input

// EEG Envelope Configuration for blink detection
#define ENVELOPE_WINDOW_MS 100
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

// Double/Triple Blink Configuration
const unsigned long BLINK_DEBOUNCE_MS = 250;
const unsigned long DOUBLE_BLINK_MS = 600;
unsigned long lastBlinkTime = 0;
unsigned long firstBlinkTime = 0;
unsigned long secondBlinkTime = 0;
unsigned long triple_blink_ms = 800;
int blinkCount = 0;
bool blinkActive = false;

float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = { 0 };
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;
float BlinkThreshold = 50.0;

// ─── FILTERS ───
// Band-Stop Butterworth IIR digital filter (50Hz notch)
class NotchFilter {
private:
  struct BiquadState {
    float z1 = 0, z2 = 0;
  };
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
} eegNotchFilter;

// High-Pass Butterworth IIR digital filter (for EOG/blinks)
class EOGFilter {
private:
  struct BiquadState {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;

public:
  float process(float input) {
    float output = input;

    float x0 = output - (-1.91327599f * state0.z1) - (0.91688335f * state0.z2);
    output = 0.95753983f * x0 + -1.91507967f * state0.z1 + 0.95753983f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    return output;
  }

  void reset() {
    state0.z1 = state0.z2 = 0;
  }
} eogFilter;

// Low-Pass Butterworth IIR digital filter
class EEGFilter {
private:
  struct BiquadState {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;

public:
  float process(float input) {
    float output = input;

    float x0 = output - (-1.24200128f * state0.z1) - (0.45885207f * state0.z2);
    output = 0.05421270f * x0 + 0.10842539f * state0.z1 + 0.05421270f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    return output;
  }

  void reset() {
    state0.z1 = state0.z2 = 0;
  }
} eegFilter;

float updateEnvelope(float sample) {
  float absSample = fabsf(sample);
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return envelopeSum / ENVELOPE_WINDOW_SIZE;
}

// ─── VIBRATION FEEDBACK FUNCTIONS ───
void startVibration() {
  digitalWrite(VIBRATION_PIN, HIGH);
}

void stopVibration() {
  digitalWrite(VIBRATION_PIN, LOW);
}

// Pick dominant axis from an accumulated gesture vector
void resolveAxis(float diff[3], int &axisIndex, int &axisSign) {
  int a = 0;
  if (fabs(diff[1]) > fabs(diff[a])) a = 1;
  if (fabs(diff[2]) > fabs(diff[a])) a = 2;
  axisIndex = a;
  axisSign = (diff[a] > 0) ? 1 : -1;
}

void updateCalibrationStateMachine(unsigned long nowMs) {
  if (calState == CAL_IDLE || calState == CAL_COMPLETE) return;

  unsigned long elapsed = nowMs - calStateStartTime;

  switch (calState) {
    case CAL_INIT_WAIT:
      if (elapsed >= 3000) {
        calStartAcc[0] = readingsAcc[0];
        calStartAcc[1] = readingsAcc[1];
        calStartAcc[2] = readingsAcc[2];
        calState = CAL_UP_VIBRATE;
        calStateStartTime = nowMs;
        startVibration();
      }
      break;

    case CAL_UP_VIBRATE:
      if (elapsed >= 3000) {
        float d[3] = { 0 };
        d[0] = readingsAcc[0] - calStartAcc[0];
        d[1] = readingsAcc[1] - calStartAcc[1];
        d[2] = readingsAcc[2] - calStartAcc[2];
        resolveAxis(d, yAxisIndex, yAxisSign);
        stopVibration();
        calState = CAL_UP_WAIT;
        calStateStartTime = nowMs;
      }
      break;

    case CAL_UP_WAIT:
      if (elapsed >= 3000) {
        calStartAcc[0] = readingsAcc[0];
        calStartAcc[1] = readingsAcc[1];
        calStartAcc[2] = readingsAcc[2];
        calState = CAL_LEFT_VIBRATE;
        calStateStartTime = nowMs;
        startVibration();
      }
      break;

    case CAL_LEFT_VIBRATE:
      if (elapsed >= 3000) {
        float d[3] = { 0 };
        d[0] = readingsAcc[0] - calStartAcc[0];
        d[1] = readingsAcc[1] - calStartAcc[1];
        d[2] = readingsAcc[2] - calStartAcc[2];
        resolveAxis(d, xAxisIndex, xAxisSign);
        xAxisSign = -xAxisSign;
        stopVibration();
        if (xAxisIndex == yAxisIndex) {
          calState = CAL_LEFT_WAIT;
          calStateStartTime = nowMs;
          startVibration();
          break;
        }
        axisCalibrated = true;
        calState = CAL_LEFT_WAIT;
        calStateStartTime = nowMs;
      }
      break;

    case CAL_LEFT_WAIT:
      if (elapsed >= 2000) {
        biasSampleCount = 0;
        biasSumAcc[0] = biasSumAcc[1] = biasSumAcc[2] = 0;
        calState = CAL_NEUTRAL_SAMPLE;
        calStateStartTime = nowMs;
      }
      break;

    case CAL_NEUTRAL_SAMPLE:
      if (biasSampleCount < ACC_BIAS_SAMPLES) {
        biasSumAcc[0] += readingsAcc[0];
        biasSumAcc[1] += readingsAcc[1];
        biasSumAcc[2] += readingsAcc[2];
        biasSampleCount++;
      } else {
        float accBias[3] = { 0 };
        accBias[0] = biasSumAcc[0] / ACC_BIAS_SAMPLES;
        accBias[1] = biasSumAcc[1] / ACC_BIAS_SAMPLES;
        accBias[2] = biasSumAcc[2] / ACC_BIAS_SAMPLES;
        int gravAxis = 3 - xAxisIndex - yAxisIndex;
        float fwd = accBias[yAxisIndex] * yAxisSign;
        float side = accBias[xAxisIndex] * xAxisSign;
        float grav = accBias[gravAxis];
        neutralPitch = atan2(-fwd, sqrt(side * side + grav * grav)) * 180.0 / PI;
        neutralRoll = atan2(side, sqrt(fwd * fwd + grav * grav)) * 180.0 / PI;
        isMPUCalibrated = true;
        calState = CAL_COMPLETE;

        for (int i = 0; i < 3; i++) {
          startVibration();
          delay(100);
          stopVibration();
          delay(100);
        }
      }
      break;

    default:
      break;
  }
}

void getAccelerometerAngles(float &pitch, float &roll) {
  int gravAxis = 3 - xAxisIndex - yAxisIndex;
  float fwd = readingsAcc[yAxisIndex] * yAxisSign;
  float side = readingsAcc[xAxisIndex] * xAxisSign;
  float grav = readingsAcc[gravAxis];
  pitch = atan2(-fwd, sqrt(side * side + grav * grav)) * 180.0 / PI;
  roll = atan2(side, sqrt(fwd * fwd + grav * grav)) * 180.0 / PI;
}

float mapAngleToMouse(float angle) {
  float absAngle = fabs(angle);
  float sign = (angle > 0) ? 1.0f : -1.0f;
  if (absAngle <= DEADZONE) return 0;
  float norm = constrain(absAngle / (MAX_TILT_ANGLE - DEADZONE), 0.0f, 1.0f);
  float accel = pow(norm, ACCEL_CURVE);
  float speed = MIN_SENSITIVITY + (MAX_SENSITIVITY - MIN_SENSITIVITY) * accel * ACCEL_MULTIPLIER;
  return sign * speed;
}

void updatePrecisionMouse(unsigned long nowMs) {
  if (!isMPUCalibrated || !axisCalibrated) return;
  if (nowMs - lastMouseUpdate < MOUSE_UPDATE_RATE) return;
  lastMouseUpdate = nowMs;

  float currentPitch, currentRoll;
  getAccelerometerAngles(currentPitch, currentRoll);

  float deltaPitch = currentPitch - neutralPitch;
  float deltaRoll = currentRoll - neutralRoll;

  if (fabs(deltaPitch) < DEADZONE) deltaPitch = 0;
  if (fabs(deltaRoll) < DEADZONE) deltaRoll = 0;

  float targetVelX = mapAngleToMouse(deltaRoll);
  float targetVelY = mapAngleToMouse(deltaPitch);

  mouseVelocityX = MOVEMENT_SMOOTHING * mouseVelocityX + (1.0f - MOVEMENT_SMOOTHING) * targetVelX;
  mouseVelocityY = MOVEMENT_SMOOTHING * mouseVelocityY + (1.0f - MOVEMENT_SMOOTHING) * targetVelY;

  if (fabs(targetVelX) < 0.01f) {
    mouseVelocityX *= VELOCITY_DECAY;
    if (fabs(mouseVelocityX) < STOP_THRESHOLD) mouseVelocityX = 0;
  }
  if (fabs(targetVelY) < 0.01f) {
    mouseVelocityY *= VELOCITY_DECAY;
    if (fabs(mouseVelocityY) < STOP_THRESHOLD) mouseVelocityY = 0;
  }

  mouseAccumX += mouseVelocityX;
  mouseAccumY += mouseVelocityY;
  int finalMouseX = (int)mouseAccumX;
  int finalMouseY = (int)mouseAccumY;
  mouseAccumX -= finalMouseX;
  mouseAccumY -= finalMouseY;

  if (finalMouseX != 0 || finalMouseY != 0) {
    Mouse.move(finalMouseX, finalMouseY);
  }
}

// ========== BLINK DETECTION (double blink = left click, triple blink = right click) ==========
void handleBlinks(unsigned long nowMs) {
  bool envelopeHigh = currentEEGEnvelope > BlinkThreshold;
  if (!blinkActive && envelopeHigh && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
    lastBlinkTime = nowMs;
    if (blinkCount == 0) {
      firstBlinkTime = nowMs;
      blinkCount = 1;
    } else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
      secondBlinkTime = nowMs;
      blinkCount = 2;
    } else if (blinkCount == 2 && (nowMs - secondBlinkTime) <= triple_blink_ms) {
      Mouse.click(MOUSE_RIGHT);
      blinkCount = 0;
    } else {
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
    blinkActive = true;
  }

  if (!envelopeHigh) {
    blinkActive = false;
  }

  // Double blink timeout -> Left mouse click
  if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms) {
    Mouse.click(MOUSE_LEFT);
    blinkCount = 0;
  }
  // Single blink timeout
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
    blinkCount = 0;
  }
}

// ─── setup() ───
void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(22, 23);
  while (!mpu.begin()) {
    Serial.println("MPU6050 initialization FAILED!");
    delay(500);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(INPUT_PIN1, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);

  Keyboard.begin();
  Mouse.begin();

  for (int i = 0; i < 2; i++) {
    startVibration();
    delay(100);
    stopVibration();
    delay(100);
  }

  calState = CAL_INIT_WAIT;
  calStateStartTime = millis();
}

// ─── loop() ───
void loop() {
  bool connected = Keyboard.isConnected();

  static unsigned long lastMicros = micros();

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;
  static long timer = 0;
  timer -= dt;

  if (timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;
    unsigned long nowMs = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    readingsAcc[0] = a.acceleration.x;
    readingsAcc[1] = a.acceleration.y;
    readingsAcc[2] = a.acceleration.z;

    updateCalibrationStateMachine(nowMs);

    int raw1 = analogRead(INPUT_PIN1);
    float filteeg = eegFilter.process(eegNotchFilter.process(raw1));
    float filteog = eogFilter.process(filteeg);
    currentEEGEnvelope = updateEnvelope(filteog);

    if (connected) {
      handleBlinks(nowMs);
    }
  }

  // PRECISION MOUSE CONTROL (ACCEL ANGLE BASED) - runs continuously
  if (connected) {
    updatePrecisionMouse(millis());
  }
}