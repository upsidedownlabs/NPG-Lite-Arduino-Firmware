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
// Head movement (gyro rate) -> Mouse cursor movement
// Double blink -> Left mouse click
// Triple blink -> Right mouse click

// ══════════════════════════════════════════════════════════════════════════════
// ─── EASY-TO-ADJUST MOUSE CONTROL SETTINGS ───
// ══════════════════════════════════════════════════════════════════════════════

//  BASIC SETTINGS (ADJUST THESE TO FINE-TUNE)
#define MOUSE_UPDATE_RATE 8    // Update frequency: LOWER = faster updates (8-20)
#define MIN_SENSITIVITY 0.5    // Slowest speed: LOWER = more precise (0.1-0.5)
#define MAX_SENSITIVITY 10.0   // Fastest speed: LOWER = more controlled (4.0-15.0)

//  PRECISION SETTINGS (FOR MINUTE MOVEMENTS)
#define PRECISION_ZONE 4.0         // Precision angle range: HIGHER = more precision zone (1.0-4.0)
#define PRECISION_MULTIPLIER 0.1  // Precision sensitivity: LOWER = more precise (0.2-0.6)

//  SMOOTHING SETTINGS (FOR RESPONSIVENESS)
#define MOVEMENT_SMOOTHING 0.80  // Movement filter: LOWER = more responsive (0.5-0.85)
#define VELOCITY_DECAY 0.80      // Stop speed: LOWER = stops faster (0.7-0.9)
#define STOP_THRESHOLD 0.2       // Complete stop point: LOWER = stops sooner (0.1-0.5)

//  ACCELERATION SETTINGS
#define ACCEL_CURVE 2.9       // Acceleration curve: HIGHER = faster acceleration (1.5-4.0)
#define ACCEL_MULTIPLIER 2.5  // Acceleration strength: HIGHER = more acceleration (2.0-4.0)

//  RANGE / GYRO SETTINGS (deg/s)
#define MAX_RATE 60.0          // deg/s mapped to MAX_SENSITIVITY
#define GYRO_DEADZONE 5.0      // deg/s below this = no movement
#define GYRO_BIAS_SAMPLES 200  // samples averaged for bias at rest

// ══════════════════════════════════════════════════════════════════════════════

// ── VIBRATION MOTOR PIN ──
#define VIBRATION_PIN 7  // Vibration motor for calibration feedback

// ── DEBUG ENABLE ──
#define DEBUG_ENABLE 1  // Set to 1 to enable debug prints, 0 to disable

// ─── MPU6050 ───
Adafruit_MPU6050 mpu;

bool isIMUCalibrated = false;
unsigned long lastMouseUpdate = 0;

// ── GYRO BIAS (subtracted from every reading) ──
float gyroBias[3] = { 0, 0, 0 };

// ── LEARNED AXIS MAPPING ──
// which raw gyro axis (0=X,1=Y,2=Z) and sign drives each cursor axis
int xAxisIndex = 0;
int xAxisSign = 1;  // horizontal (head turn)
int yAxisIndex = 1;
int yAxisSign = 1;  // vertical   (head nod)
bool axisCalibrated = false;

// ── GESTURE ACCUMULATION (during calibration) ──
float gestureSum[3] = { 0, 0, 0 };
unsigned long lastGyroMicros = 0;

// ── BIAS SAMPLING ──
int biasSampleCount = 0;
float biasSum[3] = { 0, 0, 0 };

// ── CACHED GYRO DATA (read ONCE per loop tick, reused everywhere) ──
float readingsGyro[3] = { 0, 0, 0 };

// ── VELOCITY SMOOTHING ──
float smoothRateX = 0, smoothRateY = 0;
float mouseVelX = 0, mouseVelY = 0;
float mouseAccumX = 0, mouseAccumY = 0;

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

// ─── DEBUG FUNCTIONS ───
void debugPrint(const char *message) {
#if DEBUG_ENABLE
  Serial.println(message);
#endif
}

void debugPrint(String message) {
#if DEBUG_ENABLE
  Serial.println(message);
#endif
}

void debugPrintValue(const char *label, float value) {
#if DEBUG_ENABLE
  Serial.print(label);
  Serial.print(": ");
  Serial.println(value);
#endif
}

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

float updateEEGEnvelope(float sample) {
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
  debugPrint("Vibration ON");
}

void stopVibration() {
  digitalWrite(VIBRATION_PIN, LOW);
  debugPrint("Vibration OFF");
}

// Pick dominant axis from an accumulated gesture vector
void resolveAxis(float sum[3], int &axisIndex, int &axisSign) {
  int a = 0;
  if (fabs(sum[1]) > fabs(sum[a])) a = 1;
  if (fabs(sum[2]) > fabs(sum[a])) a = 2;
  axisIndex = a;
  axisSign = (sum[a] > 0) ? 1 : -1;
}

void updateCalibrationStateMachine(unsigned long nowMs) {
  if (calState == CAL_IDLE || calState == CAL_COMPLETE)
    return;

  unsigned long elapsed = nowMs - calStateStartTime;
  float gx, gy, gz;

  switch (calState) {
    case CAL_INIT_WAIT:
      if (elapsed >= 3000) {
        gestureSum[0] = gestureSum[1] = gestureSum[2] = 0;
        lastGyroMicros = micros();
        calState = CAL_UP_VIBRATE;
        calStateStartTime = nowMs;
        startVibration();
      }
      break;

    case CAL_UP_VIBRATE:
      {
        unsigned long n = micros();
        float dt = (n - lastGyroMicros) / 1000000.0f;
        lastGyroMicros = n;
        gx = readingsGyro[0];
        gy = readingsGyro[1];
        gz = readingsGyro[2];
        gestureSum[0] += gx * dt;
        gestureSum[1] += gy * dt;
        gestureSum[2] += gz * dt;
        if (elapsed >= 3000) {
          stopVibration();
          resolveAxis(gestureSum, yAxisIndex, yAxisSign);
          yAxisSign = -yAxisSign;
          debugPrintValue("Y axis index", yAxisIndex);
          debugPrintValue("Y axis sign", yAxisSign);
          calState = CAL_UP_WAIT;
          calStateStartTime = nowMs;
        }
        break;
      }

    case CAL_UP_WAIT:
      if (elapsed >= 3000) {
        gestureSum[0] = gestureSum[1] = gestureSum[2] = 0;
        lastGyroMicros = micros();
        calState = CAL_LEFT_VIBRATE;
        calStateStartTime = nowMs;
        startVibration();
      }
      break;

    case CAL_LEFT_VIBRATE:
      {
        unsigned long n = micros();
        float dt = (n - lastGyroMicros) / 1000000.0f;
        lastGyroMicros = n;
        gx = readingsGyro[0];
        gy = readingsGyro[1];
        gz = readingsGyro[2];
        gestureSum[0] += gx * dt;
        gestureSum[1] += gy * dt;
        gestureSum[2] += gz * dt;
        if (elapsed >= 3000) {
          stopVibration();
          resolveAxis(gestureSum, xAxisIndex, xAxisSign);
          xAxisSign = -xAxisSign;
          debugPrintValue("X axis index", xAxisIndex);
          debugPrintValue("X axis sign", xAxisSign);
          if (xAxisIndex == yAxisIndex)
            debugPrint("Both axes coincide - Calibration FAILED");

          calState = CAL_LEFT_WAIT;
          calStateStartTime = nowMs;
          axisCalibrated = true;
          debugPrint("Axis Calibrated = TRUE");
        }
        break;
      }

    case CAL_LEFT_WAIT:
      if (elapsed >= 2000) {
        biasSampleCount = 0;
        biasSum[0] = biasSum[1] = biasSum[2] = 0;
        calState = CAL_NEUTRAL_SAMPLE;
        calStateStartTime = nowMs;
        debugPrint("LEFT_WAIT complete, moving to NEUTRAL_SAMPLE");
      }
      break;

    case CAL_NEUTRAL_SAMPLE:
      if (biasSampleCount < GYRO_BIAS_SAMPLES) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        biasSum[0] += g.gyro.x * RAD_TO_DEG;
        biasSum[1] += g.gyro.y * RAD_TO_DEG;
        biasSum[2] += g.gyro.z * RAD_TO_DEG;
        biasSampleCount++;

        if (biasSampleCount % 20 == 0)
          debugPrintValue("Bias sample count", biasSampleCount);
      } else {
        gyroBias[0] = biasSum[0] / biasSampleCount;
        gyroBias[1] = biasSum[1] / biasSampleCount;
        gyroBias[2] = biasSum[2] / biasSampleCount;
        smoothRateX = smoothRateY = 0;
        isIMUCalibrated = true;
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

float mapRateToMouse(float rate) {
  if (rate == 0) return 0;

  float absRate = fabs(rate);
  float sign = (rate > 0) ? 1.0f : -1.0f;
  float norm = constrain(absRate / MAX_RATE, 0.0f, 1.0f);

  float sensitivity;
  if (absRate <= PRECISION_ZONE) {
    sensitivity = MIN_SENSITIVITY * PRECISION_MULTIPLIER * (absRate / PRECISION_ZONE);
  } else {
    float accel = pow(norm, ACCEL_CURVE);
    sensitivity = MIN_SENSITIVITY + (MAX_SENSITIVITY - MIN_SENSITIVITY) * accel * ACCEL_MULTIPLIER;
  }
  return sign * sensitivity;
}

void updatePrecisionMouse(unsigned long nowMs) {
  if (!isIMUCalibrated || !axisCalibrated) {
#if DEBUG_ENABLE
    static bool lastPrintState = false;
    if (!isIMUCalibrated && !lastPrintState) {
      debugPrint("Waiting for calibration...");
      lastPrintState = true;
    }
#endif
    return;
  }

  if (nowMs - lastMouseUpdate < MOUSE_UPDATE_RATE)
    return;
  lastMouseUpdate = nowMs;

  float rateX = readingsGyro[xAxisIndex] * xAxisSign;
  float rateY = readingsGyro[yAxisIndex] * yAxisSign;

  if (fabs(rateX) < GYRO_DEADZONE) rateX = 0;
  if (fabs(rateY) < GYRO_DEADZONE) rateY = 0;

  smoothRateX = MOVEMENT_SMOOTHING * smoothRateX + (1.0f - MOVEMENT_SMOOTHING) * rateX;
  smoothRateY = MOVEMENT_SMOOTHING * smoothRateY + (1.0f - MOVEMENT_SMOOTHING) * rateY;

  float targetVelX = mapRateToMouse(smoothRateX);
  float targetVelY = mapRateToMouse(smoothRateY);

  mouseVelX = mouseVelX * 0.8f + targetVelX * 0.2f;
  mouseVelY = mouseVelY * 0.8f + targetVelY * 0.2f;

  if (rateX == 0) {
    mouseVelX *= VELOCITY_DECAY;
    if (fabs(mouseVelX) < STOP_THRESHOLD) mouseVelX = 0;
  }
  if (rateY == 0) {
    mouseVelY *= VELOCITY_DECAY;
    if (fabs(mouseVelY) < STOP_THRESHOLD) mouseVelY = 0;
  }

  mouseAccumX += mouseVelX;
  mouseAccumY += mouseVelY;
  int finalMouseX = (int)mouseAccumX;
  int finalMouseY = (int)mouseAccumY;
  mouseAccumX -= finalMouseX;
  mouseAccumY -= finalMouseY;

  Mouse.move(finalMouseX, finalMouseY);
#if DEBUG_ENABLE
  Serial.print(finalMouseX);
  Serial.print(" , ");
  Serial.println(finalMouseY);
#endif
}

// ========== BLINK DETECTION (double blink = left click, triple blink = right click) ==========
void handleBlinks(unsigned long nowMs) {
  bool envelopeHigh = currentEEGEnvelope > BlinkThreshold;
  if (!blinkActive && envelopeHigh && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
    lastBlinkTime = nowMs;
    if (blinkCount == 0) {
      firstBlinkTime = nowMs;
      blinkCount = 1;
      debugPrint("First blink detected");
    } else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
      secondBlinkTime = nowMs;
      blinkCount = 2;
      debugPrint("Second blink detected");
    } else if (blinkCount == 2 && (nowMs - secondBlinkTime) <= triple_blink_ms) {
      Mouse.click(MOUSE_RIGHT);
      blinkCount = 0;
      debugPrint("Triple blink - Right click!");
    } else {
      firstBlinkTime = nowMs;
      blinkCount = 1;
      debugPrint("Blink timeout - resetting");
    }
    blinkActive = true;
  }

  if (!envelopeHigh) {
    blinkActive = false;
  }

  if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms) {
    Mouse.click(MOUSE_LEFT);
    blinkCount = 0;
    debugPrint("Double blink - Left click!");
  }
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
    debugPrint("MPU6050 initialization FAILED!");
    delay(500);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  debugPrint("System Starting...");

  pinMode(INPUT_PIN1, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);
  debugPrint("Pins initialized");

  Keyboard.begin();
  Mouse.begin();
  debugPrint("BLE Combo initialized");

  debugPrint("MPU6050 initialized successfully!");

  for (int i = 0; i < 2; i++) {
    startVibration();
    delay(100);
    stopVibration();
    delay(100);
  }

  calState = CAL_INIT_WAIT;
  calStateStartTime = millis();
  debugPrint("Calibration started - Keep head still for 3 seconds");
}

// ─── loop() ───
void loop() {
  bool connected = Keyboard.isConnected();

  static unsigned long lastMicros = micros();
  unsigned long nowMs = millis();

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;
  static long timer = 0;
  timer -= dt;

  if (timer <= 0 && connected) {
    timer += 1000000L / SAMPLE_RATE;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    readingsGyro[0] = g.gyro.x * RAD_TO_DEG - gyroBias[0];
    readingsGyro[1] = g.gyro.y * RAD_TO_DEG - gyroBias[1];
    readingsGyro[2] = g.gyro.z * RAD_TO_DEG - gyroBias[2];

    updateCalibrationStateMachine(nowMs);

    int raw1 = analogRead(INPUT_PIN1);

    float notchFiltered = eegNotchFilter.process(raw1);
    float filteredEEG = eegFilter.process(notchFiltered);
    float filteredEOG = eogFilter.process(filteredEEG);
    currentEEGEnvelope = updateEEGEnvelope(filteredEOG);

    handleBlinks(nowMs);

#if DEBUG_ENABLE
    static int eegCounter = 0;
    if (eegCounter++ % 100 == 0) {
      Serial.print("EEG Envelope: ");
      Serial.print(currentEEGEnvelope);
      Serial.print(" | Threshold: ");
      Serial.println(BlinkThreshold);
    }
#endif
  }

  // PRECISION MOUSE CONTROL (GYRO RATE BASED) - runs continuously
  if (connected) {
    updatePrecisionMouse(nowMs);
  }

#if DEBUG_ENABLE
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 5000) {
    lastStatusPrint = millis();
    Serial.println("═══════════════════════════════════");
    Serial.print("BLE Connected: ");
    Serial.println(connected ? "YES" : "NO");
    Serial.print("IMU Calibrated: ");
    Serial.println(isIMUCalibrated ? "YES" : "NO");
    Serial.print("Axis Calibrated: ");
    Serial.println(axisCalibrated ? "YES" : "NO");
    Serial.print("Gyro Bias - X: ");
    Serial.print(gyroBias[0]);
    Serial.print(", Y: ");
    Serial.print(gyroBias[1]);
    Serial.print(", Z: ");
    Serial.println(gyroBias[2]);
    switch (calState) {
      case CAL_IDLE:           Serial.println("IDLE"); break;
      case CAL_INIT_WAIT:      Serial.println("INIT_WAIT"); break;
      case CAL_UP_VIBRATE:     Serial.println("UP_VIBRATE"); break;
      case CAL_UP_WAIT:        Serial.println("UP_WAIT"); break;
      case CAL_LEFT_VIBRATE:   Serial.println("LEFT_VIBRATE"); break;
      case CAL_LEFT_WAIT:      Serial.println("LEFT_WAIT"); break;
      case CAL_NEUTRAL_SAMPLE: Serial.println("NEUTRAL_SAMPLE"); break;
      case CAL_COMPLETE:       Serial.println("COMPLETE"); break;
    }
    Serial.println("═══════════════════════════════════");
  }
#endif
}
