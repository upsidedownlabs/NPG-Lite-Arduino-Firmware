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

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

// Core includes
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
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
#define MOUSE_UPDATE_RATE 8   // Update frequency: LOWER = faster updates (8-20)
#define MIN_SENSITIVITY 0.5   // Slowest speed: LOWER = more precise (0.1-0.5)
#define MAX_SENSITIVITY 10.0  // Fastest speed: LOWER = more controlled (4.0-15.0)

//  PRECISION SETTINGS (FOR MINUTE MOVEMENTS)
#define PRECISION_ZONE 4.0        // Precision angle range: HIGHER = more precision zone (1.0-4.0)
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
#define VIBRATION_PIN 7 // Vibration motor for calibration feedback
#define PIN_NEOPIXEL 15
#define BATTERY_VOLTAGE_PIN A6
#define BLUE_LED_DURATION 100

Adafruit_NeoPixel pixel(6, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5
#define IMU_LED 3

// ── DEBUG ENABLE ──
#define DEBUG_ENABLE 1  // Set to 1 to enable debug prints, 0 to disable

// ─── MPU6050 ───
Adafruit_MPU6050 mpu;
uint32_t mpuAddress = 0x68;

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
enum CalibrationState
{
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
  float process(float input)
  {
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

  void reset()
  {
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
  float process(float input)
  {
    float output = input;

    float x0 = output - (-1.91327599f * state0.z1) - (0.91688335f * state0.z2);
    output = 0.95753983f * x0 + -1.91507967f * state0.z1 + 0.95753983f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    return output;
  }

  void reset()
  {
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
void startVibration()
{
  digitalWrite(VIBRATION_PIN, HIGH);
  debugPrint("Vibration ON");
}

void stopVibration()
{
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

// ── BLE LED state machine ──
enum LedState
{
  LED_RED,
  LED_GREEN,
  LED_BLUE_FADE
};
LedState ledState = LED_RED;
unsigned long lastCmdSentMs = 0;
uint32_t lastPixel0Color = 0xFFFFFFFF;
static bool pixelDirty = false;

// ─── NON-BLOCKING CALIBRATION STATE MACHINE ───
void updateCalibrationStateMachine(unsigned long nowMs)
{
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

// ── Battery level ──
static const unsigned long BATTERY_CHECK_INTERVAL = 10000;
static unsigned long lastBatteryCheck = -10000;
uint32_t batteryColor = 0;
static uint32_t batteryWinSum = 0;
static uint16_t batteryWinCount = 0;
static int lastBatteryPct = -1;
static uint8_t risingCount = 0;
static const uint8_t RISING_THRESHOLD = 3;
const float voltageLUT[] = {
    3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
    3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20};
const int percentLUT[] = {
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
    50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

float interpolatePercentage(float voltage)
{
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

int getCurrentBatteryPercentage()
{
  float avgRaw = (batteryWinCount > 0) ? (batteryWinSum / batteryWinCount) : analogRead(BATTERY_VOLTAGE_PIN);
  batteryWinSum = 0;
  batteryWinCount = 0;
  float voltage = (avgRaw / 1000.0) * 2;
  voltage += 0.022;
  float percentage = interpolatePercentage(voltage);
  if (lastBatteryPct == -1)
  {
    lastBatteryPct = (int)percentage;
  }
  else if ((int)percentage < lastBatteryPct)
  {
    lastBatteryPct = (int)percentage;
    risingCount = 0;
  }
  else if ((int)percentage > lastBatteryPct)
  {
    risingCount++;
    if (risingCount >= RISING_THRESHOLD)
    {
      lastBatteryPct = (int)percentage;
      risingCount = 0;
    }
  }
  else
  {
    risingCount = 0;
  }
  return lastBatteryPct;
}

// ========== BLINK DETECTION (double blink = left click, triple blink = right click) ==========
void handleBlinks(unsigned long nowMs)
{
  bool envelopeHigh = currentEEGEnvelope > BlinkThreshold;
  if (!blinkActive && envelopeHigh && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS)
  {
    lastBlinkTime = nowMs;
    if (blinkCount == 0)
    {
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS)
    {
      secondBlinkTime = nowMs;
      blinkCount = 2;
    }
    else if (blinkCount == 2 && (nowMs - secondBlinkTime) <= triple_blink_ms)
    {
      Mouse.click(MOUSE_RIGHT);
      if (Keyboard.isConnected())
      {
        lastCmdSentMs = millis();
        ledState = LED_BLUE_FADE;
      }
      blinkCount = 0;
    }
    else
    {
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
    blinkActive = true;
  }

  if (!envelopeHigh)
  {
    blinkActive = false;
  }

  // Double blink timeout -> Left mouse click
  if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms)
  {
    Mouse.click(MOUSE_LEFT);
    if (Keyboard.isConnected())
    {
      lastCmdSentMs = millis();
      ledState = LED_BLUE_FADE;
    }
    blinkCount = 0;
  }
  // Single blink timeout
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS)
  {
    blinkCount = 0;
  }
}

// Update IMU I2C led
void updateIMULed(bool connectionStatus)
{
  uint32_t color;

  if (!connectionStatus)
  {
    color = pixel.Color(20, 0, 0); // Red = IMU communication failed
  }
  else
  {
    color = pixel.Color(0, 20, 0); // Green = ready
  }
  pixel.setPixelColor(IMU_LED, color);
  pixel.show();
}

void updateBLELed()
{
  uint32_t color;
  if (ledState == LED_RED)
  {
    color = pixel.Color(20, 0, 0);
  }
  else if (ledState == LED_GREEN)
  {
    color = pixel.Color(0, 20, 0);
  }
  else
  {
    unsigned long elapsed = millis() - lastCmdSentMs;
    if (elapsed < BLUE_LED_DURATION)
    {
      color = pixel.Color(0, 0, 30);
    }
    else
    {
      ledState = LED_GREEN;
      color = pixel.Color(0, 20, 0);
    }
  }
  if (color != lastPixel0Color)
  {
    lastPixel0Color = color;
    pixel.setPixelColor(BLE_LED, color);
    pixel.show();
  }}

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
>>>>>>> 13f5535 (Added NPG-Mouse sketch for MPU6050 as well and tuned calibration factors for them)
  }
}

// ─── setup() ───
void setup()
{
  Serial.begin(115200);
  pixel.begin();
  pixel.clear();
  pixel.show();

  Wire.begin(22, 23);
  // Initialize MPU6050 failed

  int currentBattery = getCurrentBatteryPercentage();
  if (currentBattery <= 20)
  {
    batteryColor = pixel.Color(20, 0, 0);
  }
  else if (currentBattery <= 70)
  {
    batteryColor = pixel.Color(35, 7, 0);
  }
  else
  {
    batteryColor = pixel.Color(0, 20, 0);
  }
  pixel.setPixelColor(BATTERY_LED, batteryColor);

  while (!mpu.begin())
  {
    Serial.print("MPU6050 initialization FAILED!");
    static uint16_t fader = 100;
    static bool decreasing = true;
    pixel.setPixelColor(IMU_LED, pixel.Color(fader, 0, 0));
    pixel.show();
    delay(20);
    if (decreasing)
    {
      fader = fader - 2;
      if (fader < 10)
      {
        decreasing = false;
      }
    }
    else
    {
      fader = fader + 2;
      if (fader > 100)
      {
        decreasing = true;
      }
    }
  }

  // MPU initialized
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // START NON-BLOCKING CALIBRATION
  calState = CAL_INIT_WAIT;
  calStateStartTime = millis();

  pinMode(INPUT_PIN1, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);
  debugPrint("Pins initialized");

  Keyboard.begin();
  Mouse.begin();
}

// ─── loop() ───
void loop()
{
  bool connected = Keyboard.isConnected();
  static bool lastConnected = false;
  if (connected != lastConnected)
  {
    lastConnected = connected;
    ledState = connected ? LED_GREEN : LED_RED;
    pixelDirty = true;
  }

  Wire.beginTransmission(mpuAddress);

  bool imuConnected;
  if (!Wire.endTransmission())
  {
    imuConnected = true;
  }
  else
  {
    imuConnected = false;
  }
  static bool lastConnection = false;
  if (imuConnected != lastConnection)
  {
    lastConnection = imuConnected;
    updateIMULed(imuConnected);
    pixelDirty = true;
    if (!imuConnected)
    {
      isMPUCalibrated = false;
      axisCalibrated = false;
      calState = CAL_IDLE;

      mouseVelocityX = 0;
      mouseVelocityY = 0;

      eegNotchFilter.reset();
      eogFilter.reset();
      eegFilter.reset();
      Serial.print("MPU disconnected - calibration invalidated");
    }
    else
    {
      if (mpu.begin())
      {
        Serial.print("MPU reconnected - restarting calibration");
        calState = CAL_INIT_WAIT;
        calStateStartTime = millis();
      }
      else
      {
        Serial.print("MPU reconnected but beginI2C() failed");
      }
    }
  }

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
    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

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

  // 4) PRECISION MOUSE CONTROL - runs continuously
  if (connected)
  {
    updatePrecisionMouse(millis());
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL)
  {
    int currentBattery = getCurrentBatteryPercentage();
    if (currentBattery <= 20)
    {
      batteryColor = pixel.Color(20, 0, 0);
    }
    else if (currentBattery <= 70)
    {
      batteryColor = pixel.Color(35, 7, 0);
    }
    else
    {
      batteryColor = pixel.Color(0, 20, 0);
    }
    pixelDirty = true;
    lastBatteryCheck = currentMillis;
  }

  if (pixelDirty)
  {
    pixel.setPixelColor(BATTERY_LED, batteryColor);
    pixel.show();
    pixelDirty = false;
  }
  updateBLELed();
}
