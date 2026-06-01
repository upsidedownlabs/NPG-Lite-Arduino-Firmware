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

// Copyright (c) 2025 Aman Maheshwari - aman@upsidedownlabs.tech
// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech
// Copyright (c) 2026 Varun Patil - vap05072006@gmail.com

// Core includes
#include <Arduino.h>
#include <Wire.h>
#include <BleCombo.h>
#include <Adafruit_NeoPixel.h>

// ── BMI270 Includes (with Accelerometer) ──
#include <SparkFun_BMI270_Arduino_Library.h>

#define PIXEL_PIN 15
#define PIXEL_COUNT 6
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5
#define IMU_LED 3
#define BLUE_LED_DURATION 100
uint32_t imuAddress = 0x68;

// ══════════════════════════════════════════════════════════════════════════════
// ─── CONTROL MAPPING CONFIGURATION ───
// ══════════════════════════════════════════════════════════════════════════════
// Head movement (accelerometer) -> Mouse cursor movement (POSITION HOLD)
// Jaw Clench -> Left mouse click
// Triple blink -> Right mouse click

// ══════════════════════════════════════════════════════════════════════════════
// ─── EASY-TO-ADJUST MOUSE CONTROL SETTINGS ───
// ══════════════════════════════════════════════════════════════════════════════

//  BASIC SETTINGS (ADJUST THESE TO FINE-TUNE)
#define MOUSE_UPDATE_RATE 12 // Update frequency: LOWER = faster updates (8-20)
#define DEADZONE 0.3         // Rest zone: HIGHER = easier to stop (0.3-2.0) degrees
#define MIN_SENSITIVITY 0.15 // Slowest speed: LOWER = more precise (0.1-0.5)
#define MAX_SENSITIVITY 8.0  // Fastest speed: LOWER = more controlled (4.0-15.0)

//  PRECISION SETTINGS (FOR MINUTE MOVEMENTS)
#define PRECISION_ZONE 4.0       // Precision angle range: HIGHER = more precision zone (1.0-4.0)
#define PRECISION_MULTIPLIER 0.1 // Precision sensitivity: LOWER = more precise (0.2-0.6)

//  SMOOTHING SETTINGS (FOR RESPONSIVENESS)
#define MOVEMENT_SMOOTHING 0.70 // Movement filter: LOWER = more responsive (0.5-0.85)
#define VELOCITY_DECAY 0.80     // Stop speed: LOWER = stops faster (0.7-0.9)
#define STOP_THRESHOLD 0.2      // Complete stop point: LOWER = stops sooner (0.1-0.5)

//  ACCELERATION SETTINGS
#define ACCEL_CURVE 2.5      // Acceleration curve: HIGHER = faster acceleration (1.5-4.0)
#define ACCEL_MULTIPLIER 2.8 // Acceleration strength: HIGHER = more acceleration (2.0-4.0)

//  RANGE SETTINGS
#define MAX_TILT_ANGLE 20.0 // Maximum head tilt: LOWER = shorter range (15.0-30.0)

// ===== JAW CLENCH CONFIGURATION =====
#define JAW_THRESHOLD 40.0     // Jaw clench detection threshold
#define JAW_DEBOUNCE_MS 500    // Debounce time for jaw clench
#define JAW_OFF_THRESHOLD 30.0 // Hysteresis: must fall below this to re-arm

// ══════════════════════════════════════════════════════════════════════════════

// ── VIBRATION MOTOR PIN ──
#define VIBRATION_PIN 7 // Vibration motor for calibration feedback

// ── DEBUG ENABLE ──
#define DEBUG_ENABLE 1 // Set to 1 to enable debug prints, 0 to disable

// ─── BMI270 Variables (using Accelerometer) ───
BMI270 imu;

// Mouse control variables with velocity-based stopping
float neutralPitch = 0, neutralRoll = 0;
float smoothedPitch = 0, smoothedRoll = 0;
float mouseVelocityX = 0, mouseVelocityY = 0; // Velocity for smooth stopping
float lastDeltaPitch = 0, lastDeltaRoll = 0;  // Track previous frame deltas
bool isIMUCalibrated = false;
unsigned long lastMouseUpdate = 0;

// ── AXIS CALIBRATION VARIABLES ──
int pitchDirection = 1; // 1 = normal, -1 = inverted
int rollDirection = 1;  // 1 = normal, -1 = inverted
bool axisCalibrated = false;

// ── NON-BLOCKING CALIBRATION STATE MACHINE ──
enum CalibrationState
{
  CAL_IDLE,
  CAL_INIT_WAIT,
  CAL_UP_VIBRATE,
  CAL_UP_WAIT,
  CAL_CENTER_WAIT1,
  CAL_LEFT_VIBRATE,
  CAL_LEFT_WAIT,
  CAL_CENTER_WAIT2,
  CAL_NEUTRAL_SAMPLE,
  CAL_COMPLETE
};

CalibrationState calState = CAL_IDLE;
unsigned long calStateStartTime = 0;
float calStartPitch = 0, calEndPitch = 0;
float calStartRoll = 0, calEndRoll = 0;
int neutralSampleCount = 0;
float neutralPitchSum = 0, neutralRollSum = 0;

// ─── EEG Signal processing config ───
#define SAMPLE_RATE 512
#define INPUT_PIN1 A0 // EEG input only (also used for jaw clench)

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

// Jaw clench variables
unsigned long lastJawClenchTime = 0;
bool jawState = false;           // true = currently in a clench
bool jawClenchTriggered = false; // true if clench already triggered for current press

float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;
float BlinkThreshold = 50.0;

// Jaw envelope buffer (separate for jaw detection)
float jawEnvelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int jawEnvelopeIndex = 0;
float jawEnvelopeSum = 0;
float currentJawEnvelope = 0;

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

#define BATTERY_VOLTAGE_PIN A6
#define BATTERY_LED 5
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

// ─── DEBUG FUNCTION ───
void debugPrint(const char *message)
{
#if DEBUG_ENABLE
  Serial.println(message);
#endif
}

void debugPrint(String message)
{
#if DEBUG_ENABLE
  Serial.println(message);
#endif
}

void debugPrintValue(const char *label, float value)
{
#if DEBUG_ENABLE
  Serial.print(label);
  Serial.print(": ");
  Serial.println(value);
#endif
}

// ─── FILTERS ───
// Band-Stop Butterworth IIR digital filter (50Hz notch)
class NotchFilter
{
private:
  struct BiquadState
  {
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
class EOGFilter
{
private:
  struct BiquadState
  {
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

// High-Pass Butterworth IIR for jaw clench (70Hz)
class JawHighPassFilter
{
private:
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;

public:
  float process(float input)
  {
    float output = input;

    float x0 = output - (-0.82523238f * state0.z1) - (0.29463653f * state0.z2);
    output = 0.52996723f * x0 + -1.05993445f * state0.z1 + 0.52996723f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    return output;
  }

  void reset()
  {
    state0.z1 = state0.z2 = 0;
  }
} jawHighPassFilter;

// Low-Pass Butterworth IIR digital filter
class EEGFilter
{
private:
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;

public:
  float process(float input)
  {
    float output = input;

    float x0 = output - (-1.24200128f * state0.z1) - (0.45885207f * state0.z2);
    output = 0.05421270f * x0 + 0.10842539f * state0.z1 + 0.05421270f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    return output;
  }

  void reset()
  {
    state0.z1 = state0.z2 = 0;
  }
} eegFilter;

// Update EEG envelope for blinks
float updateEEGEnvelope(float sample)
{
  float absSample = fabsf(sample);
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return envelopeSum / ENVELOPE_WINDOW_SIZE;
}

// Update jaw envelope for clench detection
float updateJawEnvelope(float sample)
{
  float absSample = fabsf(sample);
  jawEnvelopeSum -= jawEnvelopeBuffer[jawEnvelopeIndex];
  jawEnvelopeSum += absSample;
  jawEnvelopeBuffer[jawEnvelopeIndex] = absSample;
  jawEnvelopeIndex = (jawEnvelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return jawEnvelopeSum / ENVELOPE_WINDOW_SIZE;
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

void getAccelerometerAngles(float &pitch, float &roll)
{
  if (imu.getSensorData() != BMI2_OK)
  {
    debugPrint("Failed to read accelerometer data");
    return;
  }

  float ax = imu.data.accelX;
  float ay = imu.data.accelY;
  float az = imu.data.accelZ;

  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll = atan2(ay, az) * 180.0 / PI;
}
void updateCalibrationStateMachine(unsigned long nowMs)
{
  if (calState == CAL_IDLE || calState == CAL_COMPLETE)
    return;

  unsigned long elapsed = nowMs - calStateStartTime;
  float pitch, roll;

  switch (calState)
  {
  case CAL_INIT_WAIT:
    if (elapsed >= 3000)
    { // 3 second initial wait
      calState = CAL_UP_VIBRATE;
      calStateStartTime = nowMs;
      startVibration();
      // Get baseline pitch
      getAccelerometerAngles(pitch, roll);
      calStartPitch = pitch;
      debugPrintValue("Baseline Pitch", calStartPitch);
    }
    break;

  case CAL_UP_VIBRATE:
    if (elapsed >= 3000)
    { // 3 second vibration for UP movement
      stopVibration();
      calState = CAL_UP_WAIT;
      calStateStartTime = nowMs;
      // Get end pitch position
      getAccelerometerAngles(pitch, roll);
      calEndPitch = pitch;
      debugPrintValue("End Pitch", calEndPitch);
      // Determine pitch direction
      pitchDirection = ((calEndPitch - calStartPitch) > 0) ? -1 : 1;
      debugPrintValue("Pitch Direction", pitchDirection);
    }
    break;

  case CAL_UP_WAIT:
    if (elapsed >= 3000)
    { // 3 second wait to return to center
      calState = CAL_LEFT_VIBRATE;
      calStateStartTime = nowMs;
      startVibration();
      // Get baseline roll
      getAccelerometerAngles(pitch, roll);
      calStartRoll = roll;
      debugPrintValue("Baseline Roll", calStartRoll);
    }
    break;

  case CAL_LEFT_VIBRATE:
    if (elapsed >= 3000)
    { // 3 second vibration for LEFT movement
      stopVibration();
      calState = CAL_LEFT_WAIT;
      calStateStartTime = nowMs;
      // Get end roll position
      getAccelerometerAngles(pitch, roll);
      calEndRoll = roll;
      debugPrintValue("End Roll", calEndRoll);
      // Determine roll direction
      rollDirection = ((calEndRoll - calStartRoll) > 0) ? -1 : 1;
      debugPrintValue("Roll Direction", rollDirection);
      axisCalibrated = true;
      debugPrint("Axis Calibrated = TRUE");
    }
    break;

  case CAL_LEFT_WAIT:
    if (elapsed >= 2000)
    { // 2 second wait to return to center
      calState = CAL_NEUTRAL_SAMPLE;
      calStateStartTime = nowMs;
      neutralSampleCount = 0;
      neutralPitchSum = 0;
      neutralRollSum = 0;
      debugPrint("LEFT_WAIT complete, moving to NEUTRAL_SAMPLE");
    }
    break;

  case CAL_NEUTRAL_SAMPLE:
    if (neutralSampleCount < 100)
    {
      getAccelerometerAngles(pitch, roll);
      neutralPitchSum += pitch;
      neutralRollSum += roll;
      neutralSampleCount++;

      if (neutralSampleCount % 10 == 0)
      {
        debugPrintValue("Neutral sample count", neutralSampleCount);
      }
    }
    else
    {
      // Calibration complete
      neutralPitch = neutralPitchSum / 100;
      neutralRoll = neutralRollSum / 100;
      smoothedPitch = neutralPitch;
      smoothedRoll = neutralRoll;
      isIMUCalibrated = true;
      calState = CAL_COMPLETE;

      debugPrintValue("Neutral Pitch", neutralPitch);
      debugPrintValue("Neutral Roll", neutralRoll);
      debugPrint("Calibration COMPLETE!");

      // Give completion feedback (3 short vibrations)
      for (int i = 0; i < 3; i++)
      {
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

// ─── PRECISION MOUSE CONTROL ───
void updatePrecisionMouse(unsigned long nowMs)
{
  if (!isIMUCalibrated || !axisCalibrated)
  {
#if DEBUG_ENABLE
    static bool lastPrintState = false;
    if (!isIMUCalibrated && !lastPrintState)
    {
      debugPrint("Waiting for calibration...");
      lastPrintState = true;
    }
#endif
    return;
  }

  // High frequency updates for responsiveness
  if (nowMs - lastMouseUpdate < MOUSE_UPDATE_RATE)
    return;
  lastMouseUpdate = nowMs;

  float currentPitch, currentRoll;
  getAccelerometerAngles(currentPitch, currentRoll);

  // Apply smoothing filter for stability
  smoothedPitch = MOVEMENT_SMOOTHING * smoothedPitch + (1.0f - MOVEMENT_SMOOTHING) * currentPitch;
  smoothedRoll = MOVEMENT_SMOOTHING * smoothedRoll + (1.0f - MOVEMENT_SMOOTHING) * currentRoll;

  // Calculate movement deltas relative to neutral position
  float deltaPitch = smoothedPitch - neutralPitch;
  float deltaRoll = smoothedRoll - neutralRoll;

  // Apply deadzone for easier control
  if (abs(deltaPitch) < DEADZONE)
    deltaPitch = 0;
  if (abs(deltaRoll) < DEADZONE)
    deltaRoll = 0;

// Print debug info
#if DEBUG_ENABLE
  static int debugCounter = 0;
  if (debugCounter++ % 50 == 0)
  {
    Serial.print("Angles - Pitch: ");
    Serial.print(currentPitch);
    Serial.print(", Roll: ");
    Serial.print(currentRoll);
    Serial.print(" | Delta - X: ");
    Serial.print(deltaRoll);
    Serial.print(", Y: ");
    Serial.println(deltaPitch);
  }
#endif

  // Calculate target velocity with precision zones
  float targetVelocityX = 0, targetVelocityY = 0;

  if (deltaRoll != 0)
  {
    float absRoll = abs(deltaRoll);
    float rollDirection_sign = (deltaRoll > 0) ? 1.0f : -1.0f;

    // Normalize angle for acceleration calculation
    float normalizedRoll = constrain(absRoll / MAX_TILT_ANGLE, 0.0f, 1.0f);

    float rollSensitivity;

    // Check if in precision zone (small movements)
    if (absRoll <= PRECISION_ZONE)
    {
      // Ultra-precise control for minute movements
      rollSensitivity = MIN_SENSITIVITY * PRECISION_MULTIPLIER * (absRoll / PRECISION_ZONE);
    }
    else
    {
      // Normal acceleration curve for larger movements
      float acceleration = pow(normalizedRoll, ACCEL_CURVE);
      rollSensitivity = MIN_SENSITIVITY + (MAX_SENSITIVITY - MIN_SENSITIVITY) * acceleration * ACCEL_MULTIPLIER;
    }

    targetVelocityX = rollDirection_sign * rollSensitivity * rollDirection;
  }

  if (deltaPitch != 0)
  {
    float absPitch = abs(deltaPitch);
    float pitchDirection_sign = (deltaPitch > 0) ? 1.0f : -1.0f;

    // Normalize angle for acceleration calculation
    float normalizedPitch = constrain(absPitch / MAX_TILT_ANGLE, 0.0f, 1.0f);

    float pitchSensitivity;

    // Check if in precision zone (small movements)
    if (absPitch <= PRECISION_ZONE)
    {
      // Ultra-precise control for minute movements
      pitchSensitivity = MIN_SENSITIVITY * PRECISION_MULTIPLIER * (absPitch / PRECISION_ZONE);
    }
    else
    {
      // Normal acceleration curve for larger movements//

      float acceleration = pow(normalizedPitch, ACCEL_CURVE);
      pitchSensitivity = MIN_SENSITIVITY + (MAX_SENSITIVITY - MIN_SENSITIVITY) * acceleration * ACCEL_MULTIPLIER;
    }

    targetVelocityY = pitchDirection_sign * pitchSensitivity * pitchDirection;
  }

  // Velocity-based movement with smooth decay for stopping
  if (deltaRoll == 0 || targetVelocityX == 0)
  {
    // No input - decay velocity for smooth stopping
    mouseVelocityX *= VELOCITY_DECAY;
    if (abs(mouseVelocityX) < STOP_THRESHOLD)
      mouseVelocityX = 0;
  }
  else
  {
    // Active input - blend toward target velocity smoothly
    mouseVelocityX = mouseVelocityX * 0.8f + targetVelocityX * 0.2f;
  }

  if (deltaPitch == 0 || targetVelocityY == 0)
  {
    // No input - decay velocity for smooth stopping
    mouseVelocityY *= VELOCITY_DECAY;
    if (abs(mouseVelocityY) < STOP_THRESHOLD)
      mouseVelocityY = 0;
  }
  else
  {
    // Active input - blend toward target velocity smoothly
    mouseVelocityY = mouseVelocityY * 0.8f + targetVelocityY * 0.2f;
  }

  // Store current deltas for next frame
  lastDeltaPitch = deltaPitch;
  lastDeltaRoll = deltaRoll;

  // Convert to integer movement
  int finalMouseX = round(mouseVelocityX);
  int finalMouseY = round(mouseVelocityY);

// Print movement
#if DEBUG_ENABLE
  if (debugCounter % 50 == 0)
  {
    Serial.print("Mouse - X: ");
    Serial.print(finalMouseX);
    Serial.print(", Y: ");
    Serial.println(finalMouseY);
  }
#endif

  // Send precise mouse movement
  if (finalMouseX != 0 || finalMouseY != 0)
  {
    Mouse.move(finalMouseX, finalMouseY);
    lastCmdSentMs = millis();
    ledState = LED_BLUE_FADE;
#if DEBUG_ENABLE
    Serial.print("Mouse moved - X: ");
    Serial.print(finalMouseX);
    Serial.print(", Y: ");
    Serial.println(finalMouseY);
#endif
  }
}

// ========== JAW CLENCH DETECTION ==========
void handleJawClench(unsigned long nowMs)
{
  if (!jawState)
  {
    // Not currently clenching - check for threshold crossing
    if (currentJawEnvelope > JAW_THRESHOLD && (nowMs - lastJawClenchTime) >= JAW_DEBOUNCE_MS)
    {
      jawState = true;
      jawClenchTriggered = false; // Not triggered yet for this press
      lastJawClenchTime = nowMs;
      debugPrint("Jaw clench START");
    }
  }
  else
  {
    // Currently in clench state
    if (!jawClenchTriggered)
    {
      // This is the first time we're detecting this clench
      // Send left mouse click
      Mouse.click(MOUSE_LEFT);
      lastCmdSentMs = millis();
      ledState = LED_BLUE_FADE;

      jawClenchTriggered = true;
      debugPrint("Jaw clench - Left click!");

      // Visual feedback (vibration)
      startVibration();
      delay(50);
      stopVibration();
    }

    // Check if jaw clench has ended
    if (currentJawEnvelope < JAW_OFF_THRESHOLD)
    {
      jawState = false;
      debugPrint(" Jaw clench END");
    }
  }
}

// ========== BLINK DETECTION (for triple blink = right click) ==========
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
      debugPrint("First blink detected");
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS)
    {
      secondBlinkTime = nowMs;
      blinkCount = 2;
      debugPrint("Second blink detected");
    }
    else if (blinkCount == 2 && (nowMs - secondBlinkTime) <= triple_blink_ms)
    {
      // Triple blink detected -> Right mouse click
      Mouse.click(MOUSE_RIGHT);
      lastCmdSentMs = millis();
      ledState = LED_BLUE_FADE;

      blinkCount = 0;
      debugPrint("Triple blink - Right click!");
    }
    else
    {
      firstBlinkTime = nowMs;
      blinkCount = 1;
      debugPrint("Blink timeout - resetting");
    }
    blinkActive = true;
  }

  if (!envelopeHigh)
  {
    blinkActive = false;
  }

  // Double blink timeout (no action for double blink - only triple blink does right click)
  if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms)
  {
    blinkCount = 0;
    debugPrint("Double blink timeout - no action");
  }
  // Single blink timeout
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS)
  {
    blinkCount = 0;
  }
}

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
      color = pixel.Color(0, 0, 20);
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
  }
}

// ─── setup() ───
void setup()
{
  Serial.begin(115200);
  delay(2000);

  pixel.begin();
  pixel.clear();
  int currentBattery = getCurrentBatteryPercentage();
  if (currentBattery <= 20)
  {
    batteryColor = pixel.Color(20, 0, 0);
  }
  else if (currentBattery <= 70)
  {
    batteryColor = pixel.Color(30, 20, 0);
  }
  else
  {
    batteryColor = pixel.Color(0, 20, 0);
  }
  pixel.setPixelColor(BATTERY_LED, batteryColor);

  Wire.begin(22, 23);
  while (imu.beginI2C() != BMI2_OK)
  {
    debugPrint("BMI270 initialization FAILED!");

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

  debugPrint("System Starting...");
  debugPrint("Initializing I2C...");

  pinMode(INPUT_PIN1, INPUT);
  pinMode(VIBRATION_PIN, OUTPUT);

  digitalWrite(VIBRATION_PIN, LOW);
  debugPrint("Pins initialized");

  debugPrint("Initializing BLE Combo...");
  Keyboard.begin();
  Mouse.begin();
  debugPrint("BLE Combo initialized");

  debugPrint("Initializing BMI270...");
  debugPrint("BMI270 initialized successfully!");

  updateIMULed(true);

  // Give power-up indication (2 short vibrations)
  debugPrint("Power-up vibration pulses");
  for (int i = 0; i < 2; i++)
  {
    startVibration();
    delay(100);
    stopVibration();
    delay(100);
  }

  // START NON-BLOCKING CALIBRATION
  calState = CAL_INIT_WAIT;
  calStateStartTime = millis();
  debugPrint("Calibration started - Keep head still for 3 seconds");
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
  bool imuConnect;
  Wire.beginTransmission(imuAddress);
  if (!Wire.endTransmission())
  {
    imuConnect = true;
  }
  else
  {
    imuConnect = false;
  }
  static bool lastIMUconnectStatus = false;
  if (imuConnect != lastIMUconnectStatus)
  {
    lastIMUconnectStatus = imuConnect;
    updateIMULed(imuConnect);
    pixelDirty = true;

    if (!imuConnect)
    {
      // ── IMU just DISCONNECTED ──
      // Invalidate calibration so mouse stops moving
      isIMUCalibrated = false;
      axisCalibrated = false;
      calState = CAL_IDLE;

      mouseVelocityX = 0;
      mouseVelocityY = 0;

      eegNotchFilter.reset();
      eogFilter.reset();
      jawHighPassFilter.reset();
      eegFilter.reset();
      debugPrint("IMU disconnected - calibration invalidated");
    }
    else
    {
      // ── IMU just RECONNECTED ──
      // Re-init the IMU hardware
      if (imu.beginI2C() == BMI2_OK)
      {
        debugPrint("IMU reconnected - restarting calibration");
        calState = CAL_INIT_WAIT;
        calStateStartTime = millis();
      }
      else
      {
        debugPrint("IMU reconnected but beginI2C() failed");
      }
    }
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
      batteryColor = pixel.Color(30, 20, 0);
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
    lastPixel0Color = 0xFFFFFFFF;
    pixelDirty = false;
  }
  updateBLELed();

  static unsigned long lastMicros = micros();
  unsigned long nowMs = millis();

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;
  static long timer = 0;
  timer -= dt;

  if (timer <= 0 && connected && imu.getSensorData() == BMI2_OK)
  {
    timer += 1000000L / SAMPLE_RATE;

    // NON-BLOCKING CALIBRATION UPDATE
    updateCalibrationStateMachine(nowMs);

    // 1) EEG ADC read - only one channel
    int raw1 = analogRead(INPUT_PIN1);
    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

    // 2) Apply notch filter (50Hz removal)
    float notchFiltered = eegNotchFilter.process(raw1);

    // 3) Process for blink detection (low frequency)
    float filteredEEG = eegFilter.process(notchFiltered);
    float filteredEOG = eogFilter.process(filteredEEG);
    currentEEGEnvelope = updateEEGEnvelope(filteredEOG);

    // 4) Process for jaw clench detection (high frequency - 70Hz HPF)
    float jawFiltered = jawHighPassFilter.process(notchFiltered);
    currentJawEnvelope = updateJawEnvelope(jawFiltered);

    handleJawClench(nowMs);
    handleBlinks(nowMs);

// Print envelopes occasionally
#if DEBUG_ENABLE
    static int eegCounter = 0;
    if (eegCounter++ % 100 == 0)
    {
      Serial.print("EEG Envelope: ");
      Serial.print(currentEEGEnvelope);
      Serial.print(" | Jaw Envelope: ");
      Serial.print(currentJawEnvelope);
      Serial.print(" | Thresholds - Blink: ");
      Serial.print(BlinkThreshold);
      Serial.print(", Jaw: ");
      Serial.println(JAW_THRESHOLD);
    }
#endif
  }

  // 4) PRECISION MOUSE CONTROL (ACCELEROMETER BASED) - runs continuously
  if (connected)
  {
    updatePrecisionMouse(nowMs);
  }

// Print status every 5 seconds
#if DEBUG_ENABLE
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 5000)
  {
    lastStatusPrint = millis();
    Serial.println("═══════════════════════════════════");
    Serial.print("BLE Connected: ");
    Serial.println(connected ? "YES" : "NO");
    Serial.print("IMU Calibrated: ");
    Serial.println(isIMUCalibrated ? "YES" : "NO");
    Serial.print("Axis Calibrated: ");
    Serial.println(axisCalibrated ? "YES" : "NO");
    Serial.print("Neutral - Pitch: ");
    Serial.print(neutralPitch);
    Serial.print(", Roll: ");
    Serial.println(neutralRoll);
    Serial.print("Calibration State: ");
    switch (calState)
    {
    case CAL_IDLE:
      Serial.println("IDLE");
      break;
    case CAL_INIT_WAIT:
      Serial.println("INIT_WAIT");
      break;
    case CAL_UP_VIBRATE:
      Serial.println("UP_VIBRATE");
      break;
    case CAL_UP_WAIT:
      Serial.println("UP_WAIT");
      break;
    case CAL_CENTER_WAIT1:
      Serial.println("CENTER_WAIT1");
      break;
    case CAL_LEFT_VIBRATE:
      Serial.println("LEFT_VIBRATE");
      break;
    case CAL_LEFT_WAIT:
      Serial.println("LEFT_WAIT");
      break;
    case CAL_CENTER_WAIT2:
      Serial.println("CENTER_WAIT2");
      break;
    case CAL_NEUTRAL_SAMPLE:
      Serial.println("NEUTRAL_SAMPLE");
      break;
    case CAL_COMPLETE:
      Serial.println("COMPLETE");
      break;
    }
    Serial.println("═══════════════════════════════════");
  }
#endif
}
