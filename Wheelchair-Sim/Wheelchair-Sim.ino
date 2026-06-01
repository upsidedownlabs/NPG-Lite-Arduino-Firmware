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

// Copyright (c) 2026 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2026 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2026 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Arduino.h>
#include <vector>
#include <BleCombo.h>

#include <Adafruit_NeoPixel.h> // For controlling Neopixel

// BLE keyboard setup
BleComboKeyboard bleKeyboard("NPG Lite GAMING", "UpsideDownLabs", 100);

#define PIXEL_PIN 15
#define PIXEL_COUNT 6
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5

#define DEBUG_LEVEL 0 // 0 = off, 1 = jaw debug, 2 = eye debug

#define NOTCH_FILTER_FREQ 50 // Set to 50 or 60 according to your powerline noise

// Key mapping (change if needed)
#define EOG_LEFT_KEY 'a'   // Left eye
#define EOG_RIGHT_KEY 'd'  // Right eye
#define JAW_SINGLE_KEY 'w' // Jaw single clench
#define JAW_DOUBLE_KEY 's' // Jaw double clench

#define SAMPLE_RATE 500 // ADC sample rate (Hz)
#define INPUT_PIN A0    // Analog input pin
#define BATTERY_VOLTAGE_PIN A6
#define BLUE_LED_DURATION 100 // ms for blue fade effect

// High-Pass Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: 1.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class EOGFilter
{
private:
  float z1_0 = 0.0;
  float z2_0 = 0.0;

public:
  float process(float input_sample)
  {
    float output = input_sample;
    float x = output - (-1.98222893 * z1_0) - (0.98238545 * z2_0);
    output = 0.99115360 * x + -1.98230719 * z1_0 + 0.99115360 * z2_0;
    z2_0 = z1_0;
    z1_0 = x;
    return output;
  }

  void reset()
  {
    z1_0 = 0.0;
    z2_0 = 0.0;
  }
};

// Low-Pass Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: 10.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class LowPassFilter
{
private:
  float z1_0 = 0.0;
  float z2_0 = 0.0;

public:
  float process(float input_sample)
  {
    float output = input_sample;
    float x = output - (-1.82269493 * z1_0) - (0.83718165 * z2_0);
    output = 0.00362168 * x + 0.00724336 * z1_0 + 0.00362168 * z2_0;
    z2_0 = z1_0;
    z1_0 = x;
    return output;
  }

  void reset()
  {
    z1_0 = 0.0;
    z2_0 = 0.0;
  }
};

// Band-Stop Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class Notch50
{
private:
  float z1_0 = 0.0;
  float z2_0 = 0.0;
  float z1_1 = 0.0;
  float z2_1 = 0.0;

public:
  float process(float input_sample)
  {
    float output = input_sample;
    // Biquad section 0
    float x = output - (-1.56858163 * z1_0) - (0.96424138 * z2_0);
    output = 0.96508099 * x + -1.56202714 * z1_0 + 0.96508099 * z2_0;
    z2_0 = z1_0;
    z1_0 = x;
    // Biquad section 1
    x = output - (-1.61100358 * z1_1) - (0.96592171 * z2_1);
    output = 1.00000000 * x + -1.61854514 * z1_1 + 1.00000000 * z2_1;
    z2_1 = z1_1;
    z1_1 = x;
    return output;
  }
  void reset()
  {
    z1_0 = z2_0 = z1_1 = z2_1 = 0.0;
  }
};

// Band-Stop Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: [58.0, 62.0] Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class Notch60
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
    // Biquad section 0
    float x0 = output - (-1.40810535f * state0.z1) - (0.96443153f * state0.z2);
    output = 0.96508099f * x0 + -1.40747202f * state0.z1 + 0.96508099f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;
    // Biquad section 1
    float x1 = output - (-1.45687509f * state1.z1) - (0.96573127f * state1.z2);
    output = 1.00000000f * x1 + -1.45839783f * state1.z1 + 1.00000000f * state1.z2;
    state1.z2 = state1.z1;
    state1.z1 = x1;
    return output;
  }
  void reset()
  {
    state0.z1 = state0.z2 = 0;
    state1.z1 = state1.z2 = 0;
  }
};

// High-Pass Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class EMG
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

    // Biquad section 0
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
};

// Tracks rolling mean for eye baseline
class BaselineTracker
{
private:
  std::vector<float> buffer;
  float sum = 0.0;
  int idx = 0;
  bool filled = false;
  const int N;

public:
  BaselineTracker(int n)
      : N(n)
  {
    buffer.resize(N, 0.0f);
  }

  void update(float sample)
  {
    sum -= buffer[idx];
    sum += sample;
    buffer[idx] = sample;
    idx++;
    if (idx >= N)
    {
      idx = 0;
      filled = true;
    }
  }

  float get_baseline()
  {
    if (!filled && idx == 0)
      return 0.0f;
    int count = filled ? N : idx;
    return (count > 0) ? (sum / count) : 0.0f;
  }
};

// Tracks rolling mean of absolute value for jaw envelope
class EnvelopeDetector
{
private:
  std::vector<float> buffer;
  float sum = 0.0f;
  int idx = 0;
  const int N;

public:
  EnvelopeDetector(int n)
      : N(n)
  {
    buffer.resize(N, 0.0f);
  }

  float update(float sampleAbs)
  {
    sum -= buffer[idx];
    sum += sampleAbs;
    buffer[idx] = sampleAbs;
    idx = (idx + 1) % N;
    return sum / N;
  }
};

// Filter and detector objects
#if NOTCH_FILTER_FREQ == 50
Notch50 notchFilterShared;
#elif NOTCH_FILTER_FREQ == 60
Notch60 notchFilterShared;
#else
#error "NOTCH_FILTER_FREQ must be 50 or 60"
#endif
EOGFilter eogHP_eye;
LowPassFilter eogLP_eye;
EMG emgHP10_jaw;
BaselineTracker horizontalBaseline(256);
EnvelopeDetector jawEnvelope(50); // 100ms window

// Eye movement detection variables
const unsigned long MOVEMENT_DEBOUNCE_MS = 800;
float EYE_MOVEMENT_THRESHOLD = 150.0f;
unsigned long lastMovementDetectedTime = 0;
float horizontalSignal = 0.0f;
const unsigned long EYE_KEY_HOLD_MS = 200;

// Jaw clench detection variables
const float JAW_THRESHOLD = 160.0f;
const float JAW_RELEASE_THRESHOLD = 70.0f;
const unsigned long JAW_DEBOUNCE_MS = 270;
const unsigned long JAW_DOUBLE_WINDOW_MS = 500;
unsigned long lastJawTime = 0;
bool jawActive = false;
bool jawReleased = true;
float jawEnv = 0.0f;
const unsigned long JAW_HOLD_REPEAT_MS = 200;
unsigned long lastJawHoldSendTime = 0;
char currentJawHoldKey = 0;
unsigned long lastJawPressTime = 0;
static bool pixelDirty = false;

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

//  Battery level indication
static const unsigned long BATTERY_CHECK_INTERVAL = 10000; // Interval in milliseconds
static unsigned long lastBatteryCheck = -10000;
uint32_t batteryColor = 0; // stored battery LED color
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

// Interpolation function
float interpolatePercentage(float voltage)
{
  // Handle out-of-range voltages
  if (voltage <= voltageLUT[0])
    return 0;
  if (voltage >= voltageLUT[lutSize - 1])
    return 100;

  // Find the nearest LUT entries
  int i = 0;
  while (i < lutSize - 1 && voltage > voltageLUT[i + 1])
    i++;

  // Interpolate
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

//  BLE status LED
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
      // hold full brightness during flash
      color = pixel.Color(0, 0, 20);
    }
    else
    {
      // 2s passed with no new command — return to green
      ledState = LED_GREEN;
      color = pixel.Color(0, 20, 0);
    }
  }

  // only write to NeoPixel bus if color actually changed
  if (color != lastPixel0Color)
  {
    lastPixel0Color = color;
    pixel.setPixelColor(BLE_LED, color);
    pixel.show();
  }
}

// Debug print functions (only print if debug enabled)
static inline void debugPrintJaw(unsigned long nowMs)
{
#if (DEBUG_LEVEL == 1)
  static unsigned long last = 0;
  if (nowMs - last >= 200)
  {
    Serial.print("JAW_ENV: ");
    Serial.println(jawEnv, 1);
    last = nowMs;
  }
#else
  (void)nowMs;
#endif
}

static inline void debugPrintEye(unsigned long nowMs, float absDeviation)
{
#if (DEBUG_LEVEL == 2)
  static unsigned long last = 0;
  if (nowMs - last >= 200)
  {
    Serial.print("EYE_ABS: ");
    Serial.println(absDeviation, 1);
    last = nowMs;
  }
#else
  (void)nowMs;
  (void)absDeviation;
#endif
}

// Detects left/right eye movement and sends key
void detectEyeMovement(unsigned long nowMs, float absDeviation)
{
  float baseline = horizontalBaseline.get_baseline();
  float deviation = horizontalSignal - baseline;

  debugPrintEye(nowMs, absDeviation);

  // Handle pending key release first (must run every call)
  static unsigned long keyReleaseTime = 0;
  static char keyToRelease = 0;
  if (keyToRelease != 0 && nowMs >= keyReleaseTime)
  {
    bleKeyboard.release(keyToRelease);
    keyToRelease = 0;
  }

  if ((nowMs - lastMovementDetectedTime) < MOVEMENT_DEBOUNCE_MS)
    return;
  if (!bleKeyboard.isConnected())
    return;

  if (deviation > EYE_MOVEMENT_THRESHOLD) // Left eye movement
  {
    Serial.println("LEFT");
    bleKeyboard.press(EOG_LEFT_KEY);
    lastCmdSentMs = millis();
    ledState = LED_BLUE_FADE;
    keyToRelease = EOG_LEFT_KEY;
    keyReleaseTime = nowMs + EYE_KEY_HOLD_MS;
    lastMovementDetectedTime = nowMs;
  }
  else if (deviation < -EYE_MOVEMENT_THRESHOLD) // Right eye movement
  {
    Serial.println("RIGHT");
    bleKeyboard.press(EOG_RIGHT_KEY);
    lastCmdSentMs = millis();
    ledState = LED_BLUE_FADE;
    keyToRelease = EOG_RIGHT_KEY;
    keyReleaseTime = nowMs + EYE_KEY_HOLD_MS;
    lastMovementDetectedTime = nowMs;
  }
}

// Detects jaw clench (single/double) and sends key while held
void detectJaw(unsigned long nowMs)
{
  debugPrintJaw(nowMs);

  bool high = (jawEnv > JAW_THRESHOLD);
  bool low = (jawEnv < JAW_RELEASE_THRESHOLD);

  // If jaw released, clear hold state
  if (low && jawActive)
  {
    jawReleased = true;
    jawActive = false;
    currentJawHoldKey = 0;
    lastJawHoldSendTime = 0;
  }

  // If jaw clenched, check for single or double clench
  if (high && !jawActive && jawReleased && (nowMs - lastJawTime) >= JAW_DEBOUNCE_MS)
  {
    lastJawTime = nowMs;
    jawReleased = false;
    jawActive = true;

    // Double clench if previous clench was recent
    if (lastJawPressTime != 0 && (nowMs - lastJawPressTime) <= JAW_DOUBLE_WINDOW_MS)
    {
      currentJawHoldKey = JAW_DOUBLE_KEY;
      Serial.println("JAW DOUBLE");
    }
    else
    {
      currentJawHoldKey = JAW_SINGLE_KEY;
      Serial.println("JAW");
    }

    lastJawPressTime = nowMs;
    lastJawHoldSendTime = 0;
  }

  // While jaw is clenched, send key every 200ms
  static unsigned long jawKeyReleaseTime = 0;
  static char jawKeyToRelease = 0;

  if (jawKeyToRelease != 0 && nowMs >= jawKeyReleaseTime)
  {
    bleKeyboard.release(jawKeyToRelease);
    jawKeyToRelease = 0;
  }

  if (high && currentJawHoldKey != 0 && bleKeyboard.isConnected())
  {
    if (nowMs - lastJawHoldSendTime >= JAW_HOLD_REPEAT_MS)
    {
      bleKeyboard.press(currentJawHoldKey);
      lastCmdSentMs = millis();
      ledState = LED_BLUE_FADE;
      jawKeyToRelease = currentJawHoldKey;
      jawKeyReleaseTime = nowMs + 10; // 10ms hold
      lastJawHoldSendTime = nowMs;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(INPUT_PIN, INPUT);

  pixel.begin();
  pixel.clear();
  pixel.show();

  bleKeyboard.begin();

#if (DEBUG_LEVEL != 0)
  Serial.println("DEBUG ON");
#endif
}

void loop()
{
  static unsigned long lastMicros = micros();
  unsigned long now = micros();
  unsigned long dt = now - lastMicros;
  lastMicros = now;

  static long timer = 0;
  timer -= dt;

  if (timer <= 0)
  {
    timer += 1000000L / SAMPLE_RATE;

    int rawA0 = analogRead(INPUT_PIN);
    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

    // Notch filter (shared)
    float n = notchFilterShared.process(rawA0);

    // Eye path: Notch -> HP(1Hz) -> LP(10Hz)
    float fh = eogHP_eye.process(n);
    fh = eogLP_eye.process(fh);
    horizontalSignal = fh;
    horizontalBaseline.update(fh);

    // Jaw path: Notch -> HP(70Hz)
    float fj = emgHP10_jaw.process(n);
    jawEnv = jawEnvelope.update(fabs(fj));
  }

  // Set NeoPixel color based on BLE connection
  // Only update ledState when connection status actually changes
  static bool lastConnected = false;
  bool connected = bleKeyboard.isConnected();
  if (connected != lastConnected)
  {
    lastConnected = connected;
    ledState = connected ? LED_GREEN : LED_RED;
    pixelDirty = true;
  }

  // Battery check FIRST, before the dirty flush
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

  // NOW flush dirty — battery color is already set
  if (pixelDirty)
  {
    pixel.setPixelColor(BATTERY_LED, batteryColor);
    lastPixel0Color = 0xFFFFFFFF; // force BLE LED redraw
    pixelDirty = false;
  }
  updateBLELed();

  unsigned long nowMs = millis();

  float baseline = horizontalBaseline.get_baseline();
  float absDeviation = fabs(horizontalSignal - baseline);

  detectEyeMovement(nowMs, absDeviation);
  detectJaw(nowMs);
}
