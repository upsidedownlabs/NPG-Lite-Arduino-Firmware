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
/*
  FFT routines based on Espressif’s ESP-DSP examples:

    • Initialization (dsps_fft2r_init_fc32) from:
      https://github.com/espressif/esp-dsp/tree/master/examples/basic_math
      (examples/basic_math/main/dsps_math_main.c)

    • Two-real FFT processing
      (dsps_fft2r_fc32, dsps_bit_rev_fc32, dsps_cplx2reC_fc32)
      from: https://github.com/espressif/esp-dsp/tree/master/examples/fft
      (examples/fft/main/dsps_fft_main.c)
*/

#include <Wire.h>
#include <BleGamepad.h>
#include <Arduino.h>
#include "esp_dsp.h"
#include <vector>
#include <Adafruit_NeoPixel.h>

// ─── BLE Gamepad ───
BleGamepad bleGamepad("NPG Gamepad", "Upside Down Labs", 100);

// ─── NeoPixel ───
#define PIXEL_PIN 15
#define PIXEL_COUNT 6
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5

// ─── Constants ───
#define SAMPLE_RATE 500 // samples per second
#define FFT_SIZE 256    // must be a power of two
#define INPUT_PIN1 A0   // EEG input
#define INPUT_PIN2 A1   // Left-hand EMG
#define INPUT_PIN3 A2   // Right-hand EMG
#define BATTERY_VOLTAGE_PIN A6
#define BLUE_LED_DURATION 100

// EEG bands (Hz)
#define DELTA_LOW 0.5f
#define DELTA_HIGH 4.0f
#define THETA_LOW 4.0f
#define THETA_HIGH 8.0f
#define ALPHA_LOW 8.0f
#define ALPHA_HIGH 13.0f
#define BETA_LOW 13.0f
#define BETA_HIGH 30.0f
#define GAMMA_LOW 30.0f
#define GAMMA_HIGH 45.0f

#define SMOOTHING_FACTOR 0.73f
#define EPS 1e-7f

bool steer = true; // To switch between button 1 and 2 controlled by focus

// ─── BLE LED state machine ───
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
uint32_t batteryColor = 0;
// For battery voltage averaging
static uint32_t batteryWinSum = 0;
static uint16_t batteryWinCount = 0;
static int lastBatteryPct = -1;
static uint8_t risingCount = 0;
static const uint8_t RISING_THRESHOLD = 3; // must rise for 3 consecutive checks to confirm rising battery level

// ─── Battery check interval ───
static const unsigned long BATTERY_CHECK_INTERVAL = 10000;
static unsigned long lastBatteryCheck = -10000;

// ─── Buffers & Types for FFT ───
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

typedef struct
{
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

BandpowerResults smoothedPowers = {0, 0, 0, 0, 0, 0};

// ─── Notch Filter (50 Hz) ───
// For 50Hz AC noise removal
// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class NotchFilter
{
private:
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState state1, state2;

public:
  float process(float input)
  {
    float output = input;
    // First biquad stage
    float x = output - (-1.56858163f * state1.z1) - (0.96424138f * state1.z2);
    output = 0.96508099f * x + (-1.56202714f * state1.z1) + (0.96508099f * state1.z2);
    state1.z2 = state1.z1;
    state1.z1 = x;
    // Second biquad stage
    x = output - (-1.61100358f * state2.z1) - (0.96592171f * state2.z2);
    output = 1.00000000f * x + (-1.61854514f * state2.z1) + (1.00000000f * state2.z2);
    state2.z2 = state2.z1;
    state2.z1 = x;
    return output;
  }
  void reset()
  {
    state1.z1 = state1.z2 = state2.z1 = state2.z2 = 0;
  }
};

NotchFilter filters[3]; // One notch per channel

// ─── EMG High-Pass Filter (70 Hz) ───
// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class EMGHighPassFilter
{
private:
  double z1 = 0.0, z2 = 0.0;

public:
  double process(double input)
  {
    double x = input - -0.82523238 * z1 - 0.29463653 * z2;
    double output = 0.52996723 * x + -1.05993445 * z1 + 0.52996723 * z2;
    z2 = z1;
    z1 = x;
    return output;
  }
  void reset()
  {
    z1 = z2 = 0.0;
  }
};

EMGHighPassFilter emgfilters[2];

// ─── EMG Envelope Detector ───
class EnvelopeFilter
{
private:
  std::vector<double> circularBuffer;
  double sum = 0.0;
  int dataIndex = 0;
  const int bufferSize;

public:
  EnvelopeFilter(int bufferSize)
      : bufferSize(bufferSize)
  {
    circularBuffer.resize(bufferSize, 0.0);
  }
  double getEnvelope(double absEmg)
  {
    sum -= circularBuffer[dataIndex];
    sum += absEmg;
    circularBuffer[dataIndex] = absEmg;
    dataIndex = (dataIndex + 1) % bufferSize;
    return (sum / bufferSize);
  }
};

EnvelopeFilter Envelopefilter1(16); // left-hand EMG
EnvelopeFilter Envelopefilter2(16); // right-hand EMG

// ─── Battery Monitor LUT ───
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
  while (voltage > voltageLUT[i + 1])
    i++;
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}

int getCurrentBatteryPercentage()
{
  // int analogValue = analogRead(BATTERY_VOLTAGE_PIN);
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
    risingCount = 0; // reset rising count on drop
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

// ─── EEG Low-Pass Filter (45 Hz) ───
// Low-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: 45.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float EEGFilter(float input)
{
  float output = input;
  static float z1 = 0, z2 = 0;
  float x = output - -1.22465158 * z1 - 0.45044543 * z2;
  output = 0.05644846 * x + 0.11289692 * z1 + 0.05644846 * z2;
  z2 = z1;
  z1 = x;
  return output;
}

// ─── FFT Routines ───
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize)
{
  BandpowerResults r = {0, 0, 0, 0, 0, 0};
  for (int i = 1; i < halfSize; i++)
  {
    float freq = i * binRes;
    float p = ps[i];
    r.total += p;
    if (freq >= DELTA_LOW && freq < DELTA_HIGH)
      r.delta += p;
    else if (freq >= THETA_LOW && freq < THETA_HIGH)
      r.theta += p;
    else if (freq >= ALPHA_LOW && freq < ALPHA_HIGH)
      r.alpha += p;
    else if (freq >= BETA_LOW && freq < BETA_HIGH)
      r.beta += p;
    else if (freq >= GAMMA_LOW && freq < GAMMA_HIGH)
      r.gamma += p;
  }
  return r;
}

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s)
{
  s->delta = SMOOTHING_FACTOR * raw->delta + (1 - SMOOTHING_FACTOR) * s->delta;
  s->theta = SMOOTHING_FACTOR * raw->theta + (1 - SMOOTHING_FACTOR) * s->theta;
  s->alpha = SMOOTHING_FACTOR * raw->alpha + (1 - SMOOTHING_FACTOR) * s->alpha;
  s->beta = SMOOTHING_FACTOR * raw->beta + (1 - SMOOTHING_FACTOR) * s->beta;
  s->gamma = SMOOTHING_FACTOR * raw->gamma + (1 - SMOOTHING_FACTOR) * s->gamma;
  s->total = SMOOTHING_FACTOR * raw->total + (1 - SMOOTHING_FACTOR) * s->total;
}

void initFFT()
{
  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if (err != ESP_OK)
  {
    Serial.println("FFT init failed");
    while (1)
      delay(10);
  }
}

void processFFT()
{
  // 1) Pack real → complex
  for (int i = 0; i < FFT_SIZE; i++)
  {
    y_cf[2 * i] = inputBuffer[i];
    y_cf[2 * i + 1] = 0.0f;
  }
  // 2) FFT + bit-reverse + complex→real
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  // 3) Compute power spectrum (skip i=0)
  int half = FFT_SIZE / 2;
  for (int i = 0; i < half; i++)
  {
    float re = y1_cf[2 * i];
    float im = y1_cf[2 * i + 1];
    powerSpectrum[i] = re * re + im * im;
  }

  // 4) Find peak bin
  int maxIdx = 1;
  float maxP = powerSpectrum[1];
  for (int i = 2; i < half; i++)
  {
    if (powerSpectrum[i] > maxP)
    {
      maxP = powerSpectrum[i];
      maxIdx = i;
    }
  }
  float binRes = float(SAMPLE_RATE) / FFT_SIZE;
  float peakHz = maxIdx * binRes;

  // 5) Compute & smooth bandpowers
  BandpowerResults raw = calculateBandpower(powerSpectrum, binRes, half);
  smoothBandpower(&raw, &smoothedPowers);
  float T = smoothedPowers.total + EPS;

  // 6) Button control based on beta waves
  if (!bleGamepad.isConnected())
    return;

  if (steer)
  { // Button 1
    if (((smoothedPowers.beta / T) * 100) > 15)
    {
      bleGamepad.press(BUTTON_1);
      lastCmdSentMs = millis();
      ledState = LED_BLUE_FADE;
    }
    else
    {
      bleGamepad.release(BUTTON_1);
    }
  }
  else if (!steer)
  { // Button 2
    if (((smoothedPowers.beta / T) * 100) > 12)
    {
      bleGamepad.press(BUTTON_2);
      lastCmdSentMs = millis();
      ledState = LED_BLUE_FADE;
    }
    else
    {
      bleGamepad.release(BUTTON_2);
    }
  }
}

// ─── BLE Status LED ───

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
  delay(1000);

  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(INPUT_PIN3, INPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  pixel.begin();
  pixel.clear();
  pixel.show();

  // Initial battery color on pixel 5
  int initBattery = getCurrentBatteryPercentage();
  if (initBattery <= 20)
  {
    batteryColor = pixel.Color(20, 0, 0);
  }
  else if (initBattery <= 70)
  {
    batteryColor = pixel.Color(30, 20, 0);
  }
  else
  {
    batteryColor = pixel.Color(0, 20, 0);
  }
  pixel.setPixelColor(BATTERY_LED, batteryColor);
  pixel.setPixelColor(BLE_LED, pixel.Color(20, 0, 0)); // red = disconnected
  lastPixel0Color = pixel.Color(20, 0, 0);
  pixel.show();

  initFFT();

  Serial.println("Starting BLE Gamepad...");
  bleGamepad.begin();
}

void loop()
{
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  static long timer = 0;
  timer -= dt;
  if (timer <= 0)
  {
    timer += 1000000L / SAMPLE_RATE;

    // 1) Read ADCs
    int raw1 = analogRead(INPUT_PIN1);
    int raw2 = analogRead(INPUT_PIN2);
    int raw3 = analogRead(INPUT_PIN3);

    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

    // 2) Filter EEG → FFT buffer
    float filteeg = EEGFilter(filters[0].process(raw1));
    inputBuffer[idx++] = filteeg;

    // 3) Filter & envelope EMG
    float filtemg1 = emgfilters[0].process(filters[1].process(raw2));
    float filtemg2 = emgfilters[1].process(filters[2].process(raw3));
    float env1 = Envelopefilter1.getEnvelope(abs(filtemg1));
    float env2 = Envelopefilter2.getEnvelope(abs(filtemg2));

    static unsigned long lastModeSwitch = 0;
    const unsigned long MODE_SWITCH_DEBOUNCE = 500; // 500ms debounce

    if (env1 > 200 && env2 > 200 && (millis() - lastModeSwitch > MODE_SWITCH_DEBOUNCE))
    { // Switching between button 1 and button 2
      steer = !steer;
      lastModeSwitch = millis();
    }

    // 4) Map to 16-bit X
    uint16_t x16;
    if (env1 > 80 && env1 > env2)
    {
      x16 = 0; // full left
    }
    else if (env2 > 80 && env2 > env1)
    {
      x16 = 32767; // full right
    }
    else
    {
      x16 = 16383; // center
    }

    // 5) Send X-axis + any button states
    if (bleGamepad.isConnected())
    {
      // setAxes(x, y, z, rx, ry, rz) only X changes, others stay 0
      bleGamepad.setAxes(x16, 0, 0, 0, 0, 0);
      bleGamepad.sendReport();
      if (x16 != 16383)
      {
        lastCmdSentMs = millis();
        ledState = LED_BLUE_FADE;
      }
    }

    // 6) Update battery level every 10 seconds
    unsigned long nowMs = millis();
    if (nowMs - lastBatteryCheck >= BATTERY_CHECK_INTERVAL)
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

      if (bleGamepad.isConnected())
      {
        bleGamepad.setBatteryLevel(currentBattery);
      }

      Serial.print("Battery: ");
      Serial.print(currentBattery);
      Serial.println("%");

      lastBatteryCheck = nowMs;
    }

    // 7) Do FFT every FFT_SIZE samples
    if (idx >= FFT_SIZE)
    {
      processFFT();
      idx = 0;
    }
  }

  // BLE connection tracking
  static bool lastConnected = false;
  bool connected = bleGamepad.isConnected();
  if (connected != lastConnected)
  {
    lastConnected = connected;
    ledState = connected ? LED_GREEN : LED_RED;
    pixelDirty = true;
  }

  // Flush dirty pixel (battery color) and force BLE LED redraw
  if (pixelDirty)
  {
    pixel.setPixelColor(BATTERY_LED, batteryColor);
    lastPixel0Color = 0xFFFFFFFF;
    pixelDirty = false;
  }
  updateBLELed();
}
