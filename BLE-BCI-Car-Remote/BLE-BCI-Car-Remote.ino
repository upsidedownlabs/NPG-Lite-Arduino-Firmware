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

// Copyright (c) 2024-2025 Aman Maheshwari    - Aman@upsidedownlabs.tech
// Copyright (c) 2024-2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2024-2025 Deepak Khatri      - deepak@upsidedownlabs.tech
// Copyright (c) 2024-2025 Upside Down Labs   - contact@upsidedownlabs.tech
//
// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
 This example is adapted from the client and server code provided by MoThunderz
 Firmware: https://github.com/mo-thunderz/Esp32BlePart2
 YouTube video: https://www.youtube.com/watch?v=s3yoZa6kzus
*/
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include "esp_dsp.h"
#include <vector>
#include <Preferences.h>

// ---------------------------------------------------------------
//  BLE UUIDs
// ---------------------------------------------------------------
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_1 "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// ---------------------------------------------------------------
//  Hardware pins
// ---------------------------------------------------------------
#define PIN_NEOPIXEL 15
#define PIN_LED_VIB 7 // NPG Lite: shared LED + vibration motor

Adafruit_NeoPixel pixel(6, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5

// ---------------------------------------------------------------
//  Signal processing config
// ---------------------------------------------------------------
#define SAMPLE_RATE 512
#define FFT_SIZE 512
#define BAUD_RATE 115200
#define INPUT_PIN1 A0          // EEG
#define INPUT_PIN2 A1          // Left EMG
#define INPUT_PIN3 A2          // Right EMG
#define BATTERY_VOLTAGE_PIN A6 // Battery connected ADC
#define BLUE_LED_DURATION 100

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
#define SMOOTHING_FACTOR 0.63f
#define EPS 1e-7f

// ---------------------------------------------------------------
//  Debug print rate: print every N samples (500 Hz / N = print Hz)
//  e.g. N=50 → 10 Hz,  N=5 → 100 Hz,  N=500 → 1 Hz
// ---------------------------------------------------------------
#define DEBUG_PRINT_EVERY_N_SAMPLES 100

// ---------------------------------------------------------------
//  Thresholds - loaded from NVS on boot
// ---------------------------------------------------------------
uint32_t betaThreshold = 4;
uint32_t emg1Threshold = 150;
uint32_t emg2Threshold = 150;

Preferences prefs;

// ---------------------------------------------------------------
//  Globals shared between loop() and debug
// ---------------------------------------------------------------
float gBetaPct = 0.0f;
float gEnv1 = 0.0f;
float gEnv2 = 0.0f;
bool debugEnabled = false;
static bool pixelDirty = false;

// ---------------------------------------------------------------
//  BLE objects
// ---------------------------------------------------------------
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic_1 = NULL;
BLECharacteristic *pCharacteristic_2 = NULL;
BLEDescriptor *pDescr_1;
BLE2902 *pBLE2902_1;
BLE2902 *pBLE2902_2;

bool deviceConnected = false;
bool oldDeviceConnected = false;

// ---------------------------------------------------------------
//  Control state
// ---------------------------------------------------------------
bool isGoingBackward = false;
uint32_t lastSentCmd = 255;

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

// ---------------------------------------------------------------
//  DSP buffers
// ---------------------------------------------------------------
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

typedef struct
{
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;
BandpowerResults smoothedPowers = {0, 0, 0, 0, 0, 0};

// ---------------------------------------------------------------
//  Battery level indication
// ---------------------------------------------------------------
static const unsigned long BATTERY_CHECK_INTERVAL = 10000; // Interval in milliseconds
static unsigned long lastBatteryCheck = -10000;
;
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

// ----------------- NOTCH FILTER CLASSES -----------------
// For 50Hz AC noise removal
// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
class NotchFilter
{
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState s1, s2;

public:
  float process(float in)
  {
    float x = in - (-1.56858163f * s1.z1) - (0.96424138f * s1.z2);
    float out = 0.96508099f * x + (-1.56202714f * s1.z1) + (0.96508099f * s1.z2);
    s1.z2 = s1.z1;
    s1.z1 = x;
    x = out - (-1.61100358f * s2.z1) - (0.96592171f * s2.z2);
    out = 1.0f * x + (-1.61854514f * s2.z1) + (1.0f * s2.z2);
    s2.z2 = s2.z1;
    s2.z1 = x;
    return out;
  }
  void reset() { s1.z1 = s1.z2 = s2.z1 = s2.z2 = 0; }
};

// ----------------- EMG FILTER CLASSES -----------------
// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html

class EMGHighPassFilter
{
  double z1 = 0, z2 = 0;

public:
  double process(double in)
  {
    double x = in - (-0.82523238) * z1 - (0.29463653) * z2;
    double out = 0.52996723 * x + (-1.05993445) * z1 + 0.52996723 * z2;
    z2 = z1;
    z1 = x;
    return out;
  }
  void reset() { z1 = z2 = 0; }
};

// ---------------------------------------------------------------
//  Envelope filter
// ---------------------------------------------------------------
class EnvelopeFilter
{
  std::vector<double> buf;
  double sum = 0;
  int idx = 0;
  const int sz;

public:
  EnvelopeFilter(int s) : sz(s) { buf.resize(s, 0.0); }
  double getEnvelope(double v)
  {
    sum -= buf[idx];
    sum += v;
    buf[idx] = v;
    idx = (idx + 1) % sz;
    return sum / sz;
  }
};

// ---------------------------------------------------------------
//  EEG low-pass filter
// ---------------------------------------------------------------
float EEGFilter(float in)
{
  static float z1 = 0, z2 = 0;
  float x = in - (-1.22465158f) * z1 - (0.45044543f) * z2;
  float out = 0.05644846f * x + 0.11289692f * z1 + 0.05644846f * z2;
  z2 = z1;
  z1 = x;
  return out;
}

NotchFilter filters[3];
EMGHighPassFilter emgfilters[2];
EnvelopeFilter Envelopefilter1(16);
EnvelopeFilter Envelopefilter2(16);

// ---------------------------------------------------------------
//  sendCmd - only transmits over BLE when the command changes.
// ---------------------------------------------------------------
void sendCmd(uint32_t cmd)
{
  if (cmd == lastSentCmd)
    return;
  lastSentCmd = cmd;
  uint8_t val = (uint8_t)cmd;
  pCharacteristic_1->setValue(&val, 1);
  pCharacteristic_1->notify();
  Serial.print("cmd: ");
  Serial.println(cmd);
  lastCmdSentMs = millis();
  ledState = LED_BLUE_FADE;
}

// ---------------------------------------------------------------
//  MAC address helper
// ---------------------------------------------------------------
void printMacAddress()
{
  String mac = BLEDevice::getAddress().toString().c_str();
  Serial.println("============================================");
  Serial.println("  NPG Lite MAC Address (copy this line)   ");
  Serial.println("============================================");
  Serial.println("Set MAC " + mac);
  Serial.println("Paste the line above into the car firmware  ");
  Serial.println("when prompted, then reset the car.         ");
  Serial.println("============================================");
}

// ---------------------------------------------------------------
//  Serial command parser
// ---------------------------------------------------------------
void handleSerialCommands()
{
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();

  if (line == "debug")
  {
    debugEnabled = true;
    Serial.println("Debug mode ENABLED");
    Serial.print("Print rate: every ");
    Serial.print(DEBUG_PRINT_EVERY_N_SAMPLES);
    Serial.print(" samples (~");
    Serial.print(SAMPLE_RATE / DEBUG_PRINT_EVERY_N_SAMPLES);
    Serial.println(" Hz)");
    return;
  }

  if (line == "mac")
  {
    printMacAddress();
    return;
  }

  if (line == "exit")
  {
    debugEnabled = false;
    Serial.println("Debug mode DISABLED");
    return;
  }

  if (line.startsWith("set "))
  {
    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);
    if (secondSpace == -1)
    {
      Serial.println("Usage: set <betathreshold|emg1threshold|emg2threshold> <value>");
      return;
    }
    String key = line.substring(firstSpace + 1, secondSpace);
    String valStr = line.substring(secondSpace + 1);
    valStr.trim();
    uint32_t val = (uint32_t)valStr.toInt();

    prefs.begin("thresholds", false);

    if (key == "betathreshold")
    {
      betaThreshold = val;
      prefs.putUInt("betathr", betaThreshold);
      Serial.print("betaThreshold set to ");
      Serial.println(betaThreshold);
    }
    else if (key == "emg1threshold")
    {
      emg1Threshold = val;
      prefs.putUInt("emg1thr", emg1Threshold);
      Serial.print("emg1Threshold set to ");
      Serial.println(emg1Threshold);
    }
    else if (key == "emg2threshold")
    {
      emg2Threshold = val;
      prefs.putUInt("emg2thr", emg2Threshold);
      Serial.print("emg2Threshold set to ");
      Serial.println(emg2Threshold);
    }
    else
    {
      Serial.print("Unknown key: ");
      Serial.println(key);
    }

    prefs.end();
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
}

// ---------------------------------------------------------------
//  BLE status LED — pixel 0
// ---------------------------------------------------------------
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
      color = pixel.Color(0, 0, 30);
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

// ---------------------------------------------------------------
//  BLE server callbacks
// ---------------------------------------------------------------
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    for (int i = 0; i < 2; i++)
    {
      digitalWrite(PIN_LED_VIB, HIGH);
      delay(120);
      digitalWrite(PIN_LED_VIB, LOW);
      if (i == 0)
        delay(100);
    }
    ledState = LED_GREEN;
    pixelDirty = true;
    Serial.println("car connected");
    BLEDevice::getAdvertising()->stop();
  }
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    lastSentCmd = 255;
    digitalWrite(PIN_LED_VIB, LOW);
    ledState = LED_RED;
    pixelDirty = true;
    Serial.println("car disconnected, re-advertising");
    BLEDevice::startAdvertising();
  }
};

// ---------------------------------------------------------------
//  Bandpower helpers
// ---------------------------------------------------------------
BandpowerResults calculateBandpower(float *ps, float binRes, int half)
{
  BandpowerResults r = {0, 0, 0, 0, 0, 0};
  for (int i = 1; i < half; i++)
  {
    float freq = i * binRes, p = ps[i];
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

// ---------------------------------------------------------------
//  FFT init + processing
// ---------------------------------------------------------------
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
  for (int i = 0; i < FFT_SIZE; i++)
  {
    y_cf[2 * i] = inputBuffer[i];
    y_cf[2 * i + 1] = 0;
  }
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  int half = FFT_SIZE / 2;
  for (int i = 0; i < half; i++)
  {
    float re = y1_cf[2 * i], im = y1_cf[2 * i + 1];
    powerSpectrum[i] = re * re + im * im;
  }

  BandpowerResults raw = calculateBandpower(powerSpectrum, float(SAMPLE_RATE) / FFT_SIZE, half);
  smoothBandpower(&raw, &smoothedPowers);
  float T = smoothedPowers.total + EPS;
  gBetaPct = (smoothedPowers.beta / T) * 100.0f;

  if (deviceConnected)
  {
    if (gBetaPct > betaThreshold && !isGoingBackward)
    {
      sendCmd(3);
    }
    else
    {
      sendCmd(0);
    }
  }
}

// ---------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------
void setup()
{
  pixel.begin();
  pixel.clear();
  pixel.show();

  Serial.begin(BAUD_RATE);

  prefs.begin("thresholds", true);
  betaThreshold = prefs.getUInt("betathr", betaThreshold);
  emg1Threshold = prefs.getUInt("emg1thr", emg1Threshold);
  emg2Threshold = prefs.getUInt("emg2thr", emg2Threshold);
  prefs.end();

  Serial.print("Loaded betaThreshold=");
  Serial.print(betaThreshold);
  Serial.print("  emg1Threshold=");
  Serial.print(emg1Threshold);
  Serial.print("  emg2Threshold=");
  Serial.println(emg2Threshold);

  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(INPUT_PIN3, INPUT);
  pinMode(PIN_LED_VIB, OUTPUT);
  digitalWrite(PIN_LED_VIB, LOW);

  initFFT();

  BLEDevice::init("UDL-BCI-Car");
  printMacAddress();

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic_1 = pService->createCharacteristic(
      CHARACTERISTIC_UUID_1, BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic_2 = pService->createCharacteristic(
      CHARACTERISTIC_UUID_2,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY);

  pDescr_1 = new BLEDescriptor((uint16_t)0x2901);
  pDescr_1->setValue("BCI Car Control");
  pCharacteristic_1->addDescriptor(pDescr_1);

  pBLE2902_1 = new BLE2902();
  pBLE2902_1->setNotifications(true);
  pCharacteristic_1->addDescriptor(pBLE2902_1);

  pBLE2902_2 = new BLE2902();
  pBLE2902_2->setNotifications(true);
  pCharacteristic_2->addDescriptor(pBLE2902_2);

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  pixel.setPixelColor(BATTERY_LED, batteryColor); // batteryColor is 0 at boot, pixel stays off
  pixel.show();
  Serial.println("Advertising, waiting for car...");
  Serial.println("Serial commands:");
  Serial.println("  debug                      - start debug output");
  Serial.println("  exit                       - stop debug output");
  Serial.println("  mac                        - print MAC address");
  Serial.println("  set betathreshold  <n>     - set beta threshold");
  Serial.println("  set emg1threshold  <n>     - set EMG1 threshold");
  Serial.println("  set emg2threshold  <n>     - set EMG2 threshold");
}

// ---------------------------------------------------------------
//  Loop
// ---------------------------------------------------------------
void loop()
{
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  static long timer = 0;

  // ── debug print counter (counts samples, resets every N) ──
  static int debugSampleCount = 0;

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  handleSerialCommands();

  if (pixelDirty)
  {
    pixel.setPixelColor(BATTERY_LED, batteryColor);
    lastPixel0Color = 0xFFFFFFFF;
    pixelDirty = false;
  }
  updateBLELed();

  unsigned long currentMillis = millis();

  if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL)
  {
    int currentBattery = getCurrentBatteryPercentage();
    if (currentBattery <= 20)
    {
      batteryColor = pixel.Color(20, 0, 0); // red
    }
    else if (currentBattery <= 70)
    {
      batteryColor = pixel.Color(30, 20, 0); // orange
    }
    else
    {
      batteryColor = pixel.Color(0, 20, 0); // green
    }
    pixelDirty = true;
    lastBatteryCheck = currentMillis;
  }

  timer -= dt;
  if (timer <= 0)
  {
    timer += 1000000L / SAMPLE_RATE;

    // ── ADC reads ──
    int raw1 = analogRead(INPUT_PIN1);
    int raw2 = analogRead(INPUT_PIN2);
    int raw3 = analogRead(INPUT_PIN3);
    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

    // ── Filtering ──
    float filteeg = EEGFilter(filters[0].process(raw1));
    float filtemg1 = emgfilters[0].process(filters[1].process(raw2));
    float filtemg2 = emgfilters[1].process(filters[2].process(raw3));

    inputBuffer[idx++] = filteeg;

    gEnv1 = Envelopefilter1.getEnvelope(abs(filtemg1));
    gEnv2 = Envelopefilter2.getEnvelope(abs(filtemg2));

    // ── Debug print — once every N samples, no repetition ──
    if (debugEnabled)
    {
      if (++debugSampleCount >= DEBUG_PRINT_EVERY_N_SAMPLES)
      {
        debugSampleCount = 0;
        Serial.print("beta: ");
        Serial.print(gBetaPct, 2);
        Serial.print("  EMG1: ");
        Serial.print(gEnv1, 2);
        Serial.print("  EMG2: ");
        Serial.println(gEnv2, 2);
      }
    }
    else
    {
      debugSampleCount = 0; // reset counter when debug is off
    }

    // ── EMG control (only when BLE connected) ──
    if (deviceConnected)
    {
      if (gEnv1 > emg1Threshold * 0.5 && gEnv2 > emg2Threshold * 0.5)
      {
        isGoingBackward = true;
        sendCmd(4);
      }
      else if (gEnv1 > emg1Threshold && !isGoingBackward)
      {
        isGoingBackward = false;
        sendCmd(2);
      }
      else if (gEnv2 > emg2Threshold && !isGoingBackward)
      {
        isGoingBackward = false;
        sendCmd(1);
      }
      else
      {
        isGoingBackward = false;
      }
    }
  }

  // ── FFT when buffer full ──
  if (idx >= FFT_SIZE)
  {
    processFFT();
    idx = 0;
  }

  // ── BLE reconnect handling ──
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(300);
    BLEDevice::startAdvertising();
    oldDeviceConnected = false;
  }
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = true;
  }
}
