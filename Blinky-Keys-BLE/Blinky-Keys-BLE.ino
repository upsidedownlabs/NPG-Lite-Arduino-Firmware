// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Aman Maheshwari - aman@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech
// Copyright (c) 2026 Varun Patil - vap05072006@gmail.com

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <HIDKeyboardTypes.h>

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE 512 // samples per second
#define BAUD_RATE 115200
#define INPUT_PIN A0
#define PIN_NEOPIXEL 15
#define BATTERY_VOLTAGE_PIN A6
#define BLUE_LED_DURATION 100

Adafruit_NeoPixel pixel(6, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5

// EEG Envelope Configuration
#define ENVELOPE_WINDOW_MS 100 // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

// Double Blink Detection Configuration
const unsigned long BLINK_DEBOUNCE_MS = 250; // minimal spacing between individual blinks
const unsigned long DOUBLE_BLINK_MS = 600;   // max time between the two blinks
unsigned long lastBlinkTime = 0;             // time of most recent blink
unsigned long firstBlinkTime = 0;            // time of the first blink in a pair
unsigned long secondBlinkTime = 0;
unsigned long triple_blink_ms = 600;
int blinkCount = 0; // how many valid blinks so far (0–2)

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
  }
}

// HID Keyboard Variables
BLEHIDDevice *pHIDDevice;
BLECharacteristic *pInputCharacteristic;
BLECharacteristic *pOutputCharacteristic;
BLECharacteristic *pFeatureCharacteristic;
bool clientConnected = false;

// EEG Processing Variables
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;
float BlinkThreshold = 75.0;

// HID Report Descriptor for Keyboard
static const uint8_t _hidReportDescriptor[] = {
    USAGE_PAGE(1), 0x01, // USAGE_PAGE (Generic Desktop Ctrls)
    USAGE(1), 0x06,      // USAGE (Keyboard)
    COLLECTION(1), 0x01, // COLLECTION (Application)
    // ------------------------------------------------- Keyboard
    REPORT_ID(1), 0x01,       //   REPORT_ID (1)
    USAGE_PAGE(1), 0x07,      //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1), 0xE0,   //   USAGE_MINIMUM (0xE0)
    USAGE_MAXIMUM(1), 0xE7,   //   USAGE_MAXIMUM (0xE7)
    LOGICAL_MINIMUM(1), 0x00, //   LOGICAL_MINIMUM (0)
    LOGICAL_MAXIMUM(1), 0x01, //   LOGICAL_MAXIMUM (1)
    REPORT_SIZE(1), 0x01,     //   REPORT_SIZE (1)
    REPORT_COUNT(1), 0x08,    //   REPORT_COUNT (8)
    HIDINPUT(1), 0x02,        //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    REPORT_COUNT(1), 0x01,    //   REPORT_COUNT (1) ; 1 byte (Reserved)
    REPORT_SIZE(1), 0x08,     //   REPORT_SIZE (8)
    HIDINPUT(1), 0x01,        //   INPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    REPORT_COUNT(1), 0x05,    //   REPORT_COUNT (5) ; 5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1), 0x01,     //   REPORT_SIZE (1)
    USAGE_PAGE(1), 0x08,      //   USAGE_PAGE (LEDs)
    USAGE_MINIMUM(1), 0x01,   //   USAGE_MINIMUM (0x01) ; Num Lock
    USAGE_MAXIMUM(1), 0x05,   //   USAGE_MAXIMUM (0x05) ; Kana
    HIDOUTPUT(1), 0x02,       //   OUTPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    REPORT_COUNT(1), 0x01,    //   REPORT_COUNT (1) ; 3 bits (Padding)
    REPORT_SIZE(1), 0x03,     //   REPORT_SIZE (3)
    HIDOUTPUT(1), 0x01,       //   OUTPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    REPORT_COUNT(1), 0x06,    //   REPORT_COUNT (6) ; 6 bytes (Keys)
    REPORT_SIZE(1), 0x08,     //   REPORT_SIZE(8)
    LOGICAL_MINIMUM(1), 0x00, //   LOGICAL_MINIMUM(0)
    LOGICAL_MAXIMUM(1), 0x65, //   LOGICAL_MAXIMUM(0x65) ; 101 keys
    USAGE_PAGE(1), 0x07,      //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1), 0x00,   //   USAGE_MINIMUM (0)
    USAGE_MAXIMUM(1), 0x65,   //   USAGE_MAXIMUM (0x65)
    HIDINPUT(1), 0x00,        //   INPUT (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    END_COLLECTION(0)         // END_COLLECTION
};

class MyCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    clientConnected = true;
    ledState = LED_GREEN;
    pixelDirty = true;
    Serial.println("HID Keyboard connected");
  }

  void onDisconnect(BLEServer *pServer)
  {
    clientConnected = false;
    ledState = LED_RED;
    pixelDirty = true;
    Serial.println("HID Keyboard disconnected");
    BLEDevice::startAdvertising();
  }
};

// --- Filter Functions ---
// High-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: 5.0 Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float highpass(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.91327599 * z1 - 0.91688335 * z2;
    output = 0.95753983 * x + -1.91507967 * z1 + 0.95753983 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Band-Stop Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 512.0 Hz, frequency: [48.0, 52.0] Hz.
// Filter is order 2, implemented as second-order sections (biquads).
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
float Notch(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - -1.58696045 * z1 - 0.96505858 * z2;
    output = 0.96588529 * x + -1.57986211 * z1 + 0.96588529 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.62761184 * z1 - 0.96671306 * z2;
    output = 1.00000000 * x + -1.63566226 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// EEG Envelope Detection Function
float updateEEGEnvelope(float sample)
{
  float absSample = fabs(sample); // Rectify EEG signal

  // Update circular buffer and running sum
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;

  return envelopeSum / ENVELOPE_WINDOW_SIZE; // Return moving average
}

// Send Right Arrow Key
void sendRightArrow()
{
  if (clientConnected)
  {
    uint8_t report[8] = {0};
    report[0] = 0;    // Modifier keys
    report[1] = 0;    // Reserved
    report[2] = 0x4F; // Right Arrow key code

    // Send key press
    lastCmdSentMs = millis();
    ledState = LED_BLUE_FADE;
    pInputCharacteristic->setValue(report, 8);
    pInputCharacteristic->notify();

    delay(50); // Small delay to ensure key press is registered

    // Send key release
    memset(report, 0, 8);
    pInputCharacteristic->setValue(report, 8);
    pInputCharacteristic->notify();

    Serial.println("Right arrow key sent!");
  }
}

void sendLeftArrow()
{
  if (clientConnected)
  {
    uint8_t report[8] = {0};
    report[0] = 0;    // Modifier keys
    report[1] = 0;    // Reserved
    report[2] = 0x50; // Left Arrow key code

    // Send key press
    lastCmdSentMs = millis();
    ledState = LED_BLUE_FADE;
    pInputCharacteristic->setValue(report, 8);
    pInputCharacteristic->notify();

    delay(50); // Small delay to ensure key press is registered

    // Send key release
    memset(report, 0, 8);
    pInputCharacteristic->setValue(report, 8);
    pInputCharacteristic->notify();

    Serial.println("Left arrow key sent!");
  }
}

void setup()
{
  pixel.begin();
  pixel.clear();
  pixel.show();

  Serial.begin(BAUD_RATE);
  delay(100);

  pinMode(INPUT_PIN, INPUT);

  // Initialize BLE HID Device
  BLEDevice::init("Winky Blinky");
  // Retrieve the BLE MAC address
  String bleMAC = BLEDevice::getAddress().toString();

  // Set device name
  String deviceName = "Winky Blinky " + bleMAC.substring(bleMAC.length() - 5);
  esp_ble_gap_set_device_name(deviceName.c_str());
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyCallbacks());

  // Create HID Device
  pHIDDevice = new BLEHIDDevice(pServer);
  pInputCharacteristic = pHIDDevice->inputReport(1);     // reportID
  pOutputCharacteristic = pHIDDevice->outputReport(1);   // reportID
  pFeatureCharacteristic = pHIDDevice->featureReport(1); // reportID

  pHIDDevice->manufacturer()->setValue("Upside Down Labs");
  pHIDDevice->pnp(0x02, 0xe502, 0xa111, 0x0210);
  pHIDDevice->hidInfo(0x00, 0x02);

  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

  pHIDDevice->reportMap((uint8_t *)_hidReportDescriptor, sizeof(_hidReportDescriptor));
  pHIDDevice->startServices();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setAppearance(HID_KEYBOARD);
  pAdvertising->addServiceUUID(pHIDDevice->hidService()->getUUID());
  pAdvertising->start();

  Serial.println("EEG HID Keyboard initialized. Waiting for connection...");
}

void loop()
{
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

  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  static long timer = 0;
  timer -= dt;
  if (timer <= 0)
  {
    timer += 1000000L / SAMPLE_RATE;
    int raw = analogRead(INPUT_PIN);
    float filt = highpass(Notch(raw));
    currentEEGEnvelope = updateEEGEnvelope(filt);
    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;
  }

  // Double blink detection
  unsigned long nowMs = millis();

  // 1) Did we cross threshold and respect per‑blink debounce?
  if (currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS)
  {
    lastBlinkTime = nowMs; // mark this blink

    // 2) Count it
    if (blinkCount == 0)
    {
      // first blink of the pair
      firstBlinkTime = nowMs;
      blinkCount = 1;
      Serial.println("First blink detected");
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS)
    {
      // double blink detected - send right arrow key
      secondBlinkTime = nowMs;
      blinkCount = 2;
      Serial.println("Second blink registered, waiting for triple…");
    }
    else if (blinkCount == 2 && (nowMs - secondBlinkTime) <= triple_blink_ms)
    {
      Serial.println("Triple blink detected!");
      sendLeftArrow();
      blinkCount = 0;
    }
    else
    {
      // either too late or extra blink → restart sequence
      firstBlinkTime = nowMs;
      blinkCount = 1;
      Serial.println("Blink sequence restarted");
    }
  }

  // if we were in “2 blinks” but no third arrived in time → treat as a real double
  if (blinkCount == 2 && (nowMs - secondBlinkTime) > triple_blink_ms)
  {
    Serial.println("Double blink confirmed!");
    sendRightArrow();
    blinkCount = 0;
  }

  // 3) Timeout: if we never got the second blink in time, reset
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS)
  {
    blinkCount = 0;
    Serial.println("Double blink timeout - sequence reset");
  }
}
