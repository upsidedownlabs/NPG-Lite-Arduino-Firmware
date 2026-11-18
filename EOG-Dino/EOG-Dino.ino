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

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2026 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2026 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <HIDKeyboardTypes.h>
#include <esp_mac.h>

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE 512 // samples per second
#define BAUD_RATE 115200
#define INPUT_PIN A0
#define PIXEL_PIN 15
#define PIXEL_COUNT 6
#define BUTTON_PIN 9 // BOOT button
#define BLUE_LED_DURATION 100

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define MODE_LED 1

// EEG Envelope Configuration
#define ENVELOPE_WINDOW_MS 100 // Smoothing window in milliseconds
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

// Blink Detection Configuration
float BlinkThreshold = 75.0;                  // envelope level for a blink
float BlinkResetThreshold = 50.0;             // envelope must fall below this before next blink
const unsigned long BLINK_DEBOUNCE_MS = 250;  // minimal spacing between blinks in Dino mode
const unsigned long CAMERA_DEBOUNCE_MS = 1000; // minimal spacing between photos in Camera mode
unsigned long lastBlinkTime = 0;              // time of most recent blink
bool blinkArmed = true;                       // ready to detect next blink

// Key Configuration
#define KEY_SPACE 0x2C                       // Dino jump
#define VOLUME_UP 0x01                       // Camera shutter
#define REPORT_SIZE_BYTES 9                  // modifier, reserved, 6 keys, volume
const unsigned long KEY_HOLD_MS = 150;       // how long the key is held
bool keyPressed = false;
unsigned long keyPressTime = 0;

// Mode Configuration
const unsigned long BUTTON_DEBOUNCE_MS = 50;
enum Mode
{
  MODE_OFF,
  MODE_DINO,
  MODE_CAMERA
};
Mode currentMode = MODE_OFF;                 // Off by default
bool lastButtonState = HIGH;
unsigned long lastButtonTime = 0;

// BLE LED state machine
enum LedState
{
  LED_RED,
  LED_GREEN,
  LED_BLUE_FADE
};
LedState ledState = LED_RED;
unsigned long lastBlinkLedMs = 0;
uint32_t lastPixel0Color = 0xFFFFFFFF;

// HID Variables
BLEHIDDevice *pHIDDevice;
BLECharacteristic *pInputCharacteristic;
bool clientConnected = false;

// EEG Processing Variables
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

// HID Report Descriptor for Keyboard with Volume Up
// Single report so it works on all ESP32 core versions
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
    REPORT_COUNT(1), 0x06,    //   REPORT_COUNT (6) ; 6 bytes (Keys)
    REPORT_SIZE(1), 0x08,     //   REPORT_SIZE(8)
    LOGICAL_MINIMUM(1), 0x00, //   LOGICAL_MINIMUM(0)
    LOGICAL_MAXIMUM(1), 0x65, //   LOGICAL_MAXIMUM(0x65) ; 101 keys
    USAGE_PAGE(1), 0x07,      //   USAGE_PAGE (Kbrd/Keypad)
    USAGE_MINIMUM(1), 0x00,   //   USAGE_MINIMUM (0)
    USAGE_MAXIMUM(1), 0x65,   //   USAGE_MAXIMUM (0x65)
    HIDINPUT(1), 0x00,        //   INPUT (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    // ------------------------------------------------- Volume Up
    USAGE_PAGE(1), 0x0C,      //   USAGE_PAGE (Consumer)
    USAGE(1), 0xE9,           //   USAGE (Volume Increment)
    LOGICAL_MINIMUM(1), 0x00, //   LOGICAL_MINIMUM (0)
    LOGICAL_MAXIMUM(1), 0x01, //   LOGICAL_MAXIMUM (1)
    REPORT_SIZE(1), 0x01,     //   REPORT_SIZE (1)
    REPORT_COUNT(1), 0x01,    //   REPORT_COUNT (1)
    HIDINPUT(1), 0x02,        //   INPUT (Data,Var,Abs)
    REPORT_COUNT(1), 0x07,    //   REPORT_COUNT (7) ; 7 bits (Padding)
    HIDINPUT(1), 0x01,        //   INPUT (Const,Array,Abs)
    END_COLLECTION(0)         // END_COLLECTION
};

class MyCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    clientConnected = true;
    ledState = LED_GREEN;
    Serial.println("HID connected");
  }

  void onDisconnect(BLEServer *pServer)
  {
    clientConnected = false;
    keyPressed = false;
    ledState = LED_RED;
    Serial.println("HID disconnected");
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

// Pixel 0: red = disconnected, green = connected, blue = blink
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
    if (millis() - lastBlinkLedMs < BLUE_LED_DURATION)
    {
      color = pixel.Color(0, 0, 30);
    }
    else
    {
      ledState = clientConnected ? LED_GREEN : LED_RED;
      color = clientConnected ? pixel.Color(0, 20, 0) : pixel.Color(20, 0, 0);
    }
  }
  if (color != lastPixel0Color)
  {
    lastPixel0Color = color;
    pixel.setPixelColor(BLE_LED, color);
    pixel.show();
  }
}

// Pixel 1: red = Off, blue = Dino mode, green = Camera mode
void updateModeLed()
{
  if (currentMode == MODE_CAMERA)
  {
    pixel.setPixelColor(MODE_LED, pixel.Color(0, 20, 0));
  }
  else if (currentMode == MODE_DINO)
  {
    pixel.setPixelColor(MODE_LED, pixel.Color(0, 0, 20));
  }
  else
  {
    pixel.setPixelColor(MODE_LED, pixel.Color(20, 0, 0));
  }
  pixel.show();
}

// Press Space in Dino mode or Volume Up in Camera mode
void pressKey()
{
  if (!clientConnected || currentMode == MODE_OFF)
  {
    return;
  }

  uint8_t report[REPORT_SIZE_BYTES] = {0};
  if (currentMode == MODE_CAMERA)
  {
    report[8] = VOLUME_UP;
    Serial.println("Volume Up sent!");
  }
  else
  {
    report[2] = KEY_SPACE;
    Serial.println("Space sent!");
  }
  pInputCharacteristic->setValue(report, REPORT_SIZE_BYTES);
  pInputCharacteristic->notify();

  keyPressed = true;
  keyPressTime = millis();
}

// Release all keys
void releaseKey()
{
  if (clientConnected)
  {
    uint8_t report[REPORT_SIZE_BYTES] = {0};
    pInputCharacteristic->setValue(report, REPORT_SIZE_BYTES);
    pInputCharacteristic->notify();
  }
  keyPressed = false;
}

// Toggle mode on BOOT button press
void checkModeButton()
{
  bool buttonState = digitalRead(BUTTON_PIN);
  unsigned long nowMs = millis();

  if (buttonState != lastButtonState && (nowMs - lastButtonTime) >= BUTTON_DEBOUNCE_MS)
  {
    lastButtonTime = nowMs;
    lastButtonState = buttonState;

    if (buttonState == LOW)
    {
      // Cycle Off -> Dino -> Camera -> Off
      if (currentMode == MODE_OFF)
      {
        currentMode = MODE_DINO;
        Serial.println("Dino mode");
      }
      else if (currentMode == MODE_DINO)
      {
        currentMode = MODE_CAMERA;
        Serial.println("Camera mode");
      }
      else
      {
        currentMode = MODE_OFF;
        Serial.println("Off mode");
      }
      updateModeLed();
    }
  }
}

void setup()
{
  pixel.begin();
  pixel.clear();
  pixel.setPixelColor(BLE_LED, pixel.Color(20, 0, 0)); // red = disconnected
  lastPixel0Color = pixel.Color(20, 0, 0);
  pixel.show();
  updateModeLed();

  Serial.begin(BAUD_RATE);
  delay(100);

  pinMode(INPUT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Device name with last two bytes of the BLE MAC address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char deviceName[20];
  snprintf(deviceName, sizeof(deviceName), "Blinky Dino %02x:%02x", mac[4], mac[5]);

  // Initialize BLE HID Device
  BLEDevice::init(deviceName);

  // "Just Works" pairing with bonding and secure connection
  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyCallbacks());

  // Create HID Device
  pHIDDevice = new BLEHIDDevice(pServer);
  pInputCharacteristic = pHIDDevice->inputReport(1); // single report for all keys

  pHIDDevice->manufacturer()->setValue("Upside Down Labs");
  pHIDDevice->pnp(0x02, 0xe502, 0xa111, 0x0210);
  pHIDDevice->hidInfo(0x00, 0x02);

  pHIDDevice->reportMap((uint8_t *)_hidReportDescriptor, sizeof(_hidReportDescriptor));
  pHIDDevice->startServices();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAppearance(HID_KEYBOARD);
  pAdvertising->addServiceUUID(pHIDDevice->hidService()->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // helps with phone connection issues
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Blinky Dino initialized. Waiting for connection...");
}

void loop()
{
  updateBLELed();
  checkModeButton();

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
  }

  unsigned long nowMs = millis();

  // Release key after hold time
  if (keyPressed && (nowMs - keyPressTime) >= KEY_HOLD_MS)
  {
    releaseKey();
  }

  // Single blink detection with debounce
  unsigned long debounceMs = (currentMode == MODE_CAMERA) ? CAMERA_DEBOUNCE_MS : BLINK_DEBOUNCE_MS;
  if (blinkArmed && currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= debounceMs)
  {
    blinkArmed = false;
    lastBlinkTime = nowMs;
    lastBlinkLedMs = nowMs;
    ledState = LED_BLUE_FADE;
    Serial.println("Blink detected");
    pressKey();
  }

  // Re-arm once the envelope settles back down
  if (!blinkArmed && currentEEGEnvelope < BlinkResetThreshold)
  {
    blinkArmed = true;
  }
}
