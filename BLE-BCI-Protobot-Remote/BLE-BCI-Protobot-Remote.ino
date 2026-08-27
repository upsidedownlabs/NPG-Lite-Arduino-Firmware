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
// Copyright (c) 2024-2025 Deepak Khatri      - deepak@upsidedownlabs.tech
// Copyright (c) 2024-2025 Upside Down Labs   - contact@upsidedownlabs.tech
//
// NPG Lite BCI firmware, retargeted to drive a stock/unmodified
// microbots.io ProtoBot directly over BLE - NO firmware changes needed
// on the ProtoBot side.
//
// HOW THIS WORKS:
// ProtoBot ships running the CodeCell-MicroLink library, which exposes
// a BLE GATT "joystick" characteristic that the MicroLink phone app
// normally writes to for driving it. This board now acts as a BLE
// CLIENT that connects directly to that same characteristic and writes
// joystick values itself - so from ProtoBot's perspective it looks
// exactly like the MicroLink app is driving it.
//
// Protocol reverse-derived from the public CodeCell-MicroLink source
// (github.com/microbotsio/CodeCell-MicroLink, src/MicroLink.h/.cpp):
//   Service UUID:     12345678-1234-1234-1234-123456789012
//   Joystick char UUID: abcd1234-abcd-1234-abcd-123456789012 (WRITE)
//   Settings char UUID: dcba4330-dcba-4321-dcba-432123456789 (WRITE)
//   Value = 2 bytes [X, Y], range 0-200, 100 = center/neutral.
//     X = steering/direction, Y = throttle/speed (per library comments)
//   Device advertises as "CodeCell".
//
// NOTE: X/Y -> actual turn direction is inferred from the library's
// variable naming/comments, not from an official spec - verify on the
// bench and flip JOY_LEFT/JOY_RIGHT below if it turns the wrong way.
//

#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Arduino.h>
#include "esp_dsp.h"
#include <vector>
#include <Preferences.h>

// ---------------------------------------------------------------
//  ProtoBot / MicroLink BLE protocol (client side)
// ---------------------------------------------------------------
// This is the UUID ProtoBot actually broadcasts in its BLE advertising
// packet (confirmed via LightBlue scan) - used ONLY to find/filter the
// device while scanning.
static BLEUUID protobotAdvertisedUUID("00008018-0000-1000-8000-00805F9B34FB");

// This is the real GATT service that contains the joystick/settings
// characteristics, only visible AFTER connecting (confirmed via
// LightBlue's connected service browser - matches the public
// CodeCell-MicroLink library source exactly).
static BLEUUID joystickServiceUUID("12345678-1234-1234-1234-123456789012");
static BLEUUID joystickCharUUID   ("abcd1234-abcd-1234-abcd-123456789012");
static BLEUUID settingsCharUUID   ("dcba4330-dcba-4321-dcba-432123456789");
static BLEUUID shapeCharUUID      ("dcba4327-dcba-4321-dcba-432123456789");
const char* PROTOBOT_DEVICE_NAME = "protobot";

// Infinity-shape trigger (via SHAPE_UUID characteristic). Format:
// [shapeOnOFF(1), shape_num(4=infinity), shape_thr(0-100 size), shape_loop(0=once)]
const uint8_t SHAPE_NUM_INFINITY = 4;
const uint8_t SHAPE_THR_DEFAULT  = 50;   // 0-100, controls loop size/speed

// Joystick value range/semantics (from MicroLink source: 0-200, 100=center)
const uint8_t JOY_CENTER = 100;
const uint8_t JOY_FWD    = 0;    // Y above center = forward
const uint8_t JOY_BWD    = 200;  // Y below center = backward
const uint8_t JOY_LEFT   = 0;    // X below center = left  (flip with JOY_RIGHT if reversed)
const uint8_t JOY_RIGHT  = 200;  // X above center = right

// ---------------------------------------------------------------
//  Eye/display colors per direction - tune to taste (R,G,B 0-255)
// ---------------------------------------------------------------
const uint8_t COLOR_STOP[3]      = { 255, 255, 255 };  // white
const uint8_t COLOR_FORWARD[3]   = { 0,   255, 0   };  // green
const uint8_t COLOR_BACKWARD[3]  = { 255, 0,   0   };  // red
const uint8_t COLOR_LEFT[3]      = { 255, 200, 0   };  // amber/yellow
const uint8_t COLOR_RIGHT[3]     = { 255, 200, 0   };  // amber/yellow
const uint8_t COLOR_INFINITY[3]  = { 160, 32,  240 };  // purple

// ---------------------------------------------------------------
//  Hardware pins
// ---------------------------------------------------------------
#define PIN_NEOPIXEL 15
#define PIN_LED_VIB 7  // NPG Lite: shared LED + vibration motor

Adafruit_NeoPixel pixel(6, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5

// ---------------------------------------------------------------
//  Signal processing config
// ---------------------------------------------------------------
#define SAMPLE_RATE 512
#define FFT_SIZE 512
#define BAUD_RATE 115200
#define INPUT_PIN1 A0           // EEG
#define INPUT_PIN2 A1           // Right EMG
#define INPUT_PIN3 A2           // Left EMG
#define BATTERY_VOLTAGE_PIN A6  // Battery connected ADC
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
// ---------------------------------------------------------------
#define DEBUG_PRINT_EVERY_N_SAMPLES 100

// ---------------------------------------------------------------
//  Thresholds - loaded from NVS on boot
// ---------------------------------------------------------------
uint32_t betaThreshold = 10;
uint32_t emg1Threshold = 150;
uint32_t emg2Threshold = 150;

// ---------------------------------------------------------------
//  Blink detection (ported from the drone firmware's blink logic) -
//  triple blink triggers the ProtoBot infinity-shape drive.
// ---------------------------------------------------------------
float BlinkThreshold = 50.0;
#define ENVELOPE_WINDOW_MS 100
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)
const unsigned long BLINK_DEBOUNCE_MS = 250;
const unsigned long DOUBLE_BLINK_MS   = 800;
const unsigned long TRIPLE_BLINK_MS   = 1000;

float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = { 0 };
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

unsigned long lastBlinkTime = 0;
unsigned long firstBlinkTime = 0;
int blinkCount = 0;

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
//  BLE client state (connects OUT to ProtoBot)
// ---------------------------------------------------------------
static BLEAdvertisedDevice*     myDevice        = nullptr;
static BLERemoteCharacteristic* pJoystickChar   = nullptr;
static BLERemoteCharacteristic* pSettingsChar   = nullptr;
static BLERemoteCharacteristic* pShapeChar      = nullptr;
static BLEClient*               pClient         = nullptr;

static boolean doConnect = false;
static boolean doScan    = false;
bool deviceConnected     = false;   // true once connected to ProtoBot

// ---------------------------------------------------------------
//  Control state
// ---------------------------------------------------------------
bool isGoingBackward = false;
uint32_t lastSentCmd = 255;

// ── BLE LED state machine ──
enum LedState {
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
BandpowerResults smoothedPowers = { 0, 0, 0, 0, 0, 0 };

// ---------------------------------------------------------------
//  Battery level indication
// ---------------------------------------------------------------
static const unsigned long BATTERY_CHECK_INTERVAL = 10000;
static unsigned long lastBatteryCheck = 0;

uint32_t batteryColor = 0;
static uint32_t batteryWinSum = 0;
static uint16_t batteryWinCount = 0;
static int lastBatteryPct = -1;
static uint8_t risingCount = 0;
static const uint8_t RISING_THRESHOLD = 3;
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

// ----------------- NOTCH FILTER CLASSES -----------------
class NotchFilter {
  struct BiquadState {
    float z1 = 0, z2 = 0;
  };
  BiquadState s1, s2;

public:
  float process(float in) {
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
  void reset() {
    s1.z1 = s1.z2 = s2.z1 = s2.z2 = 0;
  }
};

class EMGHighPassFilter {
  double z1 = 0, z2 = 0;

public:
  double process(double in) {
    double x = in - (-0.82523238) * z1 - (0.29463653) * z2;
    double out = 0.52996723 * x + (-1.05993445) * z1 + 0.52996723 * z2;
    z2 = z1;
    z1 = x;
    return out;
  }
  void reset() {
    z1 = z2 = 0;
  }
};

class EnvelopeFilter {
  std::vector<double> buf;
  double sum = 0;
  int idx = 0;
  const int sz;

public:
  EnvelopeFilter(int s)
    : sz(s) {
    buf.resize(s, 0.0);
  }
  double getEnvelope(double v) {
    sum -= buf[idx];
    sum += v;
    buf[idx] = v;
    idx = (idx + 1) % sz;
    return sum / sz;
  }
};

float EEGFilter(float in) {
  static float z1 = 0, z2 = 0;
  float x = in - (-1.22465158f) * z1 - (0.45044543f) * z2;
  float out = 0.05644846f * x + 0.11289692f * z1 + 0.05644846f * z2;
  z2 = z1;
  z1 = x;
  return out;
}

// Separate standalone highpass used specifically to isolate blink
// transients from the EEG-filtered signal (ported from drone firmware).
float highpass(float in) {
  static float z1 = 0, z2 = 0;
  float x = in - (-1.91327599f) * z1 - (0.91688335f) * z2;
  float out = 0.95753983f * x + (-1.91507967f) * z1 + 0.95753983f * z2;
  z2 = z1;
  z1 = x;
  return out;
}

float updateEEGEnvelope(float sample) {
  float absSample = fabs(sample);
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return envelopeSum / ENVELOPE_WINDOW_SIZE;
}

NotchFilter filters[3];
EMGHighPassFilter emgfilters[2];
EnvelopeFilter Envelopefilter1(16);
EnvelopeFilter Envelopefilter2(16);

// ---------------------------------------------------------------
//  Forward declarations
// ---------------------------------------------------------------
void printHelp();

// ---------------------------------------------------------------
//  Joystick write helper - talks directly to ProtoBot's stock
//  MicroLink joystick characteristic
// ---------------------------------------------------------------
void writeJoystick(uint8_t x, uint8_t y) {
  if (!deviceConnected || pJoystickChar == nullptr) return;
  uint8_t buf[2] = { x, y };
  pJoystickChar->writeValue(buf, 2, false);  // write WITHOUT response - matches official ProtoBot joystick.ino example
}

// ---------------------------------------------------------------
//  Eye/display color helper - writes the full 10-byte settings
//  packet (same fields as before) but with the given RGB color.
// ---------------------------------------------------------------
void updateEyeColor(uint8_t r, uint8_t g, uint8_t b) {
  if (pSettingsChar == nullptr) return;
  uint8_t settings[10] = {
    1,     // [0] isAppConnectionReady
    0,     // [1] avoidOnOFF
    0,     // [2] FrontAvoidOnOFF
    0,     // [3] faceNorthOnOFF
    0xFF,  // [4] displayOnOFF
    r,     // [5] displayRed
    g,     // [6] displayGreen
    b,     // [7] displayBlue
    3,     // [8] speed_reduction (3 -> full speed)
    0      // [9] touchGesture
  };
  pSettingsChar->writeValue(settings, sizeof(settings), false);
}

// ---------------------------------------------------------------
//  Infinity-shape trigger - fires the ProtoBot's built-in
//  DriveInfinity() routine via the SHAPE_UUID characteristic.
//  shape_loop=0 means it runs the figure-8 once and the firmware
//  auto-clears shapeOnOFF back to 0 when it completes - which we
//  detect via BLE notify on the same characteristic (see
//  shapeNotifyCallback below), with a timeout as a safety fallback.
// ---------------------------------------------------------------
bool infinityShapeActive = false;
unsigned long infinityShapeStartMs = 0;
const unsigned long INFINITY_SHAPE_TIMEOUT_MS = 15000;  // safety: re-arm even if notify is missed

void triggerInfinityShape() {
  if (!deviceConnected || pShapeChar == nullptr) return;
  if (infinityShapeActive) return;  // already running, ignore repeat triggers

  uint8_t shape[4] = {
    1,                    // [0] shapeOnOFF
    SHAPE_NUM_INFINITY,   // [1] shape_num (4 = infinity)
    SHAPE_THR_DEFAULT,    // [2] shape_thr (0-100, size/speed)
    0                     // [3] shape_loop (0 = once)
  };
  pShapeChar->writeValue(shape, sizeof(shape), false);
  infinityShapeActive = true;
  infinityShapeStartMs = millis();
  updateEyeColor(COLOR_INFINITY[0], COLOR_INFINITY[1], COLOR_INFINITY[2]);
  Serial.println("Triple blink detected -> infinity shape triggered! (other commands paused)");
}

// Maps a cmd (0-4) to its display color
void colorForCommand(uint32_t cmd, uint8_t &r, uint8_t &g, uint8_t &b) {
  const uint8_t *c;
  switch (cmd) {
    case 1:  c = COLOR_LEFT;     break;
    case 2:  c = COLOR_RIGHT;    break;
    case 3:  c = COLOR_FORWARD;  break;
    case 4:  c = COLOR_BACKWARD; break;
    default: c = COLOR_STOP;     break;
  }
  r = c[0]; g = c[1]; b = c[2];
}

// Maps a cmd (0-4) to its joystick X/Y pair
void joystickForCommand(uint32_t cmd, uint8_t &x, uint8_t &y) {
  switch (cmd) {
    case 1:  x = JOY_LEFT;   y = JOY_CENTER; break;  // turn left
    case 2:  x = JOY_RIGHT;  y = JOY_CENTER; break;  // turn right
    case 3:  x = JOY_CENTER; y = JOY_FWD;    break;  // forward
    case 4:  x = JOY_CENTER; y = JOY_BWD;    break;  // backward
    default: x = JOY_CENTER; y = JOY_CENTER; break;  // stop
  }
}

// ---------------------------------------------------------------
//  Continuous joystick refresh
//
//  Some BLE-controlled robots (ProtoBot included, based on observed
//  behavior) apply a failsafe: if no fresh joystick write arrives
//  within a short window, they auto-stop (in case the controller app
//  disconnects mid-drive). A single write-on-change is NOT enough -
//  we must keep re-sending the current command at a steady rate for
//  as long as it's active, or the bot will randomly stop.
// ---------------------------------------------------------------
uint32_t currentCommand = 0;
uint32_t lastLoggedCmd = 255;
unsigned long lastJoystickSendMs = 0;
const unsigned long JOYSTICK_REFRESH_MS = 100;  // resend at ~10 Hz

void sendJoystickNow() {
  uint8_t x, y;
  joystickForCommand(currentCommand, x, y);
  writeJoystick(x, y);
  lastJoystickSendMs = millis();
}

// applyCommand - only writes over BLE when the command actually
// CHANGES (critical: this gets called at up to 512Hz from the EMG
// sample loop while a threshold is held, so writing on every call
// floods the BLE stack and eventually jams it - which is why the
// robot appeared to "freeze" on the last command after a few
// seconds). The periodic ~10Hz refresh in loop() is solely
// responsible for keeping the command alive/repeated over time.
void applyCommand(uint32_t cmd) {
  currentCommand = cmd;

  if (cmd == lastLoggedCmd)
    return;  // no change -> no write, let the periodic refresh handle repetition

  lastLoggedCmd = cmd;
  lastSentCmd = cmd;
  Serial.print("cmd: ");
  Serial.println(cmd);
  lastCmdSentMs = millis();
  ledState = LED_BLUE_FADE;

  sendJoystickNow();

  uint8_t r, g, b;
  colorForCommand(cmd, r, g, b);
  updateEyeColor(r, g, b);
}

// ---------------------------------------------------------------
//  Shape-characteristic notify callback - detects when ProtoBot's
//  firmware auto-clears shapeOnOFF back to 0 (infinity shape done)
//  and resumes normal joystick/color control immediately.
// ---------------------------------------------------------------
static void shapeNotifyCallback(BLERemoteCharacteristic* pChar,
                                 uint8_t* pData, size_t length, bool isNotify) {
  if (length > 0 && pData[0] == 0 && infinityShapeActive) {
    infinityShapeActive = false;
    lastLoggedCmd = 255;  // force an immediate fresh joystick+color write on the next sample
    lastSentCmd   = 255;
    Serial.println("Infinity shape complete - resuming normal control");
  }
}

// ---------------------------------------------------------------
//  Serial command parser
// ---------------------------------------------------------------
void handleSerialCommands() {
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();

  if (line == "debug") {
    debugEnabled = true;
    Serial.println("Debug mode ENABLED");
    Serial.print("Print rate: every ");
    Serial.print(DEBUG_PRINT_EVERY_N_SAMPLES);
    Serial.print(" samples (~");
    Serial.print(SAMPLE_RATE / DEBUG_PRINT_EVERY_N_SAMPLES);
    Serial.println(" Hz)");
    return;
  }

  if (line == "exit") {
    debugEnabled = false;
    Serial.println("Debug mode DISABLED");
    return;
  }

  if (line == "status") {
    Serial.println("--------------------------------------------");
    Serial.println("BLE        : " + String(deviceConnected ? "Connected to ProtoBot" : "Disconnected"));
    Serial.println("Infinity   : " + String(infinityShapeActive ? "RUNNING (other cmds paused)" : "idle"));
    Serial.println("--------------------------------------------");
    return;
  }

  if (line.startsWith("set ")) {
    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);
    if (secondSpace == -1) {
      Serial.println("Usage: set <betathreshold|emg1threshold|emg2threshold|blinkthreshold> <value>");
      return;
    }
    String key = line.substring(firstSpace + 1, secondSpace);
    String valStr = line.substring(secondSpace + 1);
    valStr.trim();
    uint32_t val = (uint32_t)valStr.toInt();

    prefs.begin("thresholds", false);

    if (key == "betathreshold") {
      betaThreshold = val;
      prefs.putUInt("betathr", betaThreshold);
      Serial.print("betaThreshold set to ");
      Serial.println(betaThreshold);
    } else if (key == "emg1threshold") {
      emg1Threshold = val;
      prefs.putUInt("emg1thr", emg1Threshold);
      Serial.print("emg1Threshold set to ");
      Serial.println(emg1Threshold);
    } else if (key == "emg2threshold") {
      emg2Threshold = val;
      prefs.putUInt("emg2thr", emg2Threshold);
      Serial.print("emg2Threshold set to ");
      Serial.println(emg2Threshold);
    } else if (key == "blinkthreshold") {
      BlinkThreshold = (float)val;
      prefs.putFloat("blinkthr", BlinkThreshold);
      Serial.print("blinkThreshold set to ");
      Serial.println(BlinkThreshold);
    } else {
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
//  BLE status LED
// ---------------------------------------------------------------
void updateBLELed() {
  uint32_t color;

  if (ledState == LED_RED) {
    color = pixel.Color(20, 0, 0);
  } else if (ledState == LED_GREEN) {
    color = pixel.Color(0, 20, 0);
  } else {
    unsigned long elapsed = millis() - lastCmdSentMs;
    if (elapsed < BLUE_LED_DURATION) {
      color = pixel.Color(0, 0, 30);
    } else {
      ledState = LED_GREEN;
      color = pixel.Color(0, 20, 0);
    }
  }

  if (color != lastPixel0Color) {
    lastPixel0Color = color;
    pixel.setPixelColor(BLE_LED, color);
    pixel.show();
  }
}

// ---------------------------------------------------------------
//  BLE client callbacks
// ---------------------------------------------------------------
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}
  void onDisconnect(BLEClient* pclient) {
    deviceConnected = false;
    lastSentCmd = 255;
    infinityShapeActive = false;
    digitalWrite(PIN_LED_VIB, LOW);
    ledState = LED_RED;
    pixelDirty = true;
    Serial.println("ProtoBot disconnected, re-scanning...");
    doScan = true;
  }
};

bool connectToProtoBot() {
  if (pClient) {
    if (pClient->isConnected()) pClient->disconnect();
    delete pClient;
    pClient = nullptr;
  }
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  if (!pClient->connect(myDevice)) return false;

  BLERemoteService* svc = pClient->getService(joystickServiceUUID);
  if (!svc) { pClient->disconnect(); return false; }

  pJoystickChar = svc->getCharacteristic(joystickCharUUID);
  if (!pJoystickChar) { pClient->disconnect(); return false; }

  pSettingsChar = svc->getCharacteristic(settingsCharUUID);
  updateEyeColor(COLOR_STOP[0], COLOR_STOP[1], COLOR_STOP[2]);  // full settings packet (incl. speed_reduction) + starting color

  pShapeChar = svc->getCharacteristic(shapeCharUUID);
  if (pShapeChar && pShapeChar->canNotify()) {
    pShapeChar->registerForNotify(shapeNotifyCallback);
  }
  infinityShapeActive = false;

  // Start centered/stopped, and reset the continuous-refresh state
  currentCommand    = 0;
  lastLoggedCmd     = 255;
  lastSentCmd       = 255;
  writeJoystick(JOY_CENTER, JOY_CENTER);
  lastJoystickSendMs = millis();

  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (!dev.haveServiceUUID() || !dev.isAdvertisingService(protobotAdvertisedUUID)) return;

    BLEDevice::getScan()->stop();
    if (myDevice) delete myDevice;
    myDevice  = new BLEAdvertisedDevice(dev);
    doConnect = true;
    doScan    = false;
    Serial.println("ProtoBot found: " + String(dev.getAddress().toString().c_str()));
  }
};

// ---------------------------------------------------------------
//  Bandpower helpers
// ---------------------------------------------------------------
BandpowerResults calculateBandpower(float *ps, float binRes, int half) {
  BandpowerResults r = { 0, 0, 0, 0, 0, 0 };
  for (int i = 1; i < half; i++) {
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

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s) {
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
void initFFT() {
  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if (err != ESP_OK) {
    Serial.println("FFT init failed");
    while (1)
      delay(10);
  }
}

void processFFT() {
  for (int i = 0; i < FFT_SIZE; i++) {
    y_cf[2 * i] = inputBuffer[i];
    y_cf[2 * i + 1] = 0;
  }
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  int half = FFT_SIZE / 2;
  for (int i = 0; i < half; i++) {
    float re = y1_cf[2 * i], im = y1_cf[2 * i + 1];
    powerSpectrum[i] = re * re + im * im;
  }

  BandpowerResults raw = calculateBandpower(powerSpectrum, float(SAMPLE_RATE) / FFT_SIZE, half);
  smoothBandpower(&raw, &smoothedPowers);
  float T = smoothedPowers.total + EPS;
  gBetaPct = (smoothedPowers.beta / T) * 100.0f;

  // Paused while the infinity shape is running - ProtoBot's own
  // ShapeHandler owns the motors until it finishes.
  if (deviceConnected && !infinityShapeActive) {
    if (gBetaPct > betaThreshold && !isGoingBackward) {
      applyCommand(3);
    } else {
      applyCommand(0);
    }
  }
}

// ---------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------
void setup() {
  pixel.begin();
  pixel.clear();
  pixel.show();

  Serial.begin(BAUD_RATE);

  prefs.begin("thresholds", true);
  betaThreshold = prefs.getUInt("betathr", betaThreshold);
  emg1Threshold = prefs.getUInt("emg1thr", emg1Threshold);
  emg2Threshold = prefs.getUInt("emg2thr", emg2Threshold);
  BlinkThreshold = prefs.getFloat("blinkthr", BlinkThreshold);
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

  BLEDevice::init("NPG-Lite-Controller");

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setActiveScan(true);

  Serial.println("Scanning for any ProtoBot (CodeCell)...");
  doScan = true;
  scan->start(5, false);

  int initBattery = getCurrentBatteryPercentage();
  if (initBattery <= 20) {
    batteryColor = pixel.Color(20, 0, 0);
  } else if (initBattery <= 70) {
    batteryColor = pixel.Color(35, 7, 0);
  } else {
    batteryColor = pixel.Color(0, 20, 0);
  }
  pixel.setPixelColor(BATTERY_LED, batteryColor);
  pixel.show();

  printHelp();
}

void printHelp() {
  Serial.println("============================================");
  Serial.println("  NPG Lite -> ProtoBot direct control        ");
  Serial.println("  status                     - show info     ");
  Serial.println("  debug / exit               - BCI debug log ");
  Serial.println("  set betathreshold  <n>                     ");
  Serial.println("  set emg1threshold  <n>                     ");
  Serial.println("  set emg2threshold  <n>                     ");
  Serial.println("  set blinkthreshold <n>                     ");
  Serial.println("  Triple blink -> ProtoBot infinity shape!   ");
  Serial.println("  (other commands pause until it completes)  ");
  Serial.println("============================================");
}

// ---------------------------------------------------------------
//  Loop
// ---------------------------------------------------------------
unsigned long lastReconnect = 0;
const unsigned long RECONNECT_INTERVAL = 3000;

void loop() {
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  static long timer = 0;
  static int debugSampleCount = 0;

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  handleSerialCommands();

  // --- Connect to ProtoBot when found ---
  if (doConnect) {
    doConnect = false;
    if (connectToProtoBot()) {
      deviceConnected = true;
      ledState = LED_GREEN;
      pixelDirty = true;
      Serial.println("Connected to ProtoBot");
    } else {
      Serial.println("Connect failed, retrying...");
      doScan = true;
    }
  }

  // --- Re-scan while not connected ---
  if (!deviceConnected && doScan) {
    unsigned long nowMs = millis();
    if (nowMs - lastReconnect >= RECONNECT_INTERVAL) {
      lastReconnect = nowMs;
      BLEDevice::getScan()->start(3, false);
    }
  }

  if (pixelDirty) {
    pixel.setPixelColor(BATTERY_LED, batteryColor);
    pixel.show();
    pixelDirty = false;
  }
  updateBLELed();

  // --- Keep the active joystick command flowing at a steady rate ---
  // Paused while the infinity shape is running, so we don't fight
  // ProtoBot's own ShapeHandler with stale joystick writes.
  if (deviceConnected && !infinityShapeActive && (millis() - lastJoystickSendMs >= JOYSTICK_REFRESH_MS)) {
    sendJoystickNow();
  }

  unsigned long currentMillis = millis();

  if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    int currentBattery = getCurrentBatteryPercentage();
    if (currentBattery <= 20) {
      batteryColor = pixel.Color(20, 0, 0);
    } else if (currentBattery <= 70) {
      batteryColor = pixel.Color(35, 7, 0);
    } else {
      batteryColor = pixel.Color(0, 20, 0);
    }
    pixelDirty = true;
    lastBatteryCheck = currentMillis;
  }

  timer -= dt;
  if (timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;

    int raw1 = analogRead(INPUT_PIN1);
    int raw2 = analogRead(INPUT_PIN2);
    int raw3 = analogRead(INPUT_PIN3);
    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

    float filteeg = EEGFilter(filters[0].process(raw1));
    float filtemg1 = emgfilters[0].process(filters[1].process(raw2));
    float filtemg2 = emgfilters[1].process(filters[2].process(raw3));

    inputBuffer[idx++] = filteeg;

    gEnv1 = Envelopefilter1.getEnvelope(abs(filtemg1));
    gEnv2 = Envelopefilter2.getEnvelope(abs(filtemg2));
    currentEEGEnvelope = updateEEGEnvelope(highpass(filteeg));

    if (debugEnabled) {
      if (++debugSampleCount >= DEBUG_PRINT_EVERY_N_SAMPLES) {
        debugSampleCount = 0;
        Serial.print("beta: ");
        Serial.print(gBetaPct, 2);
        Serial.print("  EMG1: ");
        Serial.print(gEnv1, 2);
        Serial.print("  EMG2: ");
        Serial.print(gEnv2, 2);
        Serial.print("  blink: ");
        Serial.println(currentEEGEnvelope, 2);
      }
    } else {
      debugSampleCount = 0;
    }

    // ---- BLINK DETECTION (ported from drone firmware) ----
    // Triple blink within TRIPLE_BLINK_MS triggers the ProtoBot's
    // built-in infinity-shape drive. (Blink detection itself always
    // stays active, even during a running shape - triggerInfinityShape()
    // ignores repeat triggers on its own via the infinityShapeActive check.)
    unsigned long nowMs = millis();
    if (currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
      lastBlinkTime = nowMs;
      if (blinkCount == 0) {
        firstBlinkTime = nowMs;
        blinkCount = 1;
      } else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
        blinkCount = 2;
      } else if (blinkCount == 2 && (nowMs - firstBlinkTime) <= TRIPLE_BLINK_MS) {
        triggerInfinityShape();
        blinkCount = 0;
      } else {
        firstBlinkTime = nowMs;
        blinkCount = 1;
      }
    }
    if (blinkCount == 2 && (nowMs - firstBlinkTime) > TRIPLE_BLINK_MS)
      blinkCount = 0;
    if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS)
      blinkCount = 0;

    // Paused while the infinity shape is running - ProtoBot's own
    // ShapeHandler owns the motors until it finishes (via notify or
    // the timeout fallback below).
    if (deviceConnected && !infinityShapeActive) {
      if (gEnv1 > emg1Threshold * 0.5 && gEnv2 > emg2Threshold * 0.5) {
        isGoingBackward = true;
        applyCommand(4);
      } else if (gEnv1 > emg1Threshold && !isGoingBackward) {
        isGoingBackward = false;
        applyCommand(2);
      } else if (gEnv2 > emg2Threshold && !isGoingBackward) {
        isGoingBackward = false;
        applyCommand(1);
      } else {
        isGoingBackward = false;
      }
    }
  }

  // Safety re-arm: if we never hear back that the shape finished
  // (e.g. a dropped notify), allow new triple-blink triggers after a
  // timeout rather than staying locked out / purple forever.
  if (infinityShapeActive && (millis() - infinityShapeStartMs >= INFINITY_SHAPE_TIMEOUT_MS)) {
    infinityShapeActive = false;
    lastLoggedCmd = 255;
    lastSentCmd   = 255;
    Serial.println("Infinity shape timeout - resuming normal control");
  }

  if (idx >= FFT_SIZE) {
    processFFT();
    idx = 0;
  }
}
