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

// Copyright (c) 2025 Aman Maheshwari - Aman@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2024 - 2025 Upside Down Labs - contact@upsidedownlabs.tech

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "esp_dsp.h"
#include <vector>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Preferences.h>

typedef struct
{
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

// Preferences object for saving thresholds
Preferences preferences;

WiFiUDP udp;

// Forward declarations (some Arduino/ESP32 toolchains don't auto-prototype
// functions defined later in the file when they're used earlier, e.g.
// droneToggleFlight() being called inside processFFT()).
void droneToggleFlight();
void droneEmergency();
void droneSendCommand(uint8_t cmd);
bool connectToDrone();
void updateDroneHeartbeat();
void updateDroneControlLoop();
void sendDroneHeartbeatPacket();
void sendDroneControlPacket();
void buildDroneControlMessage(uint8_t *out, int &len);

// ===== DRONE CONNECTION =====
// drone broadcasts an SSID like
// "FLOW-UFO-XXXXXX", so we scan for anything starting with this prefix
// instead of using a fixed SSID.
const char *DRONE_SSID_PREFIX = "FLOW-UFO-";
const char *DRONE_PASSWORD = "";  // set this if your FLOW-UFO drone requires a password
const char *DRONE_IP = "192.168.1.1";
const int DRONE_PORT = 7099;
const int LOCAL_UDP_PORT = 8890;  // arbitrary local port for the outgoing socket

// Ported from
const unsigned long CONTROL_INTERVAL_MS = 50;
const unsigned long HEARTBEAT_INTERVAL_MS = 50;
const int HEARTBEAT_BURST_COUNT = 7;
const unsigned long HEARTBEAT_BURST_INTERVAL_MS = 50;
const unsigned long COMMAND_HOLD_MS = 500;

const uint8_t DRONE_CMD_NONE = 0x00;
const uint8_t DRONE_CMD_TAKE_OFF = 0x01;  // shared by takeoff & land - drone toggles flight state
const uint8_t DRONE_CMD_EMERGENCY = 0x02;
const uint8_t DRONE_CMD_UNLOCK_MOTOR = 0x40;
const uint8_t DRONE_CMD_CALIBRATE_GYRO = 0x80;

// Joystick-style stick values (0-255, 128 = neutral)
const uint8_t STICK_NEUTRAL = 128;
const uint8_t STICK_THROTTLE_UP = 158;
const uint8_t STICK_THROTTLE_DOWN = 78;
const uint8_t STICK_PITCH_FORWARD = 178;
const uint8_t STICK_TURN = 158;

uint8_t droneRoll = STICK_NEUTRAL;
uint8_t dronePitch = STICK_NEUTRAL;
uint8_t droneThrottle = STICK_NEUTRAL;
uint8_t droneTurn = STICK_NEUTRAL;
uint8_t droneCurrentCommand = DRONE_CMD_NONE;
unsigned long droneCommandStartTime = 0;

unsigned long lastControlSendTime = 0;
unsigned long lastHeartbeatTime = 0;
int heartbeatBurstSent = 0;
bool heartbeatBurstDone = false;

// UDP connection status
bool udpConnected = false;
unsigned long lastConnectionCheck = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 5000;

// OTA Configuration
const char *ap_ssid = "NPG-Lite-UFO";
const char *ap_password = "12345678";
WebServer server(80);
bool otaMode = false;
unsigned long bootTime = 0;
const unsigned long OTA_TRIGGER_DELAY = 1000;

// Live data for web interface
float currentBetaPercent = 0;
float currentEMG1 = 0;
float currentEMG2 = 0;
bool TakeoffSent = false;  // <-- true after we've sent the one-time toggle for this connection
unsigned long lastWebUpdate = 0;
const unsigned long WEB_UPDATE_INTERVAL = 200;

// Some variables to keep track on device connected
Adafruit_NeoPixel pixel(6, 15, NEO_GRB + NEO_KHZ800);
float betaThreshold = 8;
float EMG_THRESHOLD_LEFT = 150.0;
float EMG_THRESHOLD_RIGHT = 150.0;

// ----------------- USER CONFIGURATION -----------------
#define SAMPLE_RATE 512
#define FFT_SIZE 512
#define BAUD_RATE 115200
#define INPUT_PIN1 A0
#define INPUT_PIN2 A1
#define INPUT_PIN3 A2
#define BOOT_BUTTON 9
#define BUZZER_PIN 8
#define LED_MOTOR_PIN 7

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

#define SMOOTHING_FACTOR 0.63f
#define EPS 1e-7f
#define NOTE_C4 262
#define NOTE_G3 196
#define NOTE_A3 220
#define NOTE_B3 247

#define ENVELOPE_WINDOW_MS 100
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

const unsigned long BLINK_DEBOUNCE_MS = 250;
const unsigned long DOUBLE_BLINK_MS = 800;
const unsigned long TRIPLE_BLINK_MS = 1000;

// ===== JAW CLENCH CONFIGURATION =====
const unsigned long JAW_DEBOUNCE_MS = 500;
const unsigned long JAW_BLOCK_DURATION_MS = 500;
float JAW_ON_THRESHOLD = 60.0;
float JAW_OFF_THRESHOLD = 50.0;

unsigned long lastBlinkTime = 0;
unsigned long firstBlinkTime = 0;
int blinkCount = 0;
bool is_rotation_mode = LOW;

// Jaw clench variables
unsigned long lastJawDetectionTime = 0;
unsigned long lastJawClenchTime = 0;
bool jawState = false;

float BlinkThreshold = 50.0;

// Emergency stop variables
bool emergencyStop = false;
unsigned long lastButtonPressTime = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 500;
unsigned long lastLandCommandTime = 0;

// Police LED and buzzer timing
unsigned long lastPoliceFlashTime = 0;
unsigned long lastBuzzerToggleTime = 0;
bool policeRedState = true;
bool buzzerState = false;

int melody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = { 0 };
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

float jawEnvelopeBuffer[ENVELOPE_WINDOW_SIZE] = { 0 };
int jawEnvelopeIndex = 0;
float jawEnvelopeSum = 0;
float currentJawEnvelope = 0;
static bool firstCommandSent = false;
static bool secondCommandScheduled = false;

// ----------------- BUFFERS & TYPES -----------------
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];

__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

BandpowerResults smoothedPowers = { 0, 0, 0, 0, 0, 0 };

// ----------------- NOTCH FILTER CLASSES -----------------
class NotchFilter {
private:
  struct BiquadState {
    float z1 = 0;
    float z2 = 0;
  };
  BiquadState state1;
  BiquadState state2;

public:
  float process(float input) {
    float output = input;
    float x = output - (-1.56858163f * state1.z1) - (0.96424138f * state1.z2);
    output = 0.96508099f * x + (-1.56202714f * state1.z1) + (0.96508099f * state1.z2);
    state1.z2 = state1.z1;
    state1.z1 = x;
    x = output - (-1.61100358f * state2.z1) - (0.96592171f * state2.z2);
    output = 1.00000000f * x + (-1.61854514f * state2.z1) + (1.00000000f * state2.z2);
    state2.z2 = state2.z1;
    state2.z1 = x;
    return output;
  }
  void reset() {
    state1.z1 = state1.z2 = 0;
    state2.z1 = state2.z2 = 0;
  }
};

float highpass(float input) {
  float output = input;
  {
    static float z1 = 0, z2 = 0;
    float x = output - -1.91327599 * z1 - 0.91688335 * z2;
    output = 0.95753983 * x + -1.91507967 * z1 + 0.95753983 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

class EMGHighPassFilter {
private:
  double z1 = 0.0;
  double z2 = 0.0;

public:
  double process(double input) {
    const double x = input - -0.82523238 * z1 - 0.29463653 * z2;
    const double output = 0.52996723 * x + -1.05993445 * z1 + 0.52996723 * z2;
    z2 = z1;
    z1 = x;
    return output;
  }

  void reset() {
    z1 = 0.0;
    z2 = 0.0;
  }
};

class EnvelopeFilter {
private:
  std::vector<double> circularBuffer;
  double sum = 0.0;
  int dataIndex = 0;
  const int bufferSize;

public:
  EnvelopeFilter(int bufferSize)
    : bufferSize(bufferSize) {
    circularBuffer.resize(bufferSize, 0.0);
  }

  double getEnvelope(double absEmg) {
    sum -= circularBuffer[dataIndex];
    sum += absEmg;
    circularBuffer[dataIndex] = absEmg;
    dataIndex = (dataIndex + 1) % bufferSize;
    return (sum / bufferSize);
  }

  void reset() {
    sum = 0.0;
    dataIndex = 0;
    for (int i = 0; i < bufferSize; i++)
      circularBuffer[i] = 0.0;
  }
};

float EEGFilter(float input) {
  float output = input;
  {
    static float z1 = 0, z2 = 0;
    float x = output - -1.22465158 * z1 - 0.45044543 * z2;
    output = 0.05644846 * x + 0.11289692 * z1 + 0.05644846 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

float jawClenchFilter(float input) {
  float output = input;
  {
    static float z1 = 0, z2 = 0;
    float x = output - -0.82523238 * z1 - 0.29463653 * z2;
    output = 0.52996723 * x + -1.05993445 * z1 + 0.52996723 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

float updateEEGEnvelope(float sample) {
  float absSample = fabs(sample);
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return envelopeSum / ENVELOPE_WINDOW_SIZE;
}

float updateJawEnvelope(float sample) {
  float absSample = fabs(sample);
  jawEnvelopeSum -= jawEnvelopeBuffer[jawEnvelopeIndex];
  jawEnvelopeSum += absSample;
  jawEnvelopeBuffer[jawEnvelopeIndex] = absSample;
  jawEnvelopeIndex = (jawEnvelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return jawEnvelopeSum / ENVELOPE_WINDOW_SIZE;
}

NotchFilter notchFilters[3];
EMGHighPassFilter emgFilters[3];
EnvelopeFilter envelopeFilterLeft(32);   // For left hand EMG
EnvelopeFilter envelopeFilterRight(32);  // For right hand EMG

// Function to load thresholds from preferences
void loadThresholds() {
  Serial.println("Loading thresholds from preferences...");

  preferences.begin("neuro-config", false);

  betaThreshold = preferences.getFloat("betaTh", 8.0);
  BlinkThreshold = preferences.getFloat("blinkTh", 50.0);
  JAW_ON_THRESHOLD = preferences.getFloat("jawOnTh", 60.0);
  JAW_OFF_THRESHOLD = preferences.getFloat("jawOffTh", 50.0);
  EMG_THRESHOLD_LEFT = preferences.getFloat("emgLeftTh", 150.0);
  EMG_THRESHOLD_RIGHT = preferences.getFloat("emgRightTh", 150.0);

  preferences.end();

  Serial.println("=== LOADED THRESHOLDS ===");
  Serial.printf("  Beta Threshold: %.1f\n", betaThreshold);
  Serial.printf("  Blink Threshold: %.1f\n", BlinkThreshold);
  Serial.printf("  Jaw ON Threshold: %.1f\n", JAW_ON_THRESHOLD);
  Serial.printf("  Jaw OFF Threshold: %.1f\n", JAW_OFF_THRESHOLD);
  Serial.printf("  Left EMG Threshold: %.1f\n", EMG_THRESHOLD_LEFT);
  Serial.printf("  Right EMG Threshold: %.1f\n", EMG_THRESHOLD_RIGHT);
  Serial.println("==========================");
}

// Function to save thresholds to preferences
void saveThresholds() {
  preferences.begin("neuro-config", false);

  preferences.putFloat("betaTh", betaThreshold);
  preferences.putFloat("blinkTh", BlinkThreshold);
  preferences.putFloat("jawOnTh", JAW_ON_THRESHOLD);
  preferences.putFloat("jawOffTh", JAW_OFF_THRESHOLD);
  preferences.putFloat("emgLeftTh", EMG_THRESHOLD_LEFT);
  preferences.putFloat("emgRightTh", EMG_THRESHOLD_RIGHT);

  preferences.end();

  Serial.println("Thresholds saved to flash memory");
}

// ----------------- BANDPOWER & SMOOTHING -----------------
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize) {
  BandpowerResults r = { 0, 0, 0, 0, 0, 0 };
  for (int i = 1; i < halfSize; i++) {
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

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s) {
  s->delta = SMOOTHING_FACTOR * raw->delta + (1 - SMOOTHING_FACTOR) * s->delta;
  s->theta = SMOOTHING_FACTOR * raw->theta + (1 - SMOOTHING_FACTOR) * s->theta;
  s->alpha = SMOOTHING_FACTOR * raw->alpha + (1 - SMOOTHING_FACTOR) * s->alpha;
  s->beta = SMOOTHING_FACTOR * raw->beta + (1 - SMOOTHING_FACTOR) * s->beta;
  s->gamma = SMOOTHING_FACTOR * raw->gamma + (1 - SMOOTHING_FACTOR) * s->gamma;
  s->total = SMOOTHING_FACTOR * raw->total + (1 - SMOOTHING_FACTOR) * s->total;
}

// ----------------- DSP FFT SETUP -----------------
void initFFT() {
  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if (err != ESP_OK) {
    Serial.println("FFT init failed");
    while (1)
      delay(10);
  }
}

// ----------------- PROCESS FFT -----------------
void processFFT() {
  for (int i = 0; i < FFT_SIZE; i++) {
    y_cf[2 * i] = inputBuffer[i];
    y_cf[2 * i + 1] = 0.0f;
  }

  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  int half = FFT_SIZE / 2;
  for (int i = 0; i < half; i++) {
    float re = y1_cf[2 * i];
    float im = y1_cf[2 * i + 1];
    powerSpectrum[i] = re * re + im * im;
  }

  float binRes = float(SAMPLE_RATE) / FFT_SIZE;

  BandpowerResults raw = calculateBandpower(powerSpectrum, binRes, half);
  smoothBandpower(&raw, &smoothedPowers);
  float T = smoothedPowers.total + EPS;

  // Store current beta percentage for web display
  currentBetaPercent = (smoothedPowers.beta / T) * 100;

  if (udpConnected && !emergencyStop && !TakeoffSent && currentBetaPercent > betaThreshold) {
    delay(2000);
    droneToggleFlight();  //takeoff
    TakeoffSent = true;
    Serial.println("take off send");
  }
}

// ----------------- WEB SERVER HANDLERS -----------------
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=yes">
    <title>ESP32-C6 Neuro-Controller | NPG LITE Drone</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        html, body { min-height: 100%; background: #0a0c12; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            display: flex; flex-direction: column; min-height: 100vh; background: #0a0c12;
        }
        .app-wrapper { display: flex; flex-direction: column; min-height: 100vh; background: #0a0c12; }
        .container { flex: 1; display: flex; flex-direction: column; padding: 12px 16px 8px 16px; }
        h1 {
            color: #eef2ff; text-align: center; padding-bottom: 6px; font-size: 1.5rem;
            font-weight: 600; flex-shrink: 0; letter-spacing: -0.2px; text-shadow: 0 1px 2px rgba(0,0,0,0.2);
        }
        .dashboard-split { display: flex; gap: 18px; flex: 1; margin: 8px 0 6px 0; }
        .ch2-ch3-row { display: flex; flex-direction: column; gap: 14px; flex: 1; }
        .ch2-ch3-row .slider-card { flex: 1; min-width: 200px; display: flex; flex-direction: column; }
        .firmware-wrapper { flex: 1; display: flex; flex-direction: column; margin-top: 0; min-height: 220px; }
        .slider-card {
            background: #14171f; border-radius: 20px; padding: 18px 20px 20px 20px;
            box-shadow: 0 8px 20px rgba(0,0,0,0.4), 0 1px 2px rgba(255,255,255,0.05);
            transition: all 0.2s ease; display: flex; flex-direction: column;
            justify-content: space-between; border: 1px solid #2a2e3a;
        }
        .slider-card:hover { box-shadow: 0 12px 28px rgba(0,0,0,0.5); border-color: #3a3f4e; }
        .card-header { display: flex; justify-content: space-between; align-items: baseline; flex-wrap: wrap; gap: 8px; }
        .card-title { font-size: 1.4rem; font-weight: 600; color: #d6e3ff; display: flex; align-items: center; gap: 6px; letter-spacing: -0.2px; }
        .threshold-value { background: #1e2432; padding: 4px 12px; border-radius: 40px; font-size: 0.95rem; font-weight: 600; color: #ff9f6e; white-space: nowrap; border: 1px solid #2d3343; }
        .slider-container { position: relative; margin: 10px 0 90px 0; cursor: pointer; height: 80px; }
        .slider-track-bg { position: absolute; top: 50%; left: 0; right: 0; height: 80px; background: #232838; transform: translateY(-50%); pointer-events: none; z-index: 1; border-radius: 6px; box-shadow: inset 0 1px 3px rgba(0,0,0,0.4); }
        .live-fill { position: absolute; top: 50%; left: 0; height: 80px; background: #4caf50; transform: translateY(-50%); pointer-events: none; z-index: 2; transition: width 0.05s linear; border-radius: 6px 0 0 6px; filter: brightness(1.05); box-shadow: 0 0 4px rgba(76,175,80,0.3); }
        .live-value-label { position: absolute; top: 50%; left: 5%; transform: translate(-50%, -50%); color: #f0f3fc; padding: 2px 8px; border-radius: 30px; font-size: 1.5rem; font-weight: 500; white-space: nowrap; pointer-events: none; z-index: 15; text-shadow: 0 1px 2px black; }
        .threshold-thumb { position: absolute; top: 50%; width: 20px; height: 95px; background: #ff8a5c; transform: translate(-50%, -50%); cursor: grab; z-index: 30; box-shadow: 0 2px 8px rgba(0,0,0,0.5); border: 2px solid #ffcfb0; border-radius: 4px; transition: left 0.05s linear; }
        .threshold-thumb:active { cursor: grabbing; }
        .threshold-value-badge { position: absolute; top: 95px; transform: translateX(-50%); background: #ff8a5c; color: #121212; padding: 2px 8px; border-radius: 10px; font-size: 2rem; font-weight: 700; white-space: nowrap; pointer-events: none; z-index: 25; transition: left 0.05s linear; box-shadow: 0 2px 6px rgba(0,0,0,0.3); border: 1px solid #ffbb95; }
        .hidden-slider { position: absolute; width: 100%; height: 40px; opacity: 0; cursor: pointer; z-index: 35; top: 50%; transform: translateY(-50%); margin: 0; }
        .save-success { background: #1e4620; color: #b9f6ca; padding: 8px 20px; border-radius: 40px; text-align: center; display: none; position: fixed; top: 16px; right: 20px; z-index: 1200; font-size: 0.8rem; font-weight: 500; box-shadow: 0 4px 12px rgba(0,0,0,0.3); border: 1px solid #2e7d32; }
        .emg-card .live-fill { background: #f97316; filter: brightness(0.95); }
        .jaw-card .live-fill { background: #f59e0b; }
        .beta-card .live-fill { background: #8b5cf6; }
        .blink-card .live-fill { background: #10b981; }
        .firmware-dropzone-modern { flex: 1; display: flex; flex-direction: column; align-items: center; justify-content: center; border: 3px dashed #ff9f4a; border-radius: 24px; background: #1b1f2b; transition: all 0.25s ease; cursor: pointer; margin-top: 12px; padding: 24px 20px; min-height: 150px; position: relative; }
        .firmware-dropzone-modern.has-file { border: 3px solid #2ecc71 !important; background: #1a2a1f; box-shadow: 0 4px 14px rgba(46,204,113,0.2); }
        .firmware-dropzone-modern.drag-over { border-color: #2ecc71; background: #1f3825; transform: scale(0.99); }
        .dropzone-text-main { font-size: 1.05rem; font-weight: 600; color: #4ADE80; margin-bottom: 8px; text-align: center; }
        .selected-file-card { width: 100%; background: #0e111b; border-radius: 60px; padding: 10px 16px 10px 20px; display: flex; align-items: center; justify-content: space-between; gap: 12px; box-shadow: 0 4px 14px rgba(0,0,0,0.3); border: 1px solid #2e3440; }
        .file-info { display: flex; align-items: center; gap: 10px; flex: 1; min-width: 0; }
        .file-icon { font-size: 1.5rem; }
        .file-details { flex: 1; min-width: 0; }
        .file-name { font-weight: 600; color: #e2e8ff; font-size: 0.9rem; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
        .file-size { font-size: 0.7rem; color: #8e9aaf; margin-top: 2px; }
        .action-buttons { display: flex; gap: 8px; align-items: center; }
        .update-btn-small { background: linear-gradient(135deg, #2ecc71, #27ae60); border: none; padding: 8px 20px; border-radius: 40px; font-weight: 700; font-size: 0.85rem; color: white; cursor: pointer; transition: all 0.2s; white-space: nowrap; box-shadow: 0 2px 5px rgba(0,0,0,0.3); }
        .update-btn-small:hover { transform: translateY(-2px); box-shadow: 0 6px 14px rgba(46,204,113,0.35); filter: brightness(1.05); }
        .update-btn-small:disabled { opacity: 0.6; transform: none; cursor: not-allowed; }
        .remove-file-btn { background: #2d1f1f; border: none; width: 36px; height: 36px; border-radius: 50%; font-size: 1.1rem; font-weight: bold; color: #ff8a7a; cursor: pointer; display: flex; align-items: center; justify-content: center; transition: all 0.2s ease; border: 1px solid #632c2c; }
        .remove-file-btn:hover { background: #442a2a; transform: scale(1.08); color: #ffaa9a; border-color: #b94b4b; }
        .empty-state-message { width: 100%; text-align: center; margin-top: 8px; font-size: 0.85rem; color: #ffaa66; font-weight: 500; background: rgba(30,30,45,0.6); padding: 10px; border-radius: 40px; }
        .firmware-card-content { flex: 1; display: flex; flex-direction: column; min-height: 0; }
        .footer { flex-shrink: 0; text-align: center; padding: 8px 0; font-size: 1.1rem; font-weight: 600; color: #a5b3cf; display: flex; justify-content: center; gap: 6px; background: #0a0c12; }
        .footer a { color: #ff9f6e; text-decoration: none; font-weight: 600; }
        .footer a:hover { text-decoration: underline; color: #ffb57c; }
        @media (max-height: 700px) { .dashboard-split { flex-direction: column; } .slider-container { margin-bottom: 70px; } }
        @media (max-width: 780px) {
            .dashboard-split { flex-direction: column; gap: 14px; }
            .ch2-ch3-row { flex-direction: column; }
            .container { padding: 10px; }
            .selected-file-card { flex-direction: column; align-items: stretch; border-radius: 20px; padding: 16px; }
            .action-buttons { justify-content: flex-end; margin-top: 8px; }
            .file-info { margin-bottom: 8px; }
            .card-title { font-size: 2rem; }
        }
    </style>
</head>
<body>
<div class="app-wrapper">
  <div class="container">
    <h1>NPG LITE Drone-Controller</h1>
    <div id="saveSuccess" class="save-success">✓ Threshold saved</div>
    <div class="dashboard-split">
      <!-- LEFT PANEL -->
      <div class="ch2-ch3-row">
        <div class="slider-card beta-card">
          <div class="card-header">
            <div class="card-title">Ch1 Beta Waves</div>
            <span class="threshold-value">Threshold: <span id="betaThresholdDisplay">8.0</span> %</span>
          </div>
          <div class="slider-container" id="betaContainer">
            <div class="slider-track-bg"></div>
            <div class="live-fill" id="betaLiveFill"></div>
            <div class="live-value-label" id="betaLiveLabel">0%</div>
            <div class="threshold-thumb" id="betaThresholdThumb"></div>
            <div class="threshold-value-badge" id="betaThresholdBadge">0%</div>
            <input type="range" id="betaSlider" class="hidden-slider" min="0" max="30" step="0.5" value="8">
          </div>
        </div>
        <div class="slider-card blink-card">
          <div class="card-header">
            <div class="card-title">Ch1 Eye Blink</div>
            <span class="threshold-value">Threshold: <span id="blinkThresholdDisplay">50</span></span>
          </div>
          <div class="slider-container" id="blinkContainer">
            <div class="slider-track-bg"></div>
            <div class="live-fill" id="blinkLiveFill"></div>
            <div class="live-value-label" id="blinkLiveLabel">0</div>
            <div class="threshold-thumb" id="blinkThresholdThumb"></div>
            <div class="threshold-value-badge" id="blinkThresholdBadge">0</div>
            <input type="range" id="blinkSlider" class="hidden-slider" min="0" max="200" step="1" value="50">
          </div>
        </div>
        <div class="slider-card jaw-card">
          <div class="card-header">
            <div class="card-title">Ch1 Jaw Muscle</div>
            <span class="threshold-value">Threshold: <span id="jawOnDisplay">60</span></span>
          </div>
          <div class="slider-container" id="jawOnContainer">
            <div class="slider-track-bg"></div>
            <div class="live-fill" id="jawOnLiveFill"></div>
            <div class="live-value-label" id="jawOnLiveLabel">0</div>
            <div class="threshold-thumb" id="jawOnThresholdThumb"></div>
            <div class="threshold-value-badge" id="jawOnThresholdBadge">0</div>
            <input type="range" id="jawOnSlider" class="hidden-slider" min="0" max="150" step="1" value="60">
          </div>
        </div>
      </div>
      <!-- RIGHT PANEL -->
      <div class="ch2-ch3-row">
        <div class="slider-card emg-card">
          <div class="card-header">
            <div class="card-title">Ch2 Right Hand EMG</div>
            <span class="threshold-value">Threshold: <span id="emgRightDisplay">150</span></span>
          </div>
          <div class="slider-container" id="emg2Container">
            <div class="slider-track-bg"></div>
            <div class="live-fill" id="emg2LiveFill"></div>
            <div class="live-value-label" id="emg2LiveLabel">0</div>
            <div class="threshold-thumb" id="emg2ThresholdThumb"></div>
            <div class="threshold-value-badge" id="emg2ThresholdBadge">0</div>
            <input type="range" id="emgRightSlider" class="hidden-slider" min="0" max="300" step="5" value="150">
          </div>
        </div>
        <div class="slider-card emg-card">
          <div class="card-header">
            <div class="card-title">Ch3 Left Hand EMG</div>
            <span class="threshold-value">Threshold: <span id="emgLeftDisplay">150</span></span>
          </div>
          <div class="slider-container" id="emg1Container">
            <div class="slider-track-bg"></div>
            <div class="live-fill" id="emg1LiveFill"></div>
            <div class="live-value-label" id="emg1LiveLabel">0</div>
            <div class="threshold-thumb" id="emg1ThresholdThumb"></div>
            <div class="threshold-value-badge" id="emg1ThresholdBadge">0</div>
            <input type="range" id="emgLeftSlider" class="hidden-slider" min="0" max="300" step="5" value="150">
          </div>
        </div>
        <div class="slider-card firmware-wrapper">
          <div class="card-header">
            <div class="card-title">Firmware Update</div>
          </div>
          <div class="firmware-card-content" style="flex:1; display:flex; flex-direction:column;">
            <form method="POST" action="/update" enctype="multipart/form-data" id="updateForm" style="flex:1; display:flex; flex-direction:column;">
              <div style="flex:1; display:flex; flex-direction:column;">
                <div id="modernDropZone" class="firmware-dropzone-modern">
                  <div class="dropzone-text-main">📂 Drag & drop firmware .bin or click to browse</div>
                  <input type="file" name="firmware" accept=".bin" id="firmwareFileHidden" style="display:none;">
                  <div id="selectedFileArea" style="width:100%; margin-top:8px;"></div>
                </div>
              </div>
            </form>
          </div>
        </div>
      </div>
    </div>
    <div class="footer">Made with ❤️ by <a href="https://github.com/upsidedownlabs" target="_blank" rel="noopener noreferrer">Upside down labs</a></div>
  </div>
</div>
<script>
  let autoSaveTimer = null;
  let isUpdatingFromServer = false;
  const betaSlider = document.getElementById('betaSlider');
  const blinkSlider = document.getElementById('blinkSlider');
  const jawOnSlider = document.getElementById('jawOnSlider');
  const emgLeftSlider = document.getElementById('emgLeftSlider');
  const emgRightSlider = document.getElementById('emgRightSlider');

  function updateThresholdFromSlider(slider, thumbId, badgeId, displayId, suffix = '') {
    const thumb = document.getElementById(thumbId);
    const badge = document.getElementById(badgeId);
    if (!thumb || !badge) return;
    const min = parseFloat(slider.min), max = parseFloat(slider.max), value = parseFloat(slider.value);
    let percent = Math.min(100, Math.max(0, (value - min) / (max - min) * 100));
    thumb.style.left = percent + '%';
    badge.innerHTML = (suffix === '%' ? value.toFixed(1) : Math.round(value)) + suffix;
    const containerWidth = slider.parentElement.offsetWidth;
    const badgeWidth = badge.offsetWidth;
    let leftPx = (percent / 100) * containerWidth;
    leftPx = Math.min(containerWidth - badgeWidth / 2, Math.max(badgeWidth / 2, leftPx));
    badge.style.left = leftPx + 'px';
    if (displayId) document.getElementById(displayId).textContent = suffix === '%' ? value.toFixed(1) : Math.round(value);
  }

  function updateLiveDisplay(fillId, labelId, liveValue, min, max, suffix = '') {
    const fill = document.getElementById(fillId);
    const label = document.getElementById(labelId);
    if (!fill || !label) return;
    let percent = Math.min(100, Math.max(0, (liveValue - min) / (max - min) * 100));
    fill.style.width = percent + '%';
    label.innerHTML = (suffix === '%' ? liveValue.toFixed(1) : Math.round(liveValue)) + suffix;
  }

  function setupSliderDrag(slider, thumbId, badgeId, displayId, suffix = '') {
    slider.addEventListener('input', () => {
      if (!isUpdatingFromServer) { updateThresholdFromSlider(slider, thumbId, badgeId, displayId, suffix); autoSave(); }
    });
    const thumb = document.getElementById(thumbId);
    if (thumb) {
      thumb.addEventListener('mousedown', (e) => {
        e.preventDefault();
        const rect = slider.parentElement.getBoundingClientRect();
        const onMove = (me) => {
          let x = Math.max(0, Math.min(rect.width, me.clientX - rect.left));
          slider.value = Math.min(slider.max, Math.max(slider.min, parseFloat(slider.min) + (parseFloat(slider.max) - parseFloat(slider.min)) * x / rect.width));
          updateThresholdFromSlider(slider, thumbId, badgeId, displayId, suffix);
          autoSave();
        };
        const onUp = () => { document.removeEventListener('mousemove', onMove); document.removeEventListener('mouseup', onUp); };
        document.addEventListener('mousemove', onMove);
        document.addEventListener('mouseup', onUp);
      });
    }
  }

  function saveAllThresholds() {
    const params = new URLSearchParams({ beta: betaSlider.value, blink: blinkSlider.value, jawon: jawOnSlider.value, emgLeft: emgLeftSlider.value, emgRight: emgRightSlider.value });
    fetch('/set?' + params.toString()).then(r => { if (r.ok) { const m = document.getElementById('saveSuccess'); m.style.display = 'block'; setTimeout(() => m.style.display = 'none', 1500); } }).catch(e => console.error(e));
  }

  function autoSave() { if (autoSaveTimer) clearTimeout(autoSaveTimer); autoSaveTimer = setTimeout(saveAllThresholds, 800); }

  function loadLiveData() {
    fetch('/config?_=' + Date.now()).then(r => r.json()).then(data => {
      if (data.beta_percent !== undefined) updateLiveDisplay('betaLiveFill', 'betaLiveLabel', data.beta_percent, 0, 30, '%');
      if (data.blink_live !== undefined) updateLiveDisplay('blinkLiveFill', 'blinkLiveLabel', data.blink_live, 0, 200, '');
      if (data.jaw_live !== undefined) updateLiveDisplay('jawOnLiveFill', 'jawOnLiveLabel', data.jaw_live, 0, 150, '');
      if (data.emg1 !== undefined) updateLiveDisplay('emg1LiveFill', 'emg1LiveLabel', data.emg1, 0, 300, '');
      if (data.emg2 !== undefined) updateLiveDisplay('emg2LiveFill', 'emg2LiveLabel', data.emg2, 0, 300, '');
    }).catch(e => console.error(e));
  }

  function loadThresholds() {
    isUpdatingFromServer = true;
    fetch('/config?_=' + Date.now()).then(r => r.json()).then(data => {
      betaSlider.value = data.beta ?? 8; blinkSlider.value = data.blink ?? 50;
      jawOnSlider.value = data.jawon ?? 60; emgLeftSlider.value = data.emgLeft ?? 150; emgRightSlider.value = data.emgRight ?? 150;
      updateThresholdFromSlider(betaSlider, 'betaThresholdThumb', 'betaThresholdBadge', 'betaThresholdDisplay', '%');
      updateThresholdFromSlider(blinkSlider, 'blinkThresholdThumb', 'blinkThresholdBadge', 'blinkThresholdDisplay', '');
      updateThresholdFromSlider(jawOnSlider, 'jawOnThresholdThumb', 'jawOnThresholdBadge', 'jawOnDisplay', '');
      updateThresholdFromSlider(emgLeftSlider, 'emg1ThresholdThumb', 'emg1ThresholdBadge', 'emgLeftDisplay', '');
      updateThresholdFromSlider(emgRightSlider, 'emg2ThresholdThumb', 'emg2ThresholdBadge', 'emgRightDisplay', '');
      isUpdatingFromServer = false;
    }).catch(e => { console.error(e); isUpdatingFromServer = false; });
  }

  setupSliderDrag(betaSlider, 'betaThresholdThumb', 'betaThresholdBadge', 'betaThresholdDisplay', '%');
  setupSliderDrag(blinkSlider, 'blinkThresholdThumb', 'blinkThresholdBadge', 'blinkThresholdDisplay', '');
  setupSliderDrag(jawOnSlider, 'jawOnThresholdThumb', 'jawOnThresholdBadge', 'jawOnDisplay', '');
  setupSliderDrag(emgLeftSlider, 'emg1ThresholdThumb', 'emg1ThresholdBadge', 'emgLeftDisplay', '');
  setupSliderDrag(emgRightSlider, 'emg2ThresholdThumb', 'emg2ThresholdBadge', 'emgRightDisplay', '');
  window.addEventListener('resize', () => { loadThresholds(); });
  loadThresholds();
  loadLiveData();
  setInterval(loadLiveData, 200);

  // ---- FIRMWARE LOGIC ----
  const dropZoneModern = document.getElementById('modernDropZone');
  const firmwareInput = document.getElementById('firmwareFileHidden');
  const selectedFileArea = document.getElementById('selectedFileArea');
  const updateForm = document.getElementById('updateForm');
  let currentFile = null;

  function formatFileSize(bytes) {
    if (bytes === 0) return '0 Bytes';
    const k = 1024, sizes = ['Bytes', 'KB', 'MB'], i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
  }

  function clearSelectedFile() {
    currentFile = null; firmwareInput.value = ''; refreshFileUI();
    dropZoneModern.classList.remove('has-file');
    dropZoneModern.style.border = "3px dashed #ff9f4a"; dropZoneModern.style.background = "#1b1f2b";
  }

  function refreshFileUI() {
    if (currentFile && currentFile.name.toLowerCase().endsWith('.bin')) {
      dropZoneModern.classList.add('has-file'); dropZoneModern.style.border = ""; dropZoneModern.style.background = "#1a2a1f";
      const fileName = currentFile.name, fileSize = formatFileSize(currentFile.size);
      let displayName = fileName.length > 40 ? fileName.substring(0, 36) + '...' + fileName.slice(-8) : fileName;
      selectedFileArea.innerHTML = `<div class="selected-file-card"><div class="file-info"><div class="file-icon">📄</div><div class="file-details"><div class="file-name" title="${fileName}">${displayName}</div><div class="file-size">${fileSize}</div></div></div><div class="action-buttons"><button type="button" id="removeFileBtn" class="remove-file-btn" title="Remove firmware">✕</button><button type="button" id="dynamicUpdateBtn" class="update-btn-small">UPDATE</button></div></div>`;
      document.getElementById('removeFileBtn').addEventListener('click', (e) => { e.preventDefault(); e.stopPropagation(); clearSelectedFile(); });
      document.getElementById('dynamicUpdateBtn').addEventListener('click', (e) => { e.preventDefault(); e.stopPropagation(); triggerFirmwareUpdate(); });
    } else {
      dropZoneModern.classList.remove('has-file'); dropZoneModern.style.border = "3px dashed #ff9f4a"; dropZoneModern.style.background = "#1b1f2b";
      selectedFileArea.innerHTML = `<div class="empty-state-message"></div>`;
    }
  }

  function setSelectedFile(file) {
    if (file && file.name.toLowerCase().endsWith('.bin')) {
      currentFile = file; const dt = new DataTransfer(); dt.items.add(file); firmwareInput.files = dt.files; refreshFileUI();
    } else { if (file) alert('❌ Only .bin firmware files are allowed'); clearSelectedFile(); }
  }

  async function triggerFirmwareUpdate() {
    if (!currentFile || !firmwareInput.files || firmwareInput.files.length === 0) { alert('📁 Please select a .bin firmware file first'); return; }
    if (!confirm('Update firmware? Device will reboot and reconnect after update.')) return;
    const updateBtn = document.getElementById('dynamicUpdateBtn');
    const originalText = updateBtn ? updateBtn.innerHTML : 'UPDATE';
    if (updateBtn) { updateBtn.innerHTML = '⏳ Uploading...'; updateBtn.disabled = true; }
    const formData = new FormData(updateForm);
    try {
      const response = await fetch('/update', { method: 'POST', body: formData });
      if (response.ok) {
        alert('✅ Firmware uploaded successfully! Device is updating and will reboot.');
        if (updateBtn) { updateBtn.innerHTML = '✓ DONE'; setTimeout(() => { if (updateBtn) { updateBtn.innerHTML = originalText; updateBtn.disabled = false; } }, 3000); }
      } else { alert('❌ Update failed. Check connection or file integrity.'); if (updateBtn) { updateBtn.innerHTML = originalText; updateBtn.disabled = false; } }
    } catch (err) { alert('❌ Network error during update'); if (updateBtn) { updateBtn.innerHTML = originalText; updateBtn.disabled = false; } }
  }

  function preventDefaults(e) { e.preventDefault(); e.stopPropagation(); }
  ['dragenter','dragover','dragleave','drop'].forEach(ev => dropZoneModern.addEventListener(ev, preventDefaults));
  ['dragenter','dragover'].forEach(ev => dropZoneModern.addEventListener(ev, () => dropZoneModern.classList.add('drag-over')));
  ['dragleave','drop'].forEach(ev => dropZoneModern.addEventListener(ev, () => dropZoneModern.classList.remove('drag-over')));
  dropZoneModern.addEventListener('drop', (e) => { if (e.dataTransfer.files.length > 0) setSelectedFile(e.dataTransfer.files[0]); });
  dropZoneModern.addEventListener('click', (e) => { if (e.target && (e.target.id === 'removeFileBtn' || e.target.id === 'dynamicUpdateBtn')) return; firmwareInput.click(); });
  firmwareInput.addEventListener('change', () => { if (firmwareInput.files && firmwareInput.files.length > 0) setSelectedFile(firmwareInput.files[0]); else clearSelectedFile(); });
  updateForm.addEventListener('submit', (e) => { e.preventDefault(); triggerFirmwareUpdate(); });
  clearSelectedFile();
</script>
</body>
</html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleConfig() {
  String json = "{";
  json += "\"beta\":" + String(betaThreshold) + ",";
  json += "\"blink\":" + String(BlinkThreshold) + ",";
  json += "\"jawon\":" + String(JAW_ON_THRESHOLD) + ",";
  json += "\"emgLeft\":" + String(EMG_THRESHOLD_LEFT) + ",";
  json += "\"emgRight\":" + String(EMG_THRESHOLD_RIGHT) + ",";
  json += "\"beta_percent\":" + String(currentBetaPercent) + ",";
  json += "\"blink_live\":" + String(currentEEGEnvelope) + ",";
  json += "\"jaw_live\":" + String(currentJawEnvelope) + ",";
  json += "\"emg1\":" + String(currentEMG1) + ",";
  json += "\"emg2\":" + String(currentEMG2) + ",";
  json += "\"mode\":" + String(is_rotation_mode ? "\"Rotation Mode\"" : "\"Vertical Movement\"") + ",";
  json += "\"udp\":" + String(udpConnected ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleSet() {
  bool changed = false;
  if (server.hasArg("beta")) {
    betaThreshold = server.arg("beta").toFloat();
    changed = true;
  }
  if (server.hasArg("blink")) {
    BlinkThreshold = server.arg("blink").toFloat();
    changed = true;
  }
  if (server.hasArg("jawon")) {
    JAW_ON_THRESHOLD = server.arg("jawon").toFloat();
    JAW_OFF_THRESHOLD = max(0.0f, JAW_ON_THRESHOLD - 10.0f);
    changed = true;
  }
  if (server.hasArg("emgLeft")) {
    EMG_THRESHOLD_LEFT = server.arg("emgLeft").toFloat();
    changed = true;
  }
  if (server.hasArg("emgRight")) {
    EMG_THRESHOLD_RIGHT = server.arg("emgRight").toFloat();
    changed = true;
  }
  if (changed) {
    saveThresholds();
    server.send(200, "text/plain", "OK");
  } else
    server.send(400, "text/plain", "No parameters provided");
}

void handleUpdate() {
  if (server.method() == HTTP_POST) {
    HTTPUpload &upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN))
        Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
        Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        server.send(200, "text/plain", "Update successful! Rebooting...");
        delay(100);
        ESP.restart();
      } else {
        Update.printError(Serial);
        server.send(500, "text/plain", "Update failed");
      }
    }
  }
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/set", handleSet);
  server.on(
    "/update", HTTP_POST, []() {}, handleUpdate);
  server.begin();
  Serial.println("HTTP server started");
}

void startOTAMode() {
  Serial.println("Starting OTA mode...");
  otaMode = true;
  memset(envelopeBuffer, 0, sizeof(envelopeBuffer));
  memset(jawEnvelopeBuffer, 0, sizeof(jawEnvelopeBuffer));
  envelopeSum = jawEnvelopeSum = 0;
  envelopeIndex = jawEnvelopeIndex = 0;
  for (int i = 0; i < 3; i++) {
    notchFilters[i].reset();
    emgFilters[i].reset();
  }
  envelopeFilterLeft.reset();
  envelopeFilterRight.reset();
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  if (MDNS.begin("npglite")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS: http://npglite.local");
  }
  setupWebServer();
  for (int i = 0; i < 6; i++)
    pixel.setPixelColor(i, pixel.Color(128, 0, 128));
  pixel.show();
}

void processSignalsForWeb() {
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  unsigned long now = micros();
  long dt = now - lastMicros;
  lastMicros = now;
  static long timer = 0;
  timer -= dt;
  if (timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;
    int raw1 = analogRead(INPUT_PIN1), raw2 = analogRead(INPUT_PIN2), raw3 = analogRead(INPUT_PIN3);
    float n1 = notchFilters[0].process(raw1), n2 = notchFilters[1].process(raw2), n3 = notchFilters[2].process(raw3);
    currentEEGEnvelope = updateEEGEnvelope(highpass(EEGFilter(n1)));
    currentJawEnvelope = updateJawEnvelope(jawClenchFilter(n1));
    currentEMG2 = envelopeFilterLeft.getEnvelope(abs(emgFilters[1].process(n2)));
    currentEMG1 = envelopeFilterRight.getEnvelope(abs(emgFilters[2].process(n3)));
    inputBuffer[idx++] = EEGFilter(n1);
    if (idx >= FFT_SIZE) {
      processFFT();
      idx = 0;
    }
  }
}

// ===================== DRONE PROTOCOL =====================

// Connection heartbeat: standalone 2-byte 01 01 packet, independent of the
// control stream. Burst of HEARTBEAT_BURST_COUNT at HEARTBEAT_BURST_INTERVAL_MS,
// then settles to a steady HEARTBEAT_INTERVAL_MS cadence for as long as connected.
void sendDroneHeartbeatPacket() {
  uint8_t hb[2] = { 0x01, 0x01 };
  udp.beginPacket(DRONE_IP, DRONE_PORT);
  udp.write(hb, 2);
  udp.endPacket();
}

void updateDroneHeartbeat() {
  unsigned long now = millis();
  if (!heartbeatBurstDone) {
    if (heartbeatBurstSent == 0 || (now - lastHeartbeatTime >= HEARTBEAT_BURST_INTERVAL_MS)) {
      lastHeartbeatTime = now;
      sendDroneHeartbeatPacket();
      heartbeatBurstSent++;
      if (heartbeatBurstSent >= HEARTBEAT_BURST_COUNT)
        heartbeatBurstDone = true;
    }
  } else if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatTime = now;
    sendDroneHeartbeatPacket();
  }
}

// 21-byte control packet: 0x03 0x66 0x14, sticks (or forced-neutral 0x80s
// if an emergency command is embedded), command byte, fixed 0x02, 10 zero
// bytes, checksum, 0x99 trailer. Checksum = XOR of the 6 bytes starting at
// the first stick byte through the fixed 0x02.
void buildDroneControlMessage(uint8_t *out, int &len) {
  int i = 0;
  out[i++] = 0x03;
  out[i++] = 0x66;
  out[i++] = 0x14;

  if (droneCurrentCommand == DRONE_CMD_EMERGENCY) {
    out[i++] = 0x80;
    out[i++] = 0x80;
    out[i++] = 0x80;
    out[i++] = 0x80;
  } else {
    out[i++] = droneRoll;
    out[i++] = dronePitch;
    out[i++] = droneThrottle;
    out[i++] = droneTurn;
  }

  out[i++] = droneCurrentCommand;
  out[i++] = 0x02;

  for (int z = 0; z < 10; z++)
    out[i++] = 0x00;

  uint8_t checksum = out[3] ^ out[4] ^ out[5] ^ out[6] ^ out[7] ^ out[8];
  out[i++] = checksum;
  out[i++] = 0x99;

  len = i;  // 21
}

void sendDroneControlPacket() {
  uint8_t packet[21];
  int len;
  buildDroneControlMessage(packet, len);
  udp.beginPacket(DRONE_IP, DRONE_PORT);
  udp.write(packet, len);
  udp.endPacket();
}

// Call every loop() iteration. Sends the control packet at CONTROL_INTERVAL_MS
// and auto-clears any embedded command after COMMAND_HOLD_MS, mirroring the
// setTimeout in 's _sendCommand()/emergency().
void updateDroneControlLoop() {
  unsigned long now = millis();
  if (now - lastControlSendTime >= CONTROL_INTERVAL_MS) {
    lastControlSendTime = now;
    sendDroneControlPacket();
  }

  if (droneCurrentCommand != DRONE_CMD_NONE && (now - droneCommandStartTime >= COMMAND_HOLD_MS)) {
    droneCurrentCommand = DRONE_CMD_NONE;
  }
}

// One-shot command, same "only one at a time" guard as  _sendCommand.
void droneSendCommand(uint8_t cmd) {
  if (droneCurrentCommand == DRONE_CMD_NONE) {
    droneCurrentCommand = cmd;
    droneCommandStartTime = millis();
  }
}

// Takeoff/land share the same physical command byte -u the drone just
// toggles flight state on it.
void droneToggleFlight() {
  droneSendCommand(DRONE_CMD_TAKE_OFF);
}

// Emergency stop bypasses the "one at a time" guard - preempts whatever is
// currently embedded, and forces sticks neutral via buildDroneControlMessage.
void droneEmergency() {
  droneCurrentCommand = DRONE_CMD_EMERGENCY;
  droneCommandStartTime = millis();
}

bool connectToDrone() {
  Serial.println("Scanning for FLOW-UFO drones...");
  while (true) {
    int n = WiFi.scanNetworks();
    String droneSSID = "";
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i).startsWith(DRONE_SSID_PREFIX)) {
        droneSSID = WiFi.SSID(i);
        break;
      }
    }
    WiFi.scanDelete();
    if (droneSSID.length() == 0) {
      pixel.setPixelColor(5, pixel.Color(255, 0, 0));
      pixel.show();
      delay(3000);
      continue;
    }
    Serial.print("Found drone: ");
    Serial.println(droneSSID);
    WiFi.begin(droneSSID.c_str(), DRONE_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      static bool tog = false;
      tog = !tog;
      pixel.setPixelColor(5, tog ? pixel.Color(255, 80, 0) : pixel.Color(0, 0, 0));
      pixel.show();
      delay(300);
    }
    pixel.setPixelColor(5, pixel.Color(0, 255, 0));
    pixel.show();
    Serial.println("Connected to drone");
    TakeoffSent = false;  // <-- re-arm the one-time trigger for this new connection
    return true;
  }
}
// ===================================================================================

void setPoliceLEDs() {
  unsigned long now = millis();
  static byte flashCount = 0;
  static bool ledState = false;
  if (flashCount < 2) {
    if (now - lastPoliceFlashTime >= 100) {
      lastPoliceFlashTime = now;
      ledState = !ledState;
      if (ledState) {
        for (int i = 0; i < 3; i++)
          pixel.setPixelColor(i, pixel.Color(255, 0, 0));
        for (int i = 3; i < 6; i++)
          pixel.setPixelColor(i, pixel.Color(0, 0, 0));
      } else {
        for (int i = 0; i < 6; i++)
          pixel.setPixelColor(i, pixel.Color(0, 0, 0));
        flashCount++;
        if (flashCount == 2)
          ledState = false;
      }
      pixel.show();
    }
  } else if (flashCount < 4) {
    if (now - lastPoliceFlashTime >= 100) {
      lastPoliceFlashTime = now;
      ledState = !ledState;
      if (ledState) {
        for (int i = 0; i < 3; i++)
          pixel.setPixelColor(i, pixel.Color(0, 0, 0));
        for (int i = 3; i < 6; i++)
          pixel.setPixelColor(i, pixel.Color(0, 0, 255));
      } else {
        for (int i = 0; i < 6; i++)
          pixel.setPixelColor(i, pixel.Color(0, 0, 0));
        flashCount++;
        if (flashCount == 4)
          flashCount = 0;
      }
      pixel.show();
    }
  }
}

void controlEmergencyBuzzer() {
  unsigned long now = millis();
  if (now - lastBuzzerToggleTime >= 300) {
    lastBuzzerToggleTime = now;
    buzzerState = !buzzerState;
    if (buzzerState)
      tone(BUZZER_PIN, 1000, 200);
    else
      noTone(BUZZER_PIN);
  }
}

void checkConnectionStatus() {
  unsigned long now = millis();
  if (now - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL) {
    lastConnectionCheck = now;
    if (WiFi.status() == WL_CONNECTED) {
      if (!udpConnected) {
        udpConnected = true;
        udp.begin(LOCAL_UDP_PORT);
        pixel.setPixelColor(5, pixel.Color(0, 255, 0));
        pixel.show();
      }
    } else {
      if (udpConnected) {
        udpConnected = false;
        pixel.setPixelColor(5, pixel.Color(255, 0, 0));
        pixel.show();
        connectToDrone();
      }
    }
  }
}

void setNormalLEDs() {
  for (int i = 0; i < 6; i++)
    pixel.setPixelColor(i, pixel.Color(0, 0, 0));
  pixel.setPixelColor(5, udpConnected ? pixel.Color(0, 255, 0) : pixel.Color(255, 0, 0));
  pixel.show();
}

void sendContinuousLandCommands() {
  unsigned long now = millis();
  if (!firstCommandSent) {
    droneEmergency();
    lastLandCommandTime = now;
    firstCommandSent = true;
    secondCommandScheduled = true;
  }
  if (secondCommandScheduled && (now - lastLandCommandTime >= 5000)) {
    secondCommandScheduled = false;
  }
}

void handleEmergencyStop() {
  unsigned long now = millis();
  if (digitalRead(BOOT_BUTTON) == LOW && (now - lastButtonPressTime) > BUTTON_DEBOUNCE_MS) {
    lastButtonPressTime = now;
    emergencyStop = !emergencyStop;
    if (emergencyStop) {
      droneEmergency();
      TakeoffSent = false;  // <-- re-arm the one-time trigger for this new connection
      tone(BUZZER_PIN, 1000, 500);
    } else {
      noTone(BUZZER_PIN);
      setNormalLEDs();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pixel.begin();
  pixel.setPixelColor(5, pixel.Color(255, 0, 0));
  pixel.show();
  pinMode(LED_MOTOR_PIN, OUTPUT);

  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);
    int pause = noteDuration * 1.30;
    digitalWrite(LED_MOTOR_PIN, HIGH);
    delay(pause / 2);
    digitalWrite(LED_MOTOR_PIN, LOW);
    delay(pause / 2);
    noTone(BUZZER_PIN);
  }

  delay(500);
  loadThresholds();
  initFFT();

  bootTime = millis();
  while (millis() - bootTime < OTA_TRIGGER_DELAY) {
    if (digitalRead(BOOT_BUTTON) == LOW) {
      startOTAMode();
      return;
    }
    delay(10);
  }

  connectToDrone();
  udp.begin(LOCAL_UDP_PORT);
  udpConnected = true;

  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(INPUT_PIN3, INPUT);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  setupWebServer();
  Serial.println("System ready!");
}

void loop() {
  server.handleClient();

  if (otaMode) {
    processSignalsForWeb();
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      lastBlink = millis();
      static bool state = false;
      state = !state;
      for (int i = 0; i < 6; i++)
        pixel.setPixelColor(i, state ? pixel.Color(128, 0, 128) : pixel.Color(64, 0, 64));
      pixel.show();
    }
    return;
  }

  checkConnectionStatus();
  handleEmergencyStop();

  // Drone heartbeat + control packet cadence run independently of the
  // biopotential sample timer.
  updateDroneHeartbeat();
  updateDroneControlLoop();

  if (emergencyStop) {
    // sendContinuousLandCommands();
    setPoliceLEDs();
    controlEmergencyBuzzer();
    return;
  }

  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  pixel.setPixelColor(5, udpConnected ? pixel.Color(0, 255, 0) : pixel.Color(255, 0, 0));
  pixel.show();

  static long timer = 0;
  timer -= dt;
  if (timer <= 0 && !otaMode) {
    timer += 1000000L / SAMPLE_RATE;

    int raw1 = analogRead(INPUT_PIN1), raw2 = analogRead(INPUT_PIN2), raw3 = analogRead(INPUT_PIN3);
    float n1 = notchFilters[0].process(raw1), n2 = notchFilters[1].process(raw2), n3 = notchFilters[2].process(raw3);

    float filteredEEG = EEGFilter(n1);
    currentEEGEnvelope = updateEEGEnvelope(highpass(filteredEEG));
    currentJawEnvelope = updateJawEnvelope(jawClenchFilter(n1));
    currentEMG2 = envelopeFilterLeft.getEnvelope(abs(emgFilters[1].process(n2)));
    currentEMG1 = envelopeFilterRight.getEnvelope(abs(emgFilters[2].process(n3)));

    inputBuffer[idx++] = filteredEEG;

    unsigned long nowMs = millis();
    bool jawBlockActive = (nowMs - lastJawDetectionTime) < JAW_BLOCK_DURATION_MS;

    // ---- JAW CLENCH: toggle mode ---- (unchanged)
    if (!jawState) {
      if (currentJawEnvelope > JAW_ON_THRESHOLD && (nowMs - lastJawClenchTime) >= JAW_DEBOUNCE_MS) {
        jawState = true;
        lastJawClenchTime = nowMs;
        lastJawDetectionTime = nowMs;
        is_rotation_mode = !is_rotation_mode;
        pixel.setPixelColor(0, is_rotation_mode ? pixel.Color(255, 255, 0) : pixel.Color(255, 0, 0));
        pixel.show();
      }
    } else {
      if (currentJawEnvelope < JAW_OFF_THRESHOLD)
        jawState = false;
    }

    // ---- BLINK DETECTION ---- (detection logic unchanged; this drone has
    // no flip command, so a confirmed triple blink now toggles takeoff/land
    // instead, giving a manual override alongside the beta-triggered one)
    if (!jawBlockActive) {
      if (currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
        lastBlinkTime = nowMs;
        if (blinkCount == 0) {
          firstBlinkTime = nowMs;
          blinkCount = 1;
        } else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
          blinkCount = 2;
        } else if (blinkCount == 2 && (nowMs - firstBlinkTime) <= TRIPLE_BLINK_MS && TakeoffSent) {
          droneToggleFlight();  // triple blink = emergency land
          TakeoffSent = false;  // <-- re-arm the one-time trigger for this new connection
          Serial.println("Triple blink: takeoff/land toggled.");
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
    }

    droneRoll = STICK_NEUTRAL;
    dronePitch = STICK_NEUTRAL;
    droneThrottle = STICK_NEUTRAL;
    droneTurn = STICK_NEUTRAL;

    if (!jawBlockActive) {
      bool emg1Active = currentEMG1 > EMG_THRESHOLD_LEFT;
      bool emg2Active = currentEMG2 > EMG_THRESHOLD_RIGHT;

      if (is_rotation_mode) {
        if (emg1Active)
          dronePitch = STICK_PITCH_FORWARD;  // forward
        else if (emg2Active)
          droneTurn = STICK_TURN;  // rotate
      } else {
        if (emg1Active)
          droneThrottle = STICK_THROTTLE_UP;  // up
        else if (emg2Active)
          droneThrottle = STICK_THROTTLE_DOWN;  // down
      }
    }

    if (idx >= FFT_SIZE && !otaMode) {
      processFFT();
      idx = 0;
    }
  }
}
