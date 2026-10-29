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

// Copyright (c) 2026 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2026 Aman Maheshwari - aman@upsidedownlabs.tech
// Copyright (c) 2026 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

// =====================================================================
//  NPG Lite  —  FOCUS PATAKHA (controller + web UI)
//
//  * Reads EEG on A0 (Ch1), computes beta-band % (Hann window + DC removal,
//    updated 4x/sec)
//  * Creates Wi-Fi hotspot "NPG-Focus-Light" (pass 12345678)
//  * Laptop: join hotspot, open http://focus-light.local  (or http://192.168.4.1)
//  * Sends ON/OFF to the XIAO ESP32-C6 over UDP; XIAO replies with its real
//    LED state so the UI shows what is actually happening
//
//  WHY IT NO LONGER LAGS
//  The work is split into 3 FreeRTOS tasks with different priorities:
//    1. sampleTask (highest)  - woken by a hardware timer exactly 512x/sec,
//                               reads the ADC + filters. Nothing can delay it.
//    2. ctrlTask   (medium)   - FFT -> focus %, threshold decision, UDP to the
//                               XIAO every 100 ms (instantly on change),
//                               NeoPixels, saving the threshold.
//    3. loop()     (lowest)   - only the web server for the laptop UI.
//  So a slow laptop / browser can only slow down the web page, never the
//  EEG sampling or the XIAO communication.
//
//  Libraries: Adafruit NeoPixel, OneButton (Matthias Hertel), ESP-DSP (built into core 3.x)
//  Board:     ESP32C6 Dev Module (NPG Lite), arduino-esp32 core 3.x
// =====================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <OneButton.h>                   // Library Manager: "OneButton" by Matthias Hertel
#include "esp_dsp.h"

// ---------------- NETWORK ----------------
const char *AP_SSID = "NPG-Focus-Light";
const char *AP_PASS = "12345678";
const char *MDNS_NAME = "focus-light";      // -> http://focus-light.local
const uint16_t XIAO_PORT = 4210;            // XIAO listens here
const uint16_t NPG_PORT  = 4211;            // NPG listens for XIAO replies
const IPAddress BROADCAST_IP(192, 168, 4, 255);

// ---------------- SIGNAL ----------------
#define SAMPLE_RATE 512
#define FFT_SIZE    512                     // 1 s window
#define HOP_SIZE    128                     // new focus value every 0.25 s
#define INPUT_PIN   A0
#define BETA_LOW    13.0f
#define BETA_HIGH   30.0f
#define SMOOTHING   0.25f                   // lower = smoother/slower
#define EPS         1e-7f

// ---------------- LED / CRACKER LOGIC ----------------
#define HYSTERESIS        1.0f              // turns off at (threshold - 1%) to avoid flicker
#define SEND_INTERVAL_MS  100
#define XIAO_TIMEOUT_MS   1500
#define SAVE_DELAY_MS     1500              // write threshold to flash once slider stops

// ---------------- NPG HARDWARE ----------------
#define PIXEL_PIN   15
#define NUM_PIXELS  6
#define BUZZER_PIN  8
#define BOOT_BUTTON 9
#define MOTOR_PIN   7             // NPG Lite vibration motor

// ---------------- XIAO LINK FEEDBACK ----------------
//  connected    -> all NeoPixels flash GREEN once + one vibration
//  disconnected -> all NeoPixels flash RED + two vibrations, then the first
//                  pixel stays red until the XIAO is back
#define LINK_FLASH_MS   700

// ---------------- BOOT BUTTON (extra, for testing) ----------------
//  single click  -> threshold down one step
//  double click  -> threshold up one step
//  hold          -> TEST: fake focus rises until it crosses the threshold
//                   and fires; release -> back to the real EEG
#define TH_MIN          5.0f
#define TH_MAX          30.0f
#define TH_WEB_MAX      40.0f     // dashboard slider goes a bit higher than the button
#define TH_STEP         2.5f      // 1 NeoPixel = 5 %, a green pixel = +2.5 %
#define TH_SHOW_MS      2000      // threshold stays on the NeoPixels this long after a click
#define TEST_RAMP       5.0f      // fake focus rises 5 % per second while held
#define TEST_MAX        35.0f

// ---------------- BUZZER SOUNDS ----------------
//  LED bar up   -> one note per NeoPixel that turns on (Do Re Mi Fa Sol La)
//  LED bar down -> short low tick per NeoPixel that turns off
//  fire         -> cracker blast: deep boom, noise burst, crackle
#define SOUND_ON        1         // set 0 to silence the LED notes + blast
#define BEEP_ON_DOWN    1         // set 0 for no tick when an LED turns off
#define BEEP_GAP_MS     120       // shortest gap between two notes
#define FIRE_BRIGHTNESS 180       // all 6 NeoPixels white while firing (255 = max, ~360 mA)
#define NORMAL_BRIGHTNESS 40

enum LedMode { MODE_AUTO = 0, MODE_FORCE_ON = 1, MODE_FORCE_OFF = 2 };

WebServer server(80);
WiFiUDP udp;                                // used ONLY inside ctrlTask
Preferences prefs;
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- SHARED STATE (written by one task, read by others) ----------------
volatile float focusThreshold = 8.0f;
volatile float focusPercent = 0.0f;         // what the rest of the code uses
volatile float realFocus = 0.0f;            // from the EEG
volatile bool  testHold = false;            // boot button held -> fake focus
float simFocus = 0.0f;
unsigned long thShowUntil = 0;
unsigned long linkFlashUntil = 0;           // XIAO link flash (green / red)
uint32_t linkFlashColor = 0;
OneButton button(BOOT_BUTTON, true, true);  // active LOW, internal pull-up
volatile bool  ledCommand = false;
volatile int   ledMode = MODE_AUTO;
volatile bool  forceSend = false;
volatile unsigned long saveAt = 0;

volatile bool  xiaoKnown = false;
volatile unsigned long xiaoLastSeen = 0;
volatile bool  xiaoLedState = false;
IPAddress xiaoIP;

volatile uint32_t sampleCount = 0;          // diagnostics
volatile uint32_t replyCount = 0;
volatile int sampleHz = 0, xiaoPps = 0;

// ---------------- TASKS ----------------
TaskHandle_t sampleTaskHandle = nullptr;
TaskHandle_t ctrlTaskHandle = nullptr;
hw_timer_t *sampleTimer = nullptr;

// ---------------- SIGNAL BUFFERS ----------------
float ring[FFT_SIZE];
int ringIdx = 0, ringFilled = 0, hopCount = 0;
float snap[FFT_SIZE];                       // ordered copy for the FFT
volatile bool snapReady = false;
float hannWin[FFT_SIZE];
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float sBeta = 0, sTotal = 0;

// ---------------- FILTERS (from your reference firmware) ----------------
class NotchFilter
{
  struct BiquadState { float z1 = 0, z2 = 0; };
  BiquadState s1, s2;
public:
  float process(float input)
  {
    float output = input;
    float x = output - (-1.56858163f * s1.z1) - (0.96424138f * s1.z2);
    output = 0.96508099f * x + (-1.56202714f * s1.z1) + (0.96508099f * s1.z2);
    s1.z2 = s1.z1; s1.z1 = x;
    x = output - (-1.61100358f * s2.z1) - (0.96592171f * s2.z2);
    output = 1.00000000f * x + (-1.61854514f * s2.z1) + (1.00000000f * s2.z2);
    s2.z2 = s2.z1; s2.z1 = x;
    return output;
  }
};
NotchFilter notch;

float EEGFilter(float input)
{
  static float z1 = 0, z2 = 0;
  float x = input - -1.22465158f * z1 - 0.45044543f * z2;
  float output = 0.05644846f * x + 0.11289692f * z1 + 0.05644846f * z2;
  z2 = z1; z1 = x;
  return output;
}

bool xiaoOnline() { return xiaoKnown && (millis() - xiaoLastSeen < XIAO_TIMEOUT_MS); }

// =====================================================================
//  1) SAMPLING  (hardware timer -> highest priority task)
// =====================================================================
void ARDUINO_ISR_ATTR onSampleTimer()
{
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(sampleTaskHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

void sampleOnce()
{
  int raw = analogRead(INPUT_PIN);
  ring[ringIdx] = EEGFilter(notch.process(raw));
  ringIdx = (ringIdx + 1) % FFT_SIZE;
  if (ringFilled < FFT_SIZE) ringFilled++;
  sampleCount++;

  if (++hopCount >= HOP_SIZE)
  {
    hopCount = 0;
    if (ringFilled >= FFT_SIZE && !snapReady)   // ctrlTask finished the previous one
    {
      for (int i = 0; i < FFT_SIZE; i++) snap[i] = ring[(ringIdx + i) % FFT_SIZE];  // oldest -> newest
      snapReady = true;
      xTaskNotifyGive(ctrlTaskHandle);
    }
  }
}

void sampleTask(void *)
{
  for (;;)
  {
    uint32_t due = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // number of timer ticks pending
    if (due > 16) due = 16;                                   // never build a backlog
    while (due--) sampleOnce();
  }
}

// =====================================================================
//  2) CONTROL  (FFT, decision, XIAO UDP, NeoPixels)
// =====================================================================
void computeFocus()
{
  float mean = 0;
  for (int i = 0; i < FFT_SIZE; i++) mean += snap[i];
  mean /= FFT_SIZE;

  for (int i = 0; i < FFT_SIZE; i++)
  {
    y_cf[2 * i] = (snap[i] - mean) * hannWin[i];
    y_cf[2 * i + 1] = 0.0f;
  }
  dsps_fft2r_fc32(y_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y_cf, FFT_SIZE);

  const float binRes = (float)SAMPLE_RATE / FFT_SIZE;
  float beta = 0, total = 0;
  for (int i = 1; i < FFT_SIZE / 2; i++)
  {
    float re = y_cf[2 * i], im = y_cf[2 * i + 1];
    float p = re * re + im * im;
    float f = i * binRes;
    total += p;
    if (f >= BETA_LOW && f < BETA_HIGH) beta += p;
  }
  sBeta  = SMOOTHING * beta  + (1 - SMOOTHING) * sBeta;
  sTotal = SMOOTHING * total + (1 - SMOOTHING) * sTotal;
  realFocus = sBeta / (sTotal + EPS) * 100.0f;
}

bool updateLedDecision()
{
  bool prev = ledCommand;
  float f = focusPercent, th = focusThreshold;
  bool next;
  if (ledMode == MODE_FORCE_ON)       next = true;
  else if (ledMode == MODE_FORCE_OFF) next = false;
  else next = prev ? (f > th - HYSTERESIS) : (f > th);
  ledCommand = next;
  if (next != prev)
    Serial.printf("CRACKER %s  (focus %.1f%%, threshold %.1f%%)\n", next ? "ON" : "OFF", f, th);
  return next != prev;
}

void sendToXiao()
{
  udp.beginPacket(xiaoOnline() ? xiaoIP : BROADCAST_IP, XIAO_PORT);
  udp.printf("F,%.2f,%d", (float)focusPercent, ledCommand ? 1 : 0);
  udp.endPacket();
}

void readXiaoReplies()
{
  while (udp.parsePacket() > 0)
  {
    char buf[32];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = 0;
    if (buf[0] == 'A' && buf[1] == ',')
    {
      if (!xiaoOnline()) Serial.printf("XIAO connected: %s\n", udp.remoteIP().toString().c_str());
      xiaoIP = udp.remoteIP();
      xiaoKnown = true;
      xiaoLastSeen = millis();
      xiaoLedState = (buf[2] == '1');
      replyCount++;
    }
  }
}

// Threshold on the 6 NeoPixels: each BLUE pixel = 5 %, a GREEN pixel = +2.5 %
//   5 -> [B]   7.5 -> [B][G]   10 -> [B][B]   ...   30 -> [B][B][B][B][B][B]
void showThresholdPixels()
{
  float th = focusThreshold;
  int full = (int)(th / 5.0f + 0.001f);
  bool half = (th - full * 5.0f) >= 2.49f;
  for (int i = 0; i < NUM_PIXELS; i++)
  {
    uint32_t c = 0;
    if (i < full) c = pixels.Color(0, 0, 255);
    else if (i == full && half) c = pixels.Color(0, 255, 0);
    pixels.setPixelColor(i, c);
  }
  pixels.show();
}

// Hold-test bar on the same 5 %-per-pixel scale. Pixels below the threshold are
// purple; once it fires, every lit pixel past the threshold turns WHITE.
void showTestPixels()
{
  float f = focusPercent, th = focusThreshold;
  int lit = constrain((int)(f / 5.0f + 0.5f), 0, NUM_PIXELS);
  for (int i = 0; i < NUM_PIXELS; i++)
  {
    uint32_t c = 0;
    if (i < lit)
      c = (ledCommand && (i + 1) * 5.0f > th) ? pixels.Color(255, 255, 255) : pixels.Color(110, 0, 200);
    pixels.setPixelColor(i, c);
  }
  pixels.show();
}

void updatePixels()
{
  if (ledCommand)                              // firing: every pixel bright white
  {
    pixels.setBrightness(FIRE_BRIGHTNESS);
    for (int i = 0; i < NUM_PIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    pixels.show();
    return;
  }
  pixels.setBrightness(NORMAL_BRIGHTNESS);
  if ((long)(millis() - linkFlashUntil) < 0)    // XIAO just connected (green) or dropped (red)
  {
    for (int i = 0; i < NUM_PIXELS; i++) pixels.setPixelColor(i, linkFlashColor);
    pixels.show();
    return;
  }
  if ((long)(millis() - thShowUntil) < 0) { showThresholdPixels(); return; }
  if (testHold) { showTestPixels(); return; }
  // normal: threshold = 3 pixels lit, 2x threshold = all 6
  int lit = 0;
  float th = focusThreshold;
  if (th > 0)
    lit = constrain((int)roundf(focusPercent / (2.0f * th) * NUM_PIXELS), 0, NUM_PIXELS);
  uint32_t c = ledCommand ? pixels.Color(255, 140, 0) : pixels.Color(110, 0, 200);
  for (int i = 0; i < NUM_PIXELS; i++) pixels.setPixelColor(i, i < lit ? c : 0);
  if (!xiaoOnline()) pixels.setPixelColor(0, pixels.Color(255, 0, 0));   // no XIAO: first pixel red
  pixels.show();
}

// ---------------- XIAO LINK: green / red flash + vibration ----------------
uint16_t motorPattern[4];                    // on, off, on, off (ms)
int motorLen = 0, motorPos = 0;
unsigned long motorNext = 0;

void startMotor(const uint16_t *pat, int len)
{
  for (int i = 0; i < len; i++) motorPattern[i] = pat[i];
  motorLen = len; motorPos = 0; motorNext = millis();
}

void motorTick(unsigned long now)            // non-blocking vibration pattern
{
  if (motorPos >= motorLen || (long)(now - motorNext) < 0) return;
  digitalWrite(MOTOR_PIN, (motorPos % 2 == 0) ? HIGH : LOW);   // even steps = on
  motorNext = now + motorPattern[motorPos++];
  if (motorPos >= motorLen) { motorNext = now; }
}

void linkTick(unsigned long now)
{
  static bool wasOnline = false;
  bool online = xiaoOnline();
  if (online == wasOnline)
  {
    if (motorPos >= motorLen && (long)(now - motorNext) >= 0) digitalWrite(MOTOR_PIN, LOW);
    return;
  }
  wasOnline = online;
  linkFlashUntil = now + LINK_FLASH_MS;
  if (online)
  {
    static const uint16_t buzzOnce[] = {300, 0};
    linkFlashColor = pixels.Color(0, 255, 0);
    startMotor(buzzOnce, 2);
    Serial.println("XIAO link: CONNECTED");
  }
  else
  {
    static const uint16_t buzzTwice[] = {150, 120, 150, 0};
    linkFlashColor = pixels.Color(255, 0, 0);
    startMotor(buzzTwice, 4);
    Serial.println("XIAO link: LOST");
  }
  updatePixels();
}

// ---------------- BUZZER: non-blocking sound player ----------------
struct Note { uint16_t f, on, gap; };       // f = 0 -> silence
Note seq[48];
int seqLen = 0, seqPos = 0;
unsigned long seqNext = 0;
int lastLeds = 0;                            // LED count the buzzer last played
unsigned long lastBeep = 0;
const uint16_t LED_NOTES[NUM_PIXELS] = {523, 587, 659, 698, 784, 880};   // C5 D5 E5 F5 G5 A5

// How many NeoPixels the focus bar is showing right now (same maths as the LEDs)
int focusLeds()
{
  float f = focusPercent, th = focusThreshold;
  if (testHold) return constrain((int)(f / 5.0f + 0.5f), 0, NUM_PIXELS);          // test bar: 5 % per LED
  if (th <= 0) return 0;
  return constrain((int)roundf(f / (2.0f * th) * NUM_PIXELS), 0, NUM_PIXELS);     // normal bar: 3 LEDs = threshold
}

void buildFireSound()                        // cracker blast: boom -> noise burst -> crackle (~2 s)
{
  int n = 0;
  seq[n++] = {90, 60, 0};                                                       // deep boom
  seq[n++] = {70, 90, 0};
  seq[n++] = {110, 40, 0};
  seq[n++] = {60, 140, 30};
  for (int k = 0; k < 10; k++)                                                  // rough noise burst
    seq[n++] = {(uint16_t)random(150, 900), 12, 0};
  seq[n++] = {0, 0, 80};
  for (int k = 0; k < 18; k++)                                                  // crackle
    seq[n++] = {(uint16_t)random(1500, 4500), (uint16_t)random(6, 15), (uint16_t)random(15, 90)};
  seqLen = n; seqPos = 0; seqNext = millis();
}

void soundTick(unsigned long now)
{
#if SOUND_ON
  if (seqPos < seqLen)                       // a firework sound is playing
  {
    if ((long)(now - seqNext) >= 0)
    {
      Note q = seq[seqPos++];
      if (q.f) tone(BUZZER_PIN, q.f, q.on);
      seqNext = now + q.on + q.gap;
    }
    return;
  }
  int leds = focusLeds();
  static bool primed = false;
  if (!primed) { if (focusPercent > 0) { lastLeds = leds; primed = true; } return; }   // silent at power-on
  if (ledCommand) { lastLeds = leds; return; }                         // firing: the blast plays instead
  if (now - lastBeep < BEEP_GAP_MS) return;
  if (leds > lastLeds)                                                 // one more LED on -> its note
  {
    lastLeds++;                                                        // a jump of 2 LEDs plays 2 notes
    tone(BUZZER_PIN, LED_NOTES[lastLeds - 1], 90);
    lastBeep = now;
  }
  else if (leds < lastLeds)                                            // one LED off -> low tick
  {
    lastLeds--;
#if BEEP_ON_DOWN
    tone(BUZZER_PIN, 220, 30);
    lastBeep = now;
#endif
  }
#endif
}

void setThresholdFromButton(float th)
{
  th = constrain(th, TH_MIN, TH_MAX);
  bool moved = fabsf(th - focusThreshold) > 0.01f;
  focusThreshold = th;
  saveAt = millis() + SAVE_DELAY_MS;          // saved to flash like the web slider
  forceSend = true;
  thShowUntil = millis() + TH_SHOW_MS;
  showThresholdPixels();
  if (moved) tone(BUZZER_PIN, 1500, 40);
  else       tone(BUZZER_PIN, 400, 120);      // already at the limit
  Serial.printf("Button: threshold %.1f%%\n", th);
}

void onSingleClick()   // step down to the next 2.5 % mark
{
  float th = focusThreshold;
  setThresholdFromButton(ceilf(th / TH_STEP - 0.001f) * TH_STEP - TH_STEP);
}

void onDoubleClick()   // step up to the next 2.5 % mark
{
  float th = focusThreshold;
  setThresholdFromButton(floorf(th / TH_STEP + 0.001f) * TH_STEP + TH_STEP);
}

void onHoldStart()
{
  // start a little below the threshold so you can watch it climb and cross
  simFocus = min((float)realFocus, (float)focusThreshold - 5.0f);
  if (simFocus < 0) simFocus = 0;
  testHold = true;
  thShowUntil = 0;
  Serial.println("Button held: TEST mode, fake focus rising");
}

void onHoldStop()
{
  testHold = false;
  Serial.println("Button released: back to real EEG");
}

void ctrlTask(void *)
{
  unsigned long lastTx = 0, lastPix = 0, lastStat = 0;
  uint32_t prevSamples = 0, prevReplies = 0;
  unsigned long lastTick = millis();
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));   // wake on new FFT window or every 10 ms

    button.tick();                                  // boot button, every ~10 ms
    unsigned long nowT = millis();
    if (testHold) simFocus = min(simFocus + TEST_RAMP * (nowT - lastTick) / 1000.0f, TEST_MAX);
    lastTick = nowT;

    if (snapReady)
    {
      computeFocus();
      snapReady = false;
    }

    focusPercent = testHold ? simFocus : (float)realFocus;

    readXiaoReplies();
    bool changed = updateLedDecision();
#if SOUND_ON
    if (changed && ledCommand) buildFireSound();     // cracker just fired
#endif
    if (changed) updatePixels();                     // white the moment it fires
    soundTick(millis());
    linkTick(millis());
    motorTick(millis());

    unsigned long now = millis();
    if (changed || forceSend || now - lastTx >= SEND_INTERVAL_MS)
    {
      forceSend = false;
      lastTx = now;
      sendToXiao();
    }
    if (now - lastPix >= 100)
    {
      lastPix = now;
      updatePixels();
    }
    if (now - lastStat >= 1000)
    {
      uint32_t s = sampleCount, r = replyCount;
      sampleHz = (int)((s - prevSamples) * 1000UL / (now - lastStat));
      xiaoPps  = (int)((r - prevReplies) * 1000UL / (now - lastStat));
      prevSamples = s; prevReplies = r; lastStat = now;
    }
    if (saveAt && (long)(now - saveAt) >= 0)
    {
      saveAt = 0;
      prefs.begin("focus-light", false);
      prefs.putFloat("th", focusThreshold);
      prefs.end();
      Serial.printf("Threshold saved: %.1f%%\n", (float)focusThreshold);
    }
  }
}

// =====================================================================
//  3) WEB UI  (lowest priority, in loop())
// =====================================================================
#include "index_html.h"     // the web page (HTML/CSS/JS) lives in this file

void cors() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void handleData()
{
  char json[256];
  snprintf(json, sizeof(json),
           "{\"focus\":%.2f,\"th\":%.1f,\"led\":%s,\"mode\":%d,\"xiao\":%s,\"xiaoLed\":%s,"
           "\"hz\":%d,\"pps\":%d,\"ip\":\"%s\",\"test\":%s}",
           (float)focusPercent, (float)focusThreshold, ledCommand ? "true" : "false", (int)ledMode,
           xiaoOnline() ? "true" : "false", xiaoLedState ? "true" : "false",
           (int)sampleHz, (int)xiaoPps, WiFi.softAPIP().toString().c_str(), testHold ? "true" : "false");
  cors();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleSet()
{
  cors();
  if (!server.hasArg("th")) { server.send(400, "text/plain", "missing th"); return; }
  float th = server.arg("th").toFloat();
  if (!isfinite(th)) { server.send(400, "text/plain", "bad th"); return; }
  focusThreshold = constrain(th, TH_MIN, TH_WEB_MAX);   // th near 0 would fire on any noise
  saveAt = millis() + SAVE_DELAY_MS;          // ctrlTask writes flash later, not here
  forceSend = true;
  server.send(200, "text/plain", "OK");
}

void handleMode()
{
  cors();
  if (!server.hasArg("m")) { server.send(400, "text/plain", "missing m"); return; }
  ledMode = constrain(server.arg("m").toInt(), 0, 2);
  forceSend = true;
  server.send(200, "text/plain", "OK");
}

// ---------------- SETUP / LOOP ----------------
void setup()
{
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(NORMAL_BRIGHTNESS);
  pixels.clear();
  pixels.show();
  pinMode(INPUT_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  prefs.begin("focus-light", true);
  float savedTh = prefs.getFloat("th", 8.0f);
  prefs.end();
  focusThreshold = isfinite(savedTh) ? constrain(savedTh, TH_MIN, TH_WEB_MAX) : 8.0f;

  for (int i = 0; i < FFT_SIZE; i++)
    hannWin[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));

  if (dsps_fft2r_init_fc32(NULL, FFT_SIZE) != ESP_OK)
  {
    Serial.println("FFT init failed");
    while (1) delay(10);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("Hotspot \"%s\" up. Open http://%s.local  (or http://%s)\n",
                AP_SSID, MDNS_NAME, WiFi.softAPIP().toString().c_str());

  udp.begin(NPG_PORT);
  if (MDNS.begin(MDNS_NAME)) MDNS.addService("http", "tcp", 80);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/set", handleSet);
  server.on("/mode", handleMode);
  server.begin();

  // boot button (ticked inside ctrlTask)
  button.attachClick(onSingleClick);
  button.attachDoubleClick(onDoubleClick);
  button.attachLongPressStart(onHoldStart);
  button.attachLongPressStop(onHoldStop);
  button.setClickMs(300);      // wait this long for a second click
  button.setPressMs(700);      // hold this long to start the test

  // tasks: sampling > control > web (loop runs at priority 1)
  xTaskCreate(ctrlTask,   "ctrl",   6144, nullptr, 5,  &ctrlTaskHandle);
  xTaskCreate(sampleTask, "sample", 4096, nullptr, 20, &sampleTaskHandle);

  // 1 MHz timer, alarm every 1953 us = 512.03 Hz
  sampleTimer = timerBegin(1000000);
  timerAttachInterrupt(sampleTimer, &onSampleTimer);
  timerAlarm(sampleTimer, 1000000 / SAMPLE_RATE, true, 0);

  tone(BUZZER_PIN, 1000, 120);
  Serial.printf("Ready. Threshold %.1f%%\n", (float)focusThreshold);
}

void loop()
{
  server.handleClient();     // only the web page lives here
  delay(1);
}
