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

// ESP32 based IR Controller using an Adafruit IR Transceiver on two GPIO lines.
// Commands are grouped into profiles, one per appliance. Hold the user button to
// record a signal into the open profile, short press to fire the highlighted one.
//
// Profile and command names live in NVS, captured waveforms in LittleFS. What is
// highlighted is a selection rather than a setting, so it stays in RAM and is
// never written. Thresholds and the control mapping are runtime values seeded
// from the defaults below, and the app pushes its own over Bluetooth.

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <Adafruit_NeoPixel.h>

#define BIOAMP_ENABLED     true  // false = plain IR remote, no bio-potential data sampling at all

#define SAMPLE_RATE        500   // per channel, must match the filter design
#define MAX_BIOAMP_CHANNELS   6     // A0 to A5, how many are wired depends on the playmate

// Which channels sample on boot, and with what. The app changes this and the
// board re-inits DMA for exactly what is selected, so this is only the
// starting point: channel 0 on the full EEG chain, everything else off.
#define DEFAULT_FILTER_CH0 FILT_EEG
#define DEFAULT_NOTCH_HZ   50

// Starting points only. Every threshold is a per channel runtime value the app
// retunes against the live bars, so none of them are compiled in.
#define DEFAULT_MUSCLE_THRESHOLD 75    // envelope level that starts a clench
#define DEFAULT_MUSCLE_RELEASE   60    // must fall below this before the next one counts
#define DEFAULT_FOCUS_THRESHOLD  10.0f // beta share of total EEG power, percent
#define DEFAULT_BLINK_THRESHOLD  50.0f // envelope level that counts as one blink

#define CLENCH_HOLD_MS     600   // a clench held this long starts repeating
#define CLENCH_REPEAT_MS   300   // repeat cadence while held, matches a fast tap
#define CLENCH_BLOCK_MS    600   // a clench swamps the EEG, so ignore focus around it

#define BLINK_DEBOUNCE_MS  250   // minimum spacing between two counted blinks
#define BLINK_GAP_MS       600   // quiet for this long and the blink burst is over
#define BLINK_BLOCK_MS     500   // a blink swamps the EEG, so ignore focus around it
#define BLINK_RELEASE      0.7f  // envelope must fall to this share before rearming

#define FOCUS_DEBOUNCE_MS  2000  // ignore further focus triggers for this long

#define TRIGGER_DEBOUNCE   300   // ignore new triggers for this long after one lands
#define TRIGGER_SETTLE     300   // ignore triggers after a reset, filters are ringing
#define STALL_GAP_MS       25    // a service gap longer than this means we were blocked

#define STREAM_MS          50    // live level updates, 20 per second
#define STREAM_HOLDOFF_MS  60    // stay off the air this long after an IR send

#define SAMPLE_BLOCK_COUNT 30    // samples per channel per DMA frame
#define BATTERY_PIN        6     // battery divider sits on ADC1 channel 6 (A6)

// What a channel is filtered for. A channel set to FILT_OFF is not sampled at
// all, so this doubles as the channel selection.
#define FILT_OFF  0
#define FILT_EMG  1   // EMG only, one muscle level
#define FILT_EEG  2   // the lot: muscle, focus and blink off one pair of electrodes
#define FILT_EOG  3   // EEG into the EOG high pass, blinks only

// What a channel can produce. Which of these are live depends on its filter.
#define GEST_NONE          0
#define GEST_CLENCH        1   // shown as EMG on an EMG channel, Jaw Clench on an EEG one
#define GEST_CLENCH_HOLD   2
#define GEST_FOCUS         3
#define GEST_DOUBLE_BLINK  4
#define GEST_TRIPLE_BLINK  5
#define GESTURE_COUNT      6

// Playmate variants, detected on boot. Only the channel count matters here.
#define PLAYMATE_PROTO      0   // 3 BioAmp channels, no buzzer or motor
#define PLAYMATE_VIBZ       1   // 3 BioAmp channels, buzzer and motor
#define PLAYMATE_VIBZ_PLUS  2   // 6 BioAmp channels

#if BIOAMP_ENABLED
  #include "freertos/FreeRTOS.h"
  #include "freertos/semphr.h"
  #include "esp_adc/adc_continuous.h"
  #include "hal/adc_types.h"
  #include "hal/efuse_hal.h"
  #include "soc/soc_caps.h"
  #include "Filters/Notch.h"
  #include "Filters/Envelope.h"
  #include "Filters/EMGFilter.h"
  #include "Filters/EEGFilter.h"
  #include "Filters/EOGFilter.h"
  #include "Filters/BlinkEnvelope.h"
  #include "Filters/BetaPower.h"

  typedef BlinkEnvelopeT<(BLINK_ENVELOPE_MS * SAMPLE_RATE) / 1000> BlinkEnvelope;
#endif

// Pins
#define IR_RECV_PIN     23
#define IR_SEND_PIN     22
#define USER_BTN_PIN    9

// NeoPixel status ring. Pixel 0 BLE, pixel 5 battery.
#define PIN_NEOPIXEL        15
#define PIXEL_COUNT         6
#define BLE_LED             0
#define BATTERY_LED         5
#define BATTERY_VOLTAGE_PIN A6

// Timing
#define HOLD_THRESHOLD  1500
#define DEBOUNCE_MS     200
#define LIST_DELAY_MS   1000
#define LIST_PACE_MS    15     // one list entry per connection interval
#define RECORD_TIMEOUT  15000

// Limits
// Fixed slots. Every profile and every command always exists as a place to
// put something, so nothing is ever created or destroyed, only filled in and
// cleared out.
#define MAX_PROFILES    5
#define MAX_COMMANDS    10     // per profile
#define MAX_NAME_LEN    16
#define RAW_BUF_LEN     1024
#define IR_TIMEOUT_MS   15
#define IR_FREQ_HZ      38000

#define NVS_NAMESPACE   "ir"
#define STORE_VERSION   3      // bump to wipe the board on the next boot
#define BLE_NAME        "NPG-IR"
#define SVC_UUID        "12345678-1234-1234-1234-1234567890ab"
#define NOTIFY_UUID     "12345678-1234-1234-1234-1234567890ac"
#define WRITE_UUID      "12345678-1234-1234-1234-1234567890ad"

// Board -> app
#define EV_PROFILE      0x10   // [p, cmdCount | 0x80 when named, nameLen, name...]
#define EV_PROFILE_END  0x11
#define EV_LIST_ENTRY   0x12   // [id, flags, nameLen, name...]  bit0 active, bit1 recorded
#define EV_LIST_END     0x14
#define EV_SCREEN       0x15   // [screen, profile, homeCursor, cmdCursor]
#define EV_STREAM       0x16   // one level per control: [scrollDown(2), scrollUp(2), fire(2), home(2)]
#define EV_BLINK        0x17   // [channel, count]
#define EV_CHANNELS     0x18   // [available, notchHz, filter x6]
#define EV_TRIGGER      0x19   // [action]
#define EV_TUNING       0x1C   // [channel, muscle(2), release(2), focus(2), blink(2)]
#define EV_MAPPING      0x1D   // [ch,gest] x4
#define EV_CONFIG_END   0x1E   // every config notification has been sent
#define EV_PROFILE_SAVED 0x1A  // [p, nameLen, name...]
#define EV_PROFILE_DEL  0x1B   // [p]
#define EV_CAPTURE      0x20   // [isDup, dupId, proto(2), bits(2), value(8)]
#define EV_SAVED        0x21   // [id, nameLen, name...]
#define EV_DELETED      0x22   // [id]
#define EV_ACTIVE       0x23   // [id]
#define EV_OK           0x24
#define EV_FAIL         0x25
#define EV_WIPED        0x26
#define EV_LISTENING    0x27   // [timeoutSec]
#define EV_LISTEN_END   0x28

// App -> board
#define CMD_SET_ACTIVE     0x01   // [id]
#define CMD_DELETE         0x02   // [id]
#define CMD_RENAME         0x03   // [id, name...]
#define CMD_SAVE_NEW       0x04   // [name...]
#define CMD_SAVE_OVER      0x05   // [id, name...]
#define CMD_GET_LIST       0x06
#define CMD_FIRE           0x07   // [id]
#define CMD_WIPE           0x08
#define CMD_CANCEL_REC     0x09
#define CMD_DISCARD        0x0A
#define CMD_GET_PROFILES   0x0B
#define CMD_OPEN_PROFILE   0x0C   // [p]
#define CMD_GO_HOME        0x0D
#define CMD_ADD_PROFILE    0x0E   // [name...]
#define CMD_RENAME_PROFILE 0x0F   // [p, name...]
#define CMD_DEL_PROFILE    0x10   // [p]
#define CMD_SET_EDIT       0x11   // [0|1]
#define CMD_SET_TUNING     0x12   // [channel, muscle(2), release(2), focus(2), blink(2)]
#define CMD_SET_MAPPING    0x13   // [ch,gest] x4
#define CMD_GET_CONFIG     0x14
#define CMD_START_REC      0x15
#define CMD_SET_CHANNELS   0x16   // [notchHz, filter x6], re-inits sampling

// What the board can be asked to do. Index into triggerFor[]. Index 0 is
// unused, actions start at 1.
#define ACT_SCROLL_DOWN  1
#define ACT_SCROLL_UP    2
#define ACT_FIRE         3
#define ACT_HOME         4
#define ACTION_COUNT     5

#define SCREEN_HOME     0
#define SCREEN_PROFILE  1

IRrecv irrecv(IR_RECV_PIN, RAW_BUF_LEN, IR_TIMEOUT_MS, true);
IRsend irsend(IR_SEND_PIN);
Preferences prefs;
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// NeoPixel state
uint32_t bleColor     = 0;
uint32_t batteryColor = 0;

uint32_t shownBleColor     = 0xFFFFFFFF;
uint32_t shownBatteryColor = 0xFFFFFFFF;

unsigned long lastCmdSentMs = 0;

// ---- Battery ----
#define BATTERY_CHECK_MS   30000   // read the battery every 30 s
#define BATTERY_SAMPLE_MS  100     // accumulate a reading this often

unsigned long lastBatteryCheck = 0;
unsigned long lastBatterySample = 0;
uint32_t batteryWinSum = 0;
uint16_t batteryWinCount = 0;
int lastBatteryPct = -1;
uint8_t risingCount = 0;
const uint8_t RISING_THRESHOLD = 3;
const float voltageLUT[] = {
  3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
  3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20
};
const int percentLUT[] = {
  0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
  50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
};
const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

struct CmdEntry {
  bool exists;
  char name[MAX_NAME_LEN + 1];
};

struct ProfileEntry {
  bool     exists;
  char     name[MAX_NAME_LEN + 1];
  CmdEntry cmds[MAX_COMMANDS];
};

ProfileEntry profiles[MAX_PROFILES];

// Where we are. All RAM, all a selection rather than a setting.
uint8_t screen      = SCREEN_HOME;
int     openProfile = -1;   // profile being viewed, -1 at home
int     homeCursor  = -1;   // highlighted profile on the home screen
int     cmdCursor   = -1;   // highlighted command inside the open profile
bool    editMode    = false;

// How many BioAmp channels this playmate actually has, filled in on boot.
uint8_t playmate        = PLAYMATE_PROTO;
uint8_t availableChannels = 3;

// One set of thresholds per channel, because contact quality and muscle size
// differ by placement. Tunable at runtime, seeded from the defaults above and
// replaced by whatever the app pushes.
struct ChannelTuning {
  uint16_t muscleThreshold;
  uint16_t muscleRelease;
  float    focusThreshold;
  float    blinkThreshold;
};

ChannelTuning tuning[MAX_BIOAMP_CHANNELS];
uint8_t channelFilter[MAX_BIOAMP_CHANNELS];   // FILT_*, FILT_OFF means not sampled
int     notchHz = DEFAULT_NOTCH_HZ;

// A control is driven by one gesture on one channel, so both are needed to
// name it. Channel 0xFF means the control is unassigned.
struct Bind {
  uint8_t channel;
  uint8_t gesture;
};

Bind triggerFor[ACTION_COUNT] = {
  { 0xFF, GEST_NONE },          // index 0, unused
  { 0, GEST_CLENCH_HOLD },      // ACT_SCROLL_DOWN
  { 0, GEST_CLENCH },           // ACT_SCROLL_UP
  { 0, GEST_FOCUS },            // ACT_FIRE
  { 0, GEST_TRIPLE_BLINK },     // ACT_HOME
};

static void tuningDefaults() {
  for (int i = 0; i < MAX_BIOAMP_CHANNELS; i++) {
    tuning[i].muscleThreshold = DEFAULT_MUSCLE_THRESHOLD;
    tuning[i].muscleRelease   = DEFAULT_MUSCLE_RELEASE;
    tuning[i].focusThreshold  = DEFAULT_FOCUS_THRESHOLD;
    tuning[i].blinkThreshold  = DEFAULT_BLINK_THRESHOLD;
    channelFilter[i]          = FILT_OFF;
  }
  channelFilter[0] = DEFAULT_FILTER_CH0;
}

// Capture held in RAM until the app supplies a name
bool pendingValid = false;
decode_results pendingResult;
volatile uint16_t pendingRaw[RAW_BUF_LEN];

// Shared by loadIR callers. Only ever touched from loop().
volatile uint16_t scratchRaw[RAW_BUF_LEN];

// Recording latches on after a long press and stays on after the button is
// released, until a signal arrives, the timeout expires, or it is cancelled.
bool     recording = false;
uint32_t recordAt  = 0;

// BLE
BLECharacteristic* pNotify = nullptr;
bool bleConnected = false;
uint32_t connectedAt = 0;

// Work deferred out of the BLE callback so IR and filesystem access
// stay on the main task, which has a much larger stack.
int      listCursor    = -1;   // -1 idle, otherwise the next command slot
int      profileCursor = -1;   // -1 idle, otherwise the next profile slot
int      configCursor  = -1;   // -1 idle, otherwise the next config packet
uint32_t lastListSend  = 0;

volatile bool reqList     = false;
volatile bool reqProfiles = false;
volatile bool reqWipe     = false;
volatile bool reqCancel   = false;
volatile bool reqHome     = false;
volatile bool reqRecord   = false;
volatile bool reqConfig   = false;
volatile bool reqResample = false;   // channel selection changed, restart DMA
volatile int  reqFire     = -1;
volatile int  reqOpen     = -1;

void stopRecording(uint8_t reason);

#if BIOAMP_ENABLED
void bioampSuspend();
void bioampResume();
#endif

// Sends one notification and returns. The stack queues a handful of packets,
// so isolated events are safe to fire back to back. Only a long run needs
// pacing, and the lists are the one place that happens. See serviceList.
void bleNotify(const uint8_t* buf, size_t len) {
  if (!bleConnected || !pNotify) return;
  pNotify->setValue((uint8_t*)buf, len);
  pNotify->notify();
}

void bleNotify1(uint8_t op) { bleNotify(&op, 1); }

// ---- Storage ------------------------------------------------------------
// NVS holds names only, one key per thing so renaming one never rewrites the
// rest. LittleFS holds the waveforms, one file per command.
//   p<p>       profile name
//   c<p>_<c>   command name
//   /w<p>_<c>  waveform

static void profileKey(char* out, size_t len, int p) {
  snprintf(out, len, "p%d", p);
}

static void cmdKey(char* out, size_t len, int p, int c) {
  snprintf(out, len, "c%d_%d", p, c);
}

String irPath(int p, int c) { return String("/w") + p + "_" + c; }

// The layout changed when profiles arrived, so anything written by an older
// build is unreadable. Clearing once on a version bump is cheaper than
// carrying a migration path forever.
static void checkStoreVersion() {
  prefs.begin(NVS_NAMESPACE, false);
  uint8_t found = prefs.getUChar("ver", 0);
  if (found != STORE_VERSION) {
    prefs.clear();
    prefs.putUChar("ver", STORE_VERSION);
    prefs.end();
    LittleFS.format();
    return;
  }
  prefs.end();
}

static void defaultProfileName(char* out, size_t len, int p) {
  snprintf(out, len, "Remote %d", p + 1);
}

static void defaultCmdName(char* out, size_t len, int c) {
  snprintf(out, len, "Command %d", c + 1);
}

void nvsLoad() {
  prefs.begin(NVS_NAMESPACE, true);
  for (int p = 0; p < MAX_PROFILES; p++) {
    char key[12];
    profileKey(key, sizeof(key), p);
    profiles[p].exists = prefs.isKey(key);
    if (profiles[p].exists) {
      String s = prefs.getString(key, "");
      strncpy(profiles[p].name, s.c_str(), MAX_NAME_LEN);
      profiles[p].name[MAX_NAME_LEN] = '\0';
    } else {
      profiles[p].name[0] = '\0';
    }

    for (int c = 0; c < MAX_COMMANDS; c++) {
      cmdKey(key, sizeof(key), p, c);
      bool has = profiles[p].exists && prefs.isKey(key);
      profiles[p].cmds[c].exists = has;
      if (has) {
        String s = prefs.getString(key, "");
        strncpy(profiles[p].cmds[c].name, s.c_str(), MAX_NAME_LEN);
        profiles[p].cmds[c].name[MAX_NAME_LEN] = '\0';
      } else {
        profiles[p].cmds[c].name[0] = '\0';
      }
    }
  }
  prefs.end();

  // Open the first profile straight away, but leave the gestures navigating
  // profiles, so there is always a command list to show and a way to move.
  homeCursor = -1;
  for (int p = 0; p < MAX_PROFILES; p++) {
    if (profiles[p].exists) { homeCursor = p; break; }
  }
  openProfile = homeCursor;
  cmdCursor   = firstCmd(homeCursor);
  screen      = SCREEN_HOME;
}

void nvsSaveProfile(int p) {
  char key[12];
  profileKey(key, sizeof(key), p);
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putString(key, profiles[p].name);
  prefs.end();
}

void nvsSaveCmd(int p, int c) {
  char key[12];
  cmdKey(key, sizeof(key), p, c);
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putString(key, profiles[p].cmds[c].name);
  prefs.end();
}

void nvsRemoveCmd(int p, int c) {
  char key[12];
  cmdKey(key, sizeof(key), p, c);
  prefs.begin(NVS_NAMESPACE, false);
  prefs.remove(key);
  prefs.end();
}

// Layout: proto(2) bits(2) value(8) rawlen(2) state(kStateSizeMax) rawbuf(rawlen*2)
const size_t IR_HEADER_LEN = 2 + 2 + 8 + 2 + kStateSizeMax;

bool saveIR(int p, int c, decode_results* res) {
  File f = LittleFS.open(irPath(p, c), "w");
  if (!f) return false;
  uint16_t proto  = (uint16_t)res->decode_type;
  uint16_t bits   = res->bits;
  uint16_t rawlen = res->rawlen;
  uint64_t value  = res->value;
  // count what the writes accept. f.size() on a file that is still open
  // does not include buffered data, so it cannot be used to verify this.
  size_t written = 0;
  written += f.write((uint8_t*)&proto,  2);
  written += f.write((uint8_t*)&bits,   2);
  written += f.write((uint8_t*)&value,  8);
  written += f.write((uint8_t*)&rawlen, 2);
  written += f.write((uint8_t*)res->state, kStateSizeMax);
  for (uint16_t i = 0; i < rawlen; i++) {
    uint16_t v = res->rawbuf[i];
    written += f.write((uint8_t*)&v, 2);
  }
  f.close();

  size_t expected = IR_HEADER_LEN + (size_t)rawlen * 2;
  if (written != expected) {
    LittleFS.remove(irPath(p, c));
    return false;
  }
  return true;
}

// res->rawbuf must point at a buffer of RAW_BUF_LEN entries.
bool loadIR(int p, int c, decode_results* res) {
  File f = LittleFS.open(irPath(p, c), "r");
  if (!f) return false;
  if (f.size() < IR_HEADER_LEN) { f.close(); return false; }

  uint16_t proto, bits, rawlen;
  uint64_t value;
  f.read((uint8_t*)&proto,  2);
  f.read((uint8_t*)&bits,   2);
  f.read((uint8_t*)&value,  8);
  f.read((uint8_t*)&rawlen, 2);

  // Never trust a length read back from flash
  if (rawlen > RAW_BUF_LEN || f.size() < IR_HEADER_LEN + (size_t)rawlen * 2) {
    f.close();
    return false;
  }

  f.read((uint8_t*)res->state, kStateSizeMax);
  for (uint16_t i = 0; i < rawlen; i++) {
    uint16_t v = 0;
    f.read((uint8_t*)&v, 2);
    res->rawbuf[i] = v;
  }
  f.close();

  res->decode_type = (decode_type_t)proto;
  res->bits   = bits;
  res->value  = value;
  res->rawlen = rawlen;
  return true;
}

void deleteIR(int p, int c) {
  String path = irPath(p, c);
  if (LittleFS.exists(path)) LittleFS.remove(path);
}

// ---- Profile and command helpers ----------------------------------------

bool validProfile(int p) {
  return p >= 0 && p < MAX_PROFILES && profiles[p].exists;
}

bool validCmd(int p, int c) {
  return validProfile(p) && c >= 0 && c < MAX_COMMANDS && profiles[p].cmds[c].exists;
}

int countCommands(int p) {
  if (!validProfile(p)) return 0;
  int n = 0;
  for (int c = 0; c < MAX_COMMANDS; c++) if (profiles[p].cmds[c].exists) n++;
  return n;
}

int freeProfileSlot() {
  for (int p = 0; p < MAX_PROFILES; p++) if (!profiles[p].exists) return p;
  return -1;
}

int freeCmdSlot(int p) {
  if (!validProfile(p)) return -1;
  for (int c = 0; c < MAX_COMMANDS; c++) if (!profiles[p].cmds[c].exists) return c;
  return -1;
}

// Starting from nothing, a step forward should land on the first slot and a
// step back on the last. Plain arithmetic from -1 lands one short going
// backwards, so the empty cursor is parked at the far end instead.
static int stepFrom(int from, int dir, int count) {
  if (from >= 0) return from;
  return dir > 0 ? -1 : count;
}

// next occupied slot in the given direction, wrapping, -1 if nothing is there
int nextProfile(int from, int dir) {
  from = stepFrom(from, dir, MAX_PROFILES);
  for (int k = 1; k <= MAX_PROFILES; k++) {
    int i = ((from + dir * k) % MAX_PROFILES + MAX_PROFILES) % MAX_PROFILES;
    if (profiles[i].exists) return i;
  }
  return -1;
}

int nextCmd(int p, int from, int dir) {
  if (!validProfile(p)) return -1;
  from = stepFrom(from, dir, MAX_COMMANDS);
  for (int k = 1; k <= MAX_COMMANDS; k++) {
    int i = ((from + dir * k) % MAX_COMMANDS + MAX_COMMANDS) % MAX_COMMANDS;
    if (profiles[p].cmds[i].exists) return i;
  }
  return -1;
}

int firstCmd(int p) {
  if (!validProfile(p)) return -1;
  for (int c = 0; c < MAX_COMMANDS; c++) if (profiles[p].cmds[c].exists) return c;
  return -1;
}

// Duplicates are only interesting inside one appliance. Two different ACs
// sharing a code is normal and should not be flagged.
int findDuplicate(int p, decode_results* res) {
  if (res->decode_type == UNKNOWN || !validProfile(p)) return -1;
  decode_results stored;
  stored.rawbuf = scratchRaw;
  for (int c = 0; c < MAX_COMMANDS; c++) {
    if (!profiles[p].cmds[c].exists) continue;
    if (!loadIR(p, c, &stored)) continue;
    if (stored.decode_type == res->decode_type && stored.value == res->value) return c;
  }
  return -1;
}

bool fireCommand(int c) {
  if (!validCmd(openProfile, c)) return false;

  decode_results res;
  res.rawbuf = scratchRaw;
  if (!loadIR(openProfile, c, &res)) return false;

  lastCmdSentMs = millis();

  if (res.decode_type == UNKNOWN) {
    // rawbuf holds capture ticks, sendRaw wants microseconds
    uint16_t len = getCorrectedRawLength(&res);
    uint16_t* raw = resultToRawArray(&res);
    if (!raw) return false;
    irsend.sendRaw(raw, len, IR_FREQ_HZ);
    delete[] raw;
  } else if (hasACState(res.decode_type)) {
    irsend.send(res.decode_type, res.state, res.bits / 8);
  } else {
    irsend.send(res.decode_type, res.value, res.bits);
  }
  return true;
}

void deleteCommand(int p, int c) {
  if (!validCmd(p, c)) return;
  profiles[p].cmds[c].exists  = false;
  profiles[p].cmds[c].name[0] = '\0';
  deleteIR(p, c);
  nvsRemoveCmd(p, c);
  if (p == openProfile && cmdCursor == c) cmdCursor = firstCmd(p);
}

void deleteProfile(int p) {
  if (!validProfile(p)) return;
  for (int c = 0; c < MAX_COMMANDS; c++) {
    if (profiles[p].cmds[c].exists) deleteCommand(p, c);
  }
  profiles[p].exists  = false;
  profiles[p].name[0] = '\0';

  char key[12];
  profileKey(key, sizeof(key), p);
  prefs.begin(NVS_NAMESPACE, false);
  prefs.remove(key);
  prefs.end();

  if (openProfile == p) {
    openProfile = -1;
    cmdCursor   = -1;
    screen      = SCREEN_HOME;
  }
  if (homeCursor == p) homeCursor = nextProfile(p, 1);
  if (homeCursor == p) homeCursor = -1;   // it was the only one
}

void wipeAll() {
  stopRecording(2);
  for (int p = 0; p < MAX_PROFILES; p++) {
    for (int c = 0; c < MAX_COMMANDS; c++) {
      deleteIR(p, c);
      profiles[p].cmds[c].exists  = false;
      profiles[p].cmds[c].name[0] = '\0';
    }
    profiles[p].exists  = false;
    profiles[p].name[0] = '\0';
  }
  openProfile  = -1;
  homeCursor   = -1;
  cmdCursor    = -1;
  screen       = SCREEN_HOME;
  pendingValid = false;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.clear();
  prefs.putUChar("ver", STORE_VERSION);
  prefs.end();
}

void copyName(char* dst, const uint8_t* src, size_t len) {
  if (len > MAX_NAME_LEN) len = MAX_NAME_LEN;
  memcpy(dst, src, len);
  dst[len] = '\0';
}

// ---- Notifications ------------------------------------------------------

void notifyScreen() {
  uint8_t buf[5] = {
    EV_SCREEN,
    screen,
    (uint8_t)(openProfile < 0 ? 0xFF : openProfile),
    (uint8_t)(homeCursor  < 0 ? 0xFF : homeCursor),
    (uint8_t)(cmdCursor   < 0 ? 0xFF : cmdCursor),
  };
  bleNotify(buf, sizeof(buf));
}

void notifyChannels() {
  uint8_t buf[3 + MAX_BIOAMP_CHANNELS];
  buf[0] = EV_CHANNELS;
  buf[1] = availableChannels;
  buf[2] = (uint8_t)notchHz;
  for (int i = 0; i < MAX_BIOAMP_CHANNELS; i++) buf[3 + i] = channelFilter[i];
  bleNotify(buf, sizeof(buf));
}

void notifyTuning(int ch) {
  uint16_t focus = (uint16_t)(tuning[ch].focusThreshold * 10.0f);
  uint16_t blink = (uint16_t)tuning[ch].blinkThreshold;
  uint8_t buf[10] = {
    EV_TUNING, (uint8_t)ch,
    (uint8_t)(tuning[ch].muscleThreshold & 0xFF), (uint8_t)(tuning[ch].muscleThreshold >> 8),
    (uint8_t)(tuning[ch].muscleRelease   & 0xFF), (uint8_t)(tuning[ch].muscleRelease   >> 8),
    (uint8_t)(focus & 0xFF), (uint8_t)(focus >> 8),
    (uint8_t)(blink & 0xFF), (uint8_t)(blink >> 8),
  };
  bleNotify(buf, sizeof(buf));
}

void notifyMapping() {
  uint8_t buf[9] = {
    EV_MAPPING,
    triggerFor[ACT_SCROLL_DOWN].channel, triggerFor[ACT_SCROLL_DOWN].gesture,
    triggerFor[ACT_SCROLL_UP].channel,   triggerFor[ACT_SCROLL_UP].gesture,
    triggerFor[ACT_FIRE].channel,        triggerFor[ACT_FIRE].gesture,
    triggerFor[ACT_HOME].channel,        triggerFor[ACT_HOME].gesture,
  };
  bleNotify(buf, sizeof(buf));
}

// The whole config is several packets, so it is paced out of the main loop the
// same way the lists are rather than dumped into the notify queue at once.
// Step 0 is the channels, 1 to MAX_BIOAMP_CHANNELS the per channel tuning, then
// the mapping, then the end marker.
void serviceConfig(uint32_t now) {
  if (configCursor < 0) return;
  if (!bleConnected) { configCursor = -1; return; }
  if (now - lastListSend < LIST_PACE_MS) return;

  if (configCursor == 0) {
    notifyChannels();
  } else if (configCursor <= MAX_BIOAMP_CHANNELS) {
    notifyTuning(configCursor - 1);
  } else if (configCursor == MAX_BIOAMP_CHANNELS + 1) {
    notifyMapping();
  } else {
    bleNotify1(EV_CONFIG_END);
    configCursor = -1;
    return;
  }

  lastListSend = now;
  configCursor++;
}

void notifyCapture(int dupId) {
  uint8_t buf[15];
  uint16_t proto = (uint16_t)pendingResult.decode_type;
  uint64_t value = pendingResult.value;
  buf[0] = EV_CAPTURE;
  buf[1] = (dupId >= 0) ? 1 : 0;
  buf[2] = (dupId >= 0) ? (uint8_t)dupId : 0xFF;
  buf[3] = proto & 0xFF;
  buf[4] = proto >> 8;
  buf[5] = pendingResult.bits & 0xFF;
  buf[6] = pendingResult.bits >> 8;
  memcpy(&buf[7], &value, 8);
  bleNotify(buf, sizeof(buf));
}

void notifySaved(int p, int c) {
  uint8_t buf[3 + MAX_NAME_LEN];
  uint8_t nlen = strlen(profiles[p].cmds[c].name);
  buf[0] = EV_SAVED;
  buf[1] = (uint8_t)c;
  buf[2] = nlen;
  memcpy(&buf[3], profiles[p].cmds[c].name, nlen);
  bleNotify(buf, 3 + nlen);
}

void notifyProfileSaved(int p) {
  uint8_t buf[3 + MAX_NAME_LEN];
  uint8_t nlen = strlen(profiles[p].name);
  buf[0] = EV_PROFILE_SAVED;
  buf[1] = (uint8_t)p;
  buf[2] = nlen;
  memcpy(&buf[3], profiles[p].name, nlen);
  bleNotify(buf, 3 + nlen);
}

// The lists are the only bursts of notifications this firmware sends. Pushing
// faster than the connection interval overflows the stack queue and entries go
// missing, so they are paced. Pacing with delay() would stall loop() for most
// of a second, so each walks one entry per call instead.
void startList() {
  listCursor    = 0;
  profileCursor = -1;
  lastListSend  = millis() - LIST_PACE_MS;
}

void startProfileList() {
  profileCursor = 0;
  listCursor    = -1;
  lastListSend  = millis() - LIST_PACE_MS;
}

void serviceList(uint32_t now) {
  if (listCursor < 0 && profileCursor < 0) return;
  if (!bleConnected) { listCursor = -1; profileCursor = -1; return; }
  if (now - lastListSend < LIST_PACE_MS) return;

  if (profileCursor >= 0) {
    if (profileCursor >= MAX_PROFILES) {
      bleNotify1(EV_PROFILE_END);
      profileCursor = -1;
      return;
    }
    uint8_t buf[4 + MAX_NAME_LEN];
    uint8_t nlen = strlen(profiles[profileCursor].name);
    buf[0] = EV_PROFILE;
    buf[1] = (uint8_t)profileCursor;
    // the count never exceeds ten, so the top bit carries "this is a remote"
    buf[2] = (uint8_t)countCommands(profileCursor) |
             (profiles[profileCursor].exists ? 0x80 : 0);
    buf[3] = nlen;
    memcpy(&buf[4], profiles[profileCursor].name, nlen);
    bleNotify(buf, 4 + nlen);
    lastListSend = now;
    profileCursor++;
    return;
  }

  if (!validProfile(openProfile)) { listCursor = -1; bleNotify1(EV_LIST_END); return; }

  if (listCursor >= MAX_COMMANDS) {
    bleNotify1(EV_LIST_END);
    listCursor = -1;
    return;
  }

  // Every slot is sent, empty ones included, because the app lays all ten out
  // whether or not they hold anything yet.
  const CmdEntry& cmd = profiles[openProfile].cmds[listCursor];
  uint8_t buf[4 + MAX_NAME_LEN];
  uint8_t nlen = strlen(cmd.name);
  buf[0] = EV_LIST_ENTRY;
  buf[1] = (uint8_t)listCursor;
  buf[2] = (listCursor == cmdCursor ? 1 : 0) | (cmd.exists ? 2 : 0);
  buf[3] = nlen;
  memcpy(&buf[4], cmd.name, nlen);
  bleNotify(buf, 4 + nlen);

  lastListSend = now;
  listCursor++;
}

// ---- Navigation ---------------------------------------------------------

// Moves the cursor to the first remote. Does not change what is open,
// opening is the select button's job.
void goHome() {
  screen     = SCREEN_HOME;
  homeCursor = nextProfile(-1, 1);
  notifyScreen();
}

void openProfileAt(int p) {
  if (!validProfile(p)) {
    return;
  }
  screen      = SCREEN_PROFILE;
  openProfile = p;
  homeCursor  = p;
  // Something is always selected, so opening a remote lands on the first
  // command it actually has. -1 only when it has none yet.
  cmdCursor   = firstCmd(p);
  notifyScreen();
  reqList = true;
}

// ---- NeoPixel status ----------------------------------------------------
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

#if BIOAMP_ENABLED
uint16_t bioampBatteryReading();   // defined with the sampling code below
#endif

// Averaged reading for the battery divider. With the bioamp enabled the
// continuous driver owns ADC1, so the value arrives through the DMA pattern
// instead of a one shot read. On the C6 a raw count is close enough to a
// millivolt for the curve below, which is how the other NPG Lite firmware
// handles it too.
static float batteryReading() {
#if BIOAMP_ENABLED
  uint16_t avg = bioampBatteryReading();
  return (avg > 0) ? (float)avg : 0.0f;
#else
  float avg = (batteryWinCount > 0) ? (batteryWinSum / batteryWinCount)
                                    : analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
  batteryWinSum = 0;
  batteryWinCount = 0;
  return avg;
#endif
}

int getCurrentBatteryPercentage() {
  float avgRaw = batteryReading();
  if (avgRaw <= 0.0f) return (lastBatteryPct < 0) ? 100 : lastBatteryPct;
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

// Turns a battery percentage into the color for the battery led.
uint32_t batteryPercentToColor(int percent) {
  if (percent <= 20) return pixel.Color(20, 0, 0);
  if (percent <= 70) return pixel.Color(35, 7, 0);
  return pixel.Color(0, 20, 0);
}

// Writes both leds and shows them, but only when a color changed.
void updateStatusLeds(bool connected) {
  // Bluetooth led: red until connected, green once connected.
  bleColor = connected ? pixel.Color(0, 20, 0) : pixel.Color(20, 0, 0);

  if (bleColor == shownBleColor && batteryColor == shownBatteryColor) {
    return;
  }

  shownBleColor = bleColor;
  shownBatteryColor = batteryColor;

  pixel.setPixelColor(BLE_LED, bleColor);
  pixel.setPixelColor(BATTERY_LED, batteryColor);
  pixel.show();
}

// ---- Recording ----------------------------------------------------------

void startRecording() {
  if (recording) return;
  // a signal has nowhere to go without a profile open
  if (!validProfile(openProfile)) {
    bleNotify1(EV_FAIL);
    return;
  }
  // a new recording supersedes a capture that was never named, so an
  // abandoned one can never block the button
  pendingValid = false;
  recording = true;
  recordAt  = millis();
  // Capturing an IR frame means catching every edge on one core. Sampling
  // does nothing useful here, because triggers are suspended while the
  // receiver is armed, so it stands down until the capture is over.
#if BIOAMP_ENABLED
  bioampSuspend();
#endif
  irrecv.enableIRIn();
  uint8_t buf[2] = { EV_LISTENING, RECORD_TIMEOUT / 1000 };
  bleNotify(buf, 2);
}

// reason: 0 timed out, 1 cancelled by the button, 2 cancelled by the app
void stopRecording(uint8_t reason) {
  if (!recording) return;
  irrecv.disableIRIn();
  recording = false;
#if BIOAMP_ENABLED
  bioampResume();
#endif
  uint8_t buf[2] = { EV_LISTEN_END, reason };
  bleNotify(buf, 2);
}

// ---- BLE callbacks ------------------------------------------------------

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    connectedAt  = millis();
    // The app asks for the config and then the profiles, in that order, so
    // the two paced bursts never share the wire and drop each other's packets.
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    reqList      = false;
    reqProfiles  = false;
    editMode     = false;
    // nothing can name a capture now, and only loop() may touch the receiver
    pendingValid = false;
    reqCancel    = true;
    s->getAdvertising()->start();
  }
};

class WriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    uint8_t* d = c->getData();
    size_t   n = c->getLength();
    if (n == 0) return;

    switch (d[0]) {

      case CMD_SET_ACTIVE: {
        if (n < 2) return;
        int id = d[1];
        if (!validCmd(openProfile, id)) return;
        cmdCursor = id;
        uint8_t buf[2] = { EV_ACTIVE, (uint8_t)id };
        bleNotify(buf, 2);
        return;
      }

      case CMD_DELETE: {
        if (n < 2) return;
        int id = d[1];
        if (!validCmd(openProfile, id)) return;
        deleteCommand(openProfile, id);
        uint8_t buf[2] = { EV_DELETED, (uint8_t)id };
        bleNotify(buf, 2);
        return;
      }

      case CMD_RENAME: {
        if (n < 3) return;
        int id = d[1];
        if (!validCmd(openProfile, id)) return;
        copyName(profiles[openProfile].cmds[id].name, &d[2], n - 2);
        nvsSaveCmd(openProfile, id);
        notifySaved(openProfile, id);
        return;
      }

      case CMD_SAVE_NEW: {
        if (!pendingValid || n < 2 || !validProfile(openProfile)) {
          bleNotify1(EV_FAIL);
          return;
        }
        int id = freeCmdSlot(openProfile);
        if (id < 0) {
          bleNotify1(EV_FAIL);
          return;
        }
        if (!saveIR(openProfile, id, &pendingResult)) { bleNotify1(EV_FAIL); return; }
        copyName(profiles[openProfile].cmds[id].name, &d[1], n - 1);
        profiles[openProfile].cmds[id].exists = true;
        if (cmdCursor < 0) cmdCursor = id;
        nvsSaveCmd(openProfile, id);
        notifySaved(openProfile, id);
        pendingValid = false;
        return;
      }

      case CMD_SAVE_OVER: {
        if (!pendingValid || n < 3 || !validProfile(openProfile)) {
          bleNotify1(EV_FAIL);
          return;
        }
        int id = d[1];
        if (id < 0 || id >= MAX_COMMANDS) {
          bleNotify1(EV_FAIL);
          return;
        }
        // saveIR truncates, so the old waveform only goes once the new one is safe
        if (!saveIR(openProfile, id, &pendingResult)) { bleNotify1(EV_FAIL); return; }
        copyName(profiles[openProfile].cmds[id].name, &d[2], n - 2);
        profiles[openProfile].cmds[id].exists = true;
        // the first command a remote gets is the one it selects
        if (cmdCursor < 0) cmdCursor = id;
        nvsSaveCmd(openProfile, id);
        notifySaved(openProfile, id);
        pendingValid = false;
        return;
      }

      case CMD_ADD_PROFILE: {
        if (n < 2) { bleNotify1(EV_FAIL); return; }
        int p = freeProfileSlot();
        if (p < 0) {
          bleNotify1(EV_FAIL);
          return;
        }
        copyName(profiles[p].name, &d[1], n - 1);
        profiles[p].exists = true;
        for (int i = 0; i < MAX_COMMANDS; i++) {
          profiles[p].cmds[i].exists  = false;
          profiles[p].cmds[i].name[0] = '\0';
        }
        if (homeCursor < 0) homeCursor = p;
        nvsSaveProfile(p);
        notifyProfileSaved(p);
        return;
      }

      // Naming a slot is what creates the remote. The same write renames one
      // that already exists, so there is nothing else to add.
      case CMD_RENAME_PROFILE: {
        if (n < 3) return;
        int p = d[1];
        if (p < 0 || p >= MAX_PROFILES) return;
        bool fresh = !profiles[p].exists;
        copyName(profiles[p].name, &d[2], n - 2);
        profiles[p].exists = true;
        if (fresh) {
          for (int c = 0; c < MAX_COMMANDS; c++) {
            profiles[p].cmds[c].exists = false;
            defaultCmdName(profiles[p].cmds[c].name, sizeof(profiles[p].cmds[c].name), c);
          }
          if (homeCursor < 0) { homeCursor = p; openProfile = p; }
        }
        nvsSaveProfile(p);
        notifyProfileSaved(p);
        return;
      }

      case CMD_DEL_PROFILE: {
        if (n < 2) return;
        int p = d[1];
        if (!validProfile(p)) return;
        deleteProfile(p);
        uint8_t buf[2] = { EV_PROFILE_DEL, (uint8_t)p };
        bleNotify(buf, 2);
        return;
      }

      case CMD_SET_EDIT:
        if (n < 2) return;
        editMode = d[1] != 0;
        return;

      case CMD_SET_TUNING: {
        if (n < 10) return;
        int ch = d[1];
        if (ch < 0 || ch >= MAX_BIOAMP_CHANNELS) return;
        tuning[ch].muscleThreshold = d[2] | (d[3] << 8);
        tuning[ch].muscleRelease   = d[4] | (d[5] << 8);
        tuning[ch].focusThreshold  = (float)(d[6] | (d[7] << 8)) / 10.0f;
        tuning[ch].blinkThreshold  = (float)(d[8] | (d[9] << 8));
        return;
      }

      case CMD_SET_MAPPING: {
        if (n < 9) return;
        for (int i = 0; i < 4; i++) {
          uint8_t ch   = d[1 + i * 2];
          uint8_t gest = d[2 + i * 2];
          if (gest >= GESTURE_COUNT) continue;
          if (ch >= MAX_BIOAMP_CHANNELS) ch = 0xFF;      // unassigned
          triggerFor[ACT_SCROLL_DOWN + i] = { ch, gest };
        }
        // Echo what actually took effect. The app draws its live bars from
        // the mapping, so it has to follow the board rather than its own
        // copy, or a write that never landed leaves a bar reading zero.
        notifyMapping();
        return;
      }

      // The one setting that changes what the hardware is doing, so the
      // restart is queued for the main loop rather than done here.
      case CMD_SET_CHANNELS: {
        if (n < 2 + MAX_BIOAMP_CHANNELS) return;
        if (d[1] == 50 || d[1] == 60) notchHz = d[1];
        for (int i = 0; i < MAX_BIOAMP_CHANNELS; i++) {
          uint8_t f = d[2 + i];
          channelFilter[i] = (f <= FILT_EOG && i < availableChannels) ? f : FILT_OFF;
        }
        reqResample = true;
        return;
      }

      case CMD_GET_CONFIG:   reqConfig   = true; return;
      case CMD_START_REC:    reqRecord   = true; return;
      case CMD_GET_LIST:     reqList     = true; return;
      case CMD_GET_PROFILES: reqProfiles = true; return;
      case CMD_GO_HOME:      reqHome     = true; return;
      case CMD_WIPE:         reqWipe     = true; return;
      case CMD_CANCEL_REC:   reqCancel   = true; return;

      case CMD_OPEN_PROFILE:
        if (n < 2) return;
        reqOpen = d[1];
        return;

      case CMD_DISCARD:
        pendingValid = false;
        return;

      case CMD_FIRE:
        if (n < 2) return;
        reqFire = d[1];
        return;
    }
  }
};

#if BIOAMP_ENABLED
// ===========================================================================
// Bio-potential sampling and triggers
//
// The ADC runs in continuous DMA mode, so samples are taken by hardware and
// never jitter, no matter what loop() is doing. Everything else in this
// firmware blocks at some point: flash writes, IR transmission, the list
// stream. Rather than try to keep filtering through those, bioampService spots
// the gap, throws away what DMA collected, and resets the filters. That keeps
// the signal honest at the cost of a short blind window.
//
// Only the channels the app selected are sampled. Changing that selection
// tears the driver down and rebuilds the pattern, so an unused channel costs
// nothing at all. Every channel is notched, and what happens after that
// depends on the filter chosen for it:
//
//   FILT_EMG  EMG band and envelope. One muscle level.
//   FILT_EEG  all three. The EMG path for a clench, the EEG band for the beta
//             share that means focus, and that same EEG output through the EOG
//             high pass for blinks. One pair of electrodes, three gestures.
//   FILT_EOG  the EEG band into the EOG high pass. Blinks only.
// ===========================================================================

#define ADC_MAX_PATTERN  (MAX_BIOAMP_CHANNELS + 1)   // channels plus battery
#define ADC_FRAME_BYTES  (ADC_MAX_PATTERN * SOC_ADC_DIGI_RESULT_BYTES * SAMPLE_BLOCK_COUNT)

static adc_continuous_handle_t adcHandle = nullptr;
static SemaphoreHandle_t       adcSem    = nullptr;
static bool                    bioampReady = false;

// Everything one channel needs. The filters it does not use cost their state
// and nothing more, which is far cheaper than allocating them on the fly.
struct ChannelState {
  Notch         notch;
  EMGFilter     emg;
  Envelope      emgEnv;
  EEGFilter     eeg;
  BetaPower     beta;
  EOGFilter     eog;
  BlinkEnvelope blinkEnv;

  int   muscleLevel = 0;
  float blinkLevel  = 0.0f;

  bool     held = false, repeating = false;
  uint32_t startMs = 0, lastRepeatMs = 0, lastClenchMs = 0;

  bool     blinkArmed = true;
  uint8_t  blinkRun   = 0;
  uint32_t lastBlinkMs = 0;

  uint32_t lastFocusMs = 0;

  // Resets the IIR filters. Their state is meaningless across a gap, so
  // this is for a genuine fresh start, when sampling begins.
  void resetFilters() {
    notch.reset();
    emg.reset();
    eeg.reset();
    eog.reset();
    beta.resetWindow();
    clearGestures();
  }

  // Resets gesture state after a gap in the samples. Filter and envelope
  // state is kept, since it describes a signal level that has not moved.
  // Zeroing it would cause a ringing transient instead.
  void clearGestures() {
    beta.resetWindow();
    held = repeating = false;
    blinkArmed = true;
    blinkRun   = 0;
  }
};

static ChannelState channel[MAX_BIOAMP_CHANNELS];

// Which physical channels are in the DMA pattern, in pattern order.
static uint8_t activeChannel[MAX_BIOAMP_CHANNELS];
static uint8_t activeCount  = 0;
static uint8_t batterySlot  = 0;

// maps a physical ADC channel back to its slot in the pattern
static int8_t adcChannelIndex[SOC_ADC_CHANNEL_NUM(0)];

static uint32_t lastServiceMs = 0;
static uint32_t settleUntil   = 0;
static uint32_t lastStreamMs  = 0;

static uint32_t battWinSum   = 0;
static uint16_t battWinCount = 0;

// The C6 rev1 ADC tops out below full scale, so stretch it back.
static inline uint16_t fixRaw(uint16_t raw) {
  static uint32_t chiprev = efuse_hal_chip_revision();
  if (chiprev == 1) {
    uint32_t v = (uint32_t)raw * 4095u / 3249u;
    return (uint16_t)(v > 4095u ? 4095u : v);
  }
  return raw;
}

static bool IRAM_ATTR adcOnConvDone(adc_continuous_handle_t handle,
                                    const adc_continuous_evt_data_t* edata,
                                    void* user_data) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(adcSem, &woken);
  return woken == pdTRUE;
}

// Stop and restart conversions without rebuilding the pattern, so an IR
// capture gets the core to itself. Filters restart on resume, because the gap
// leaves their state meaningless.
void bioampSuspend() {
  if (adcHandle && bioampReady) {
    adc_continuous_stop(adcHandle);
    bioampReady = false;
  }
}

void bioampResume() {
  if (!adcHandle || bioampReady) return;
  if (adc_continuous_start(adcHandle) != ESP_OK) return;
  for (int i = 0; i < MAX_BIOAMP_CHANNELS; i++) channel[i].clearGestures();
  settleUntil   = millis() + TRIGGER_SETTLE;
  lastServiceMs = 0;
  bioampReady   = true;
}

static void bioampStop() {
  if (adcHandle) {
    adc_continuous_stop(adcHandle);
    adc_continuous_deinit(adcHandle);
    adcHandle = nullptr;
  }
  bioampReady = false;
}

bool bioampBegin() {
  if (!adcSem) {
    adcSem = xSemaphoreCreateBinary();
    if (!adcSem) return false;
  }

  // Build the pattern from whatever the app selected, battery always last.
  activeCount = 0;
  for (int i = 0; i < MAX_BIOAMP_CHANNELS && i < availableChannels; i++) {
    if (channelFilter[i] != FILT_OFF) activeChannel[activeCount++] = i;
  }
  batterySlot = activeCount;

  const int patternLen = activeCount + 1;
  static adc_digi_pattern_config_t pattern[ADC_MAX_PATTERN];
  for (int i = 0; i < patternLen; i++) {
    pattern[i].atten     = ADC_ATTEN_DB_12;
    pattern[i].channel   = (i == batterySlot) ? BATTERY_PIN : activeChannel[i];
    pattern[i].unit      = ADC_UNIT_1;
    pattern[i].bit_width = ADC_BITWIDTH_12;
  }
  for (size_t i = 0; i < sizeof(adcChannelIndex); i++) adcChannelIndex[i] = -1;
  for (int i = 0; i < patternLen; i++) adcChannelIndex[pattern[i].channel] = i;

  const size_t frameBytes = patternLen * SOC_ADC_DIGI_RESULT_BYTES * SAMPLE_BLOCK_COUNT;

  adc_continuous_handle_cfg_t handleCfg = {
    .max_store_buf_size = (uint32_t)(frameBytes * 4),
    .conv_frame_size    = (uint32_t)frameBytes,
  };
  if (adc_continuous_new_handle(&handleCfg, &adcHandle) != ESP_OK) return false;

  adc_continuous_evt_cbs_t cbs = { .on_conv_done = adcOnConvDone };
  if (adc_continuous_register_event_callbacks(adcHandle, &cbs, nullptr) != ESP_OK) return false;

  adc_continuous_config_t cfg = {
    .pattern_num    = (uint32_t)patternLen,
    .adc_pattern    = pattern,
    .sample_freq_hz = (uint32_t)(SAMPLE_RATE * patternLen),
    .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
    .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };
  if (adc_continuous_config(adcHandle, &cfg) != ESP_OK) return false;
  if (adc_continuous_start(adcHandle) != ESP_OK) return false;

  for (int i = 0; i < MAX_BIOAMP_CHANNELS; i++) {
    channel[i].notch.setFrequency(notchHz);
    channel[i].beta.begin((float)SAMPLE_RATE);
    channel[i].emgEnv.reset();
    channel[i].blinkEnv.reset();
    channel[i].muscleLevel = 0;
    channel[i].blinkLevel  = 0.0f;
    channel[i].resetFilters();
  }

  settleUntil   = millis() + TRIGGER_SETTLE;
  lastServiceMs = 0;
  bioampReady   = true;

  return true;
}

// Called when the app applies a new channel selection. channelFilter[] is
// already updated, so rebuilding from scratch picks up the new pattern.
void bioampRestart() {
  bioampStop();
  if (!bioampBegin()) Serial.println("[BIO] restart failed");
}

// Called after a gap in sampling. Drops the buffered samples and resets
// gesture state. Filter state is kept, since a gap does not change the
// signal level, and zeroing it would cause ringing on resume.
static void bioampReset() {
  if (adcHandle) {
    uint8_t scratch[ADC_FRAME_BYTES];
    uint32_t got = 0;
    while (adc_continuous_read(adcHandle, scratch, sizeof(scratch), &got, 0) == ESP_OK && got) {}
  }
  for (int i = 0; i < MAX_BIOAMP_CHANNELS; i++) channel[i].clearGestures();
  settleUntil   = millis() + TRIGGER_SETTLE;
  lastServiceMs = 0;
}

// Moves the highlight on whichever screen is showing.
static void stepCursor(int dir) {
  if (screen == SCREEN_HOME) {
    int next = nextProfile(homeCursor, dir);
    if (next < 0 || next == homeCursor) return;
    homeCursor = next;
    notifyScreen();
  } else {
    int next = nextCmd(openProfile, cmdCursor, dir);
    if (next < 0 || next == cmdCursor) return;
    cmdCursor = next;
    uint8_t buf[2] = { EV_ACTIVE, (uint8_t)cmdCursor };
    bleNotify(buf, 2);
  }
}

// Heavy work is queued rather than run here, because this is called from
// inside the sampling service and IR or flash access would stall it.
static void runAction(uint8_t action) {
  switch (action) {
    case ACT_SCROLL_DOWN: stepCursor(+1); break;
    case ACT_SCROLL_UP:   stepCursor(-1); break;
    case ACT_FIRE:
      if (screen == SCREEN_HOME) reqOpen = homeCursor;
      else                       reqFire = cmdCursor;
      break;
    case ACT_HOME:
      reqHome = true;
      break;
    default: return;
  }
  uint8_t buf[2] = { EV_TRIGGER, action };
  bleNotify(buf, 2);
}

// A gesture is only interesting if some control asked for it, on this channel.
static void fireGesture(uint8_t ch, uint8_t gesture) {
  for (int a = ACT_SCROLL_DOWN; a < ACTION_COUNT; a++) {
    if (triggerFor[a].channel == ch && triggerFor[a].gesture == gesture) runAction(a);
  }
}

// A tap can only be told apart from a hold once the muscle relaxes, so the tap
// lands on release. A hold repeats at the tap cadence while it lasts.
static void serviceClench(uint8_t ch, uint32_t now) {
  ChannelState& s = channel[ch];
  const ChannelTuning& t = tuning[ch];

  if (!s.held) {
    if (s.muscleLevel > (int)t.muscleThreshold && (now - s.lastClenchMs) >= TRIGGER_DEBOUNCE) {
      s.held      = true;
      s.repeating = false;
      s.startMs   = now;
    }
  } else if (s.muscleLevel < (int)t.muscleRelease) {
    // hysteresis, the muscle has to relax before the next clench counts
    s.held         = false;
    s.lastClenchMs = now;
    if (!s.repeating) fireGesture(ch, GEST_CLENCH);
    s.repeating = false;
  } else if (!s.repeating && (now - s.startMs) >= CLENCH_HOLD_MS) {
    s.repeating    = true;
    s.lastRepeatMs = now;
    fireGesture(ch, GEST_CLENCH_HOLD);
  } else if (s.repeating && (now - s.lastRepeatMs) >= CLENCH_REPEAT_MS) {
    s.lastRepeatMs = now;
    fireGesture(ch, GEST_CLENCH_HOLD);
  }
}

// Blinks are counted into a burst. The burst is only classified once the eyes
// have been still for BLINK_GAP_MS, because a double blink and the first two
// thirds of a triple look identical until then. That is also why any blink
// driven action lands about half a second after the last blink.
static void serviceBlink(uint8_t ch, uint32_t now) {
  ChannelState& s = channel[ch];
  const float threshold = tuning[ch].blinkThreshold;

  if (s.blinkLevel > threshold) {
    if (s.blinkArmed && (now - s.lastBlinkMs) >= BLINK_DEBOUNCE_MS) {
      s.blinkArmed  = false;
      s.lastBlinkMs = now;
      if (s.blinkRun < 255) s.blinkRun++;
    }
  } else if (s.blinkLevel < threshold * BLINK_RELEASE) {
    s.blinkArmed = true;
  }

  if (s.blinkRun > 0 && (now - s.lastBlinkMs) >= BLINK_GAP_MS) {
    uint8_t count = s.blinkRun;
    s.blinkRun = 0;

    uint8_t buf[3] = { EV_BLINK, ch, count };
    bleNotify(buf, sizeof(buf));
    if (count == 2)      fireGesture(ch, GEST_DOUBLE_BLINK);
    else if (count >= 3) fireGesture(ch, GEST_TRIPLE_BLINK);
    // a single blink is reported for the live bar and nothing else
  }
}

// Focus is the beta share of total EEG power. Both a clench and a blink flood
// that band, so it is ignored around either one.
static void serviceFocus(uint8_t ch, uint32_t now) {
  ChannelState& s = channel[ch];

  bool noise = s.held || (now - s.lastClenchMs) < CLENCH_BLOCK_MS ||
               s.blinkRun > 0 || (now - s.lastBlinkMs) < BLINK_BLOCK_MS;
  if (noise) return;
  if (s.beta.value() <= tuning[ch].focusThreshold) return;
  if ((now - s.lastFocusMs) < FOCUS_DEBOUNCE_MS) return;

  s.lastFocusMs = now;
  fireGesture(ch, GEST_FOCUS);
}

// The level a control is watching, so the app can draw one bar per control
// without being told which signal that is.
static uint16_t levelForBind(const Bind& b) {
  if (b.channel >= MAX_BIOAMP_CHANNELS) return 0;
  const ChannelState& s = channel[b.channel];
  switch (b.gesture) {
    case GEST_CLENCH:
    case GEST_CLENCH_HOLD:   return (uint16_t)constrain(s.muscleLevel, 0, 65535);
    case GEST_FOCUS:         return (uint16_t)constrain((int)(s.beta.value() * 10.0f), 0, 65535);
    case GEST_DOUBLE_BLINK:
    case GEST_TRIPLE_BLINK:  return (uint16_t)constrain((int)s.blinkLevel, 0, 65535);
  }
  return 0;
}

// Live levels for the bars in the app.
//
// The notify queue is only a few packets deep and a list burst already fills
// it, so a steady stream on top of one costs list entries. The stream is the
// only thing here that can be dropped without anyone noticing, so it yields to
// everything else: list and config streaming, an armed receiver, a fresh send.
static void serviceStream(uint32_t now) {
  if (!bleConnected || recording) return;
  if (listCursor >= 0 || profileCursor >= 0 || configCursor >= 0) return;
  if (now - lastCmdSentMs < STREAM_HOLDOFF_MS) return;
  if (now - lastStreamMs < STREAM_MS) return;
  lastStreamMs = now;

  uint16_t v[4] = {
    levelForBind(triggerFor[ACT_SCROLL_DOWN]),
    levelForBind(triggerFor[ACT_SCROLL_UP]),
    levelForBind(triggerFor[ACT_FIRE]),
    levelForBind(triggerFor[ACT_HOME]),
  };

  uint8_t buf[9];
  buf[0] = EV_STREAM;
  for (int i = 0; i < 4; i++) {
    buf[1 + i * 2] = v[i] & 0xFF;
    buf[2 + i * 2] = v[i] >> 8;
  }
  bleNotify(buf, sizeof(buf));
}

void bioampService() {
  if (!bioampReady) return;

  uint32_t now = millis();
  // The gap that matters is how long we were away, not how long we spent
  // working. lastServiceMs is stamped at the end of this function for that
  // reason: the beta transform alone runs for a good few milliseconds on a
  // chip with no floating point unit, and timing it as if it were a stall
  // would reset the filters once every window and spike every level.
  if (lastServiceMs && (now - lastServiceMs) > STALL_GAP_MS) {
    bioampReset();
  }

  if (xSemaphoreTake(adcSem, 0) == pdTRUE) {
    uint8_t frame[ADC_FRAME_BYTES];
    uint32_t len = 0;
    while (adc_continuous_read(adcHandle, frame, sizeof(frame), &len, 0) == ESP_OK && len) {
      for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= len; i += SOC_ADC_DIGI_RESULT_BYTES) {
        auto* p = (const adc_digi_output_data_t*)&frame[i];
        uint8_t hw = p->type2.channel;
        if (hw >= sizeof(adcChannelIndex)) continue;
        int8_t slot = adcChannelIndex[hw];
        if (slot < 0) continue;

        if (slot == (int8_t)batterySlot) {
          battWinSum += p->type2.data;
          battWinCount++;
          continue;
        }

        uint8_t ch = activeChannel[slot];
        ChannelState& s = channel[ch];
        float notched = s.notch.process(fixRaw(p->type2.data));

        switch (channelFilter[ch]) {
          case FILT_EMG:
            s.muscleLevel = s.emgEnv.process(abs((int)s.emg.process(notched)));
            break;

          case FILT_EEG: {
            s.muscleLevel = s.emgEnv.process(abs((int)s.emg.process(notched)));
            float brain = s.eeg.process(notched);
            s.beta.push(brain);
            s.blinkLevel = s.blinkEnv.process(s.eog.process(brain));
            break;
          }

          case FILT_EOG:
            s.blinkLevel = s.blinkEnv.process(s.eog.process(s.eeg.process(notched)));
            break;
        }
      }
    }
  }

  // Triggers are a user interface, not a signal path, so they are suspended
  // while the app is editing a profile or the receiver is armed.
  if (now >= settleUntil && !editMode && !recording) {
    for (int i = 0; i < activeCount; i++) {
      uint8_t ch = activeChannel[i];
      switch (channelFilter[ch]) {
        case FILT_EMG:
          serviceClench(ch, now);
          break;
        case FILT_EEG:
          serviceClench(ch, now);
          serviceBlink(ch, now);
          serviceFocus(ch, now);
          break;
        case FILT_EOG:
          serviceBlink(ch, now);
          break;
      }
    }
  }
  serviceStream(now);
  lastServiceMs = millis();      // stamped last, so our own work is not a gap
}

// Averaged battery reading in the same units the LED code expects.
// Returns 0 when no samples have arrived yet.
uint16_t bioampBatteryReading() {
  if (battWinCount == 0) return 0;
  uint16_t avg = (uint16_t)(battWinSum / battWinCount);
  battWinSum   = 0;
  battWinCount = 0;
  return avg;
}
#endif  // BIOAMP_ENABLED

// Which playmate this is, and so how many BioAmp channels exist.
//
// Proto leaves the motor and buzzer pins floating, so both read high through
// their pull-ups. Vibz drives them, so one reads low. Vibz Plus is a Vibz that
// also pulls one of A3 to A5 low. Every pin is put back before the ADC claims
// them. Same probe as the other NPG Lite firmware.
#define MOTOR_PIN  7
#define BUZZER_PIN 8

void detectPlaymate() {
  pinMode(MOTOR_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, INPUT_PULLUP);

  if (digitalRead(MOTOR_PIN) == HIGH && digitalRead(BUZZER_PIN) == HIGH) {
    playmate = PLAYMATE_PROTO;
  } else {
    playmate = PLAYMATE_VIBZ;
    pinMode(A3, INPUT_PULLUP);
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);
    uint32_t start = millis();
    while (millis() - start < 100) {
      if (digitalRead(A3) == LOW || digitalRead(A4) == LOW || digitalRead(A5) == LOW) {
        playmate = PLAYMATE_VIBZ_PLUS;
        break;
      }
    }
    // back to high impedance before the ADC uses them
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
  }

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  availableChannels = (playmate == PLAYMATE_VIBZ_PLUS) ? 6 : 3;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[BOOT] IR Transceiver");

  detectPlaymate();
  tuningDefaults();

  pinMode(USER_BTN_PIN, INPUT_PULLUP);
  irsend.begin();

#if !BIOAMP_ENABLED
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
#endif
  pixel.begin();
  pixel.clear();
  pixel.show();

  int currentBattery = getCurrentBatteryPercentage();
  batteryColor = batteryPercentToColor(currentBattery);
  if (!LittleFS.begin(true)) Serial.println("[FS] mount failed");

  updateStatusLeds(false);   // battery on, bluetooth red
  lastBatteryCheck = millis();         // the battery was just read above

  checkStoreVersion();
  nvsLoad();

  BLEDevice::init(BLE_NAME);
  BLEServer* srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());

  BLEService* svc = srv->createService(SVC_UUID);
  pNotify = svc->createCharacteristic(NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pNotify->addDescriptor(new BLE2902());

  BLECharacteristic* pWrite = svc->createCharacteristic(
      WRITE_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pWrite->setCallbacks(new WriteCB());

  svc->start();
  srv->getAdvertising()->start();
#if BIOAMP_ENABLED
  if (!bioampBegin()) Serial.println("[BIO] sampling failed to start");
#endif
}

// HELD means the hold threshold was reached while the button is still down,
// so the release that follows must not be treated as a short press.
enum BtnState { IDLE, PRESSED, HELD };
BtnState btnState    = IDLE;
uint32_t btnPressed  = 0;
uint32_t lastRelease = 0;

void loop() {
  uint32_t now = millis();

#if BIOAMP_ENABLED
  bioampService();
#else
  if (now - lastBatterySample >= BATTERY_SAMPLE_MS) {
    lastBatterySample = now;
    batteryWinSum += analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;
  }
#endif
  if (now - lastBatteryCheck >= BATTERY_CHECK_MS) {
    lastBatteryCheck = now;
    batteryColor = batteryPercentToColor(getCurrentBatteryPercentage());
  }
  // show() briefly disables interrupts, which can cost the receiver an edge,
  // so the ring is left alone while it is armed
  if (!recording) updateStatusLeds(bleConnected);

  // Deferred work from the BLE callback and from the triggers
  if (reqFire >= 0) {
    int id = reqFire;
    reqFire = -1;
    bleNotify1(fireCommand(id) ? EV_OK : EV_FAIL);
  }
  if (reqOpen >= 0) {
    int p = reqOpen;
    reqOpen = -1;
    openProfileAt(p);
  }
  if (reqHome) {
    reqHome = false;
    goHome();
  }
  if (reqWipe) {
    reqWipe = false;
    wipeAll();
    bleNotify1(EV_WIPED);
    reqProfiles = true;
  }
  if (reqCancel) {
    reqCancel = false;
    stopRecording(2);
  }
  if (reqRecord) {
    reqRecord = false;
    startRecording();
  }
#if BIOAMP_ENABLED
  if (reqResample) {
    reqResample = false;
    bioampRestart();
    reqConfig = true;      // confirm what actually took effect
  }
#endif
  // connectedAt is stamped by the Bluetooth task, so it can land after the now
  // above. Same wrap trap as the record timeout, so read the clock again.
  if (bleConnected && (millis() - connectedAt) >= LIST_DELAY_MS) {
    if (reqConfig) {
      reqConfig = false;
      configCursor = 0;
      lastListSend = now - LIST_PACE_MS;
    } else if (reqProfiles) {
      reqProfiles = false;
      startProfileList();
    } else if (reqList) {
      reqList = false;
      startList();
    }
  }
  serviceConfig(now);
  serviceList(now);

  bool btnLow = (digitalRead(USER_BTN_PIN) == LOW);

  switch (btnState) {
    case IDLE:
      if (btnLow && (now - lastRelease) >= DEBOUNCE_MS) {
        btnState   = PRESSED;
        btnPressed = now;
      }
      break;

    case PRESSED:
      if (!btnLow) {
        lastRelease = now;
        btnState    = IDLE;
        // short press: cancel a recording if one is running, otherwise act on
        // whatever is highlighted
        if (recording) stopRecording(1);
        else if (screen == SCREEN_HOME) reqOpen = homeCursor;
      } else if ((now - btnPressed) >= HOLD_THRESHOLD) {
        btnState = HELD;
        startRecording();
      }
      break;

    case HELD:
      // recording stays latched once the button comes back up
      if (!btnLow) {
        lastRelease = now;
        btnState    = IDLE;
      }
      break;
  }

  // Fresh reading on purpose. Recording can have started later in this same
  // pass, which puts recordAt after the now taken at the top, and the
  // unsigned subtraction would then wrap to a huge elapsed time and time the
  // capture out before it began.
  if (recording && (millis() - recordAt) >= RECORD_TIMEOUT) stopRecording(0);

  if (recording && irrecv.decode(&pendingResult)) {
    irrecv.disableIRIn();
    recording = false;
#if BIOAMP_ENABLED
    bioampResume();
#endif

    // rawbuf points into the receiver, copy it out before it is reused
    uint16_t len = min(pendingResult.rawlen, (uint16_t)RAW_BUF_LEN);
    for (uint16_t i = 0; i < len; i++) pendingRaw[i] = pendingResult.rawbuf[i];
    pendingResult.rawbuf = pendingRaw;
    pendingResult.rawlen = len;
    pendingValid = true;

    int dupId = findDuplicate(openProfile, &pendingResult);
    notifyCapture(dupId);
  }
}
