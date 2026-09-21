/*
  NPG Lite (ESP32-C6) - Transmitting Board
  ---------------------------------------------------------------------
  Channel A0: Horizontal EOG -> Momentary (non-latched) Turn
              - A confirmed spike left/right immediately sends a
                PIVOT command.
              - The pivot is held for PIVOT_HOLD_MS, then the board
                automatically returns to STRAIGHT and resumes whatever
                drive command was active (forward/reverse/stopped).
              - After any trigger, PIVOT_DEBOUNCE_MS must pass before
                a new trigger (either direction) can fire again.
              - A short blink-artifact guard after any confirmed blink
                ignores EOG readings, so an eye-blink doesn't get
                misread as a horizontal spike.
              This mirrors the debounce + timed-hold approach used in
              the gaming (keyboard) firmware - no latch/rearm/swap-guard
              state machine.
  Channel A1: EEG Signal Processing ->
              - Double Blink  = Toggle Drive Mode (Forward / Reverse)
                                 First double blink from rest -> FORWARD.
              - Triple Blink  = Emergency Stop / Brake
              (Single blink has no dedicated action anymore, since
              steering is momentary and there's no latch to break.)

  REST STATE:
    On BLE connect AND on BLE disconnect, the board resets to rest
    (steer straight, drive stopped) - same as an emergency stop. So
    every time the car connects, it's sitting still, and the first
    double blink you do will always start it moving FORWARD.

  CALIBRATION PACING (announce-then-measure):
    Each timed calibration step (straight/right/left/blink) is sent to
    the calibrator UI with "awaitGo": true, and the board then freezes
    on that step - it does not start its hold timer or sample any
    peaks/envelope - until the UI writes "GO" to the control
    characteristic. The UI sends "GO" only after it has finished
    speaking the voice prompt for that step, so nothing is measured
    while the announcement is still playing.

  BLE Commands Sent:
    0 = STOP / EMERGENCY BRAKE
    1 = PIVOT LEFT   (EOG Left)
    2 = PIVOT RIGHT  (EOG Right)
    3 = DRIVE FORWARD
    4 = DRIVE REVERSE

  BLE Control Writes Received (calibrator control characteristic):
    "CAL" / "CALIBRATE" = start a full recalibration cycle
    "GO"                = release the current step's freeze and start
                          its hold/measurement timer now

  ---------------------------------------------------------------------
  FIX NOTES (MAC address handling):
    1. BLEDevice::setMTU(247) requested after init() so the longer
       status payloads (which include mac) aren't truncated by the
       default 23-byte ATT MTU. Falls back automatically if the
       central can't support it - connection is unaffected.
    2. "mac" is placed immediately after "phase" in the "straight" and
       "ready" JSON payloads, so a worst-case truncation fails loudly
       (invalid JSON) instead of silently dropping just the mac field.
    3. bleAuxChar is seeded with an initial JSON value containing the
       mac address as soon as the server boots (before any client
       connects). This lets the calibrator UI simply READ the
       characteristic right after connecting and get the MAC
       immediately, instead of waiting for a calibration-phase
       notification.
  ---------------------------------------------------------------------
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ---- BLE server includes ----
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"

enum TurnState { STEER_STRAIGHT, STEER_LEFT, STEER_RIGHT };
enum DriveState { DRIVE_STOPPED, DRIVE_FORWARD, DRIVE_REVERSE };
enum EOGCalibPhase { EOGCALIB_WAITING, EOGCALIB_SETTLING, EOGCALIB_RIGHT, EOGCALIB_LEFT, EOGCALIB_COMPLETE };
enum EEGCalibPhase { EEGCALIB_WAITING, EEGCALIB_BLINK, EEGCALIB_DONE };

#define BAUD_RATE 115200
#define PIXEL_PIN 15
#define PIXEL_COUNT 6
#define BLE_CONNECTED_LED_INDEX 2  // Dedicated LED for BLE Connection Status

Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// =====================================================================
// ==========================   BLE SERVER   ===========================
// =====================================================================
#define BLE_DEVICE_NAME "NPG Lite"

static BLEUUID bleServiceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID bleCharUUID_1 ("beb5483e-36e1-4688-b7f5-ea07361b26a8"); // drive/pivot commands (NOTIFY)
static BLEUUID bleCharUUID_2 ("1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"); // calibration status JSON (NOTIFY)
static BLEUUID bleCharUUID_3 ("3c9a1235-2f9e-4d34-9f0e-6a1b6d0c9a11"); // calibrator control (WRITE)

BLEServer* bleServer = nullptr;
BLECharacteristic* bleCmdChar = nullptr;
BLECharacteristic* bleAuxChar = nullptr;
BLECharacteristic* bleCtrlChar = nullptr;
bool bleClientConnected = false;
String boardMacAddress = "";

void resetToRestState(); // fwd decl - defined after steering/drive state globals
void beginEOGCalibration(); // fwd decl - defined after steering/drive state globals
void sendCalStatus(const String& json); // fwd decl - defined after steering/drive state globals
void handleGoSignal(); // fwd decl - defined after EOG/EEG calibration globals

class BLEServerCallbacksImpl : public BLEServerCallbacks {
  void onConnect(BLEServer* srv) {
    bleClientConnected = true;
    Serial.println("[BLE] Car connected.");
    
    // Turn ON designated connection LED (Cyan)
    pixels.setPixelColor(BLE_CONNECTED_LED_INDEX, pixels.Color(0, 20, 20));
    pixels.show();

    resetToRestState(); // fresh connection always starts at rest
  }
  void onDisconnect(BLEServer* srv) {
    bleClientConnected = false;
    Serial.println("[BLE] Car disconnected. Restarting advertising...");
    
    // Turn OFF designated connection LED
    pixels.setPixelColor(BLE_CONNECTED_LED_INDEX, pixels.Color(0, 0, 0));
    pixels.show();

    resetToRestState(); // treat a lost connection like an emergency stop
    BLEDevice::startAdvertising();
  }
};

// Triggers a full recalibration (EOG + EEG), same reset the 'c' serial
// command performs. Shared by both the Serial handler and the BLE
// control characteristic so the calibrator UI can kick this off too.
// (Defined later, once the EOG/EEG calibration globals exist.)
void startCalibrationCycle();

class BLECtrlCallbacksImpl : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) {
    String value = ch->getValue().c_str();
    value.trim();
    value.toUpperCase();
    if (value == "CAL" || value == "CALIBRATE") {
      Serial.println("\n[BLE] Calibration triggered from calibrator UI.");
      startCalibrationCycle();
    } else if (value == "GO") {
      handleGoSignal();
    }
  }
};

void setupBLEServer() {
  BLEDevice::init(BLE_DEVICE_NAME);

  // Request a larger ATT MTU (default is only 23 bytes / 20 usable) so
  // status JSON payloads that include the mac field aren't silently
  // truncated by the BLE stack. This is a request only - if the
  // connecting device can't support it, the stack falls back
  // automatically and the connection is unaffected.
  BLEDevice::setMTU(247);

  boardMacAddress = BLEDevice::getAddress().toString().c_str();
  Serial.print("[BLE] Board MAC: ");
  Serial.println(boardMacAddress);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BLEServerCallbacksImpl());

  BLEService* svc = bleServer->createService(bleServiceUUID);

  bleCmdChar = svc->createCharacteristic(
    bleCharUUID_1,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleCmdChar->addDescriptor(new BLE2902());

  bleAuxChar = svc->createCharacteristic(
    bleCharUUID_2,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleAuxChar->addDescriptor(new BLE2902());

  // Seed the characteristic with a small JSON blob containing the MAC
  // address as soon as the server boots, so a client that simply READS
  // this characteristic right after connecting - before any
  // calibration has run - immediately gets the MAC.
  String initialStatus = String("{\"phase\":\"connected\",\"mac\":\"") + boardMacAddress + "\"}";
  bleAuxChar->setValue((uint8_t*)initialStatus.c_str(), initialStatus.length());

  bleCtrlChar = svc->createCharacteristic(
    bleCharUUID_3,
    BLECharacteristic::PROPERTY_WRITE
  );
  bleCtrlChar->setCallbacks(new BLECtrlCallbacksImpl());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(bleServiceUUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started. Waiting for car...");
}

// Pushes a small JSON status blob to the calibrator UI over the aux
// (notify) characteristic. Kept deliberately tiny/flat so it parses
// with a single JSON.parse() on the browser side.
void sendCalStatus(const String& json) {
  if (bleAuxChar == nullptr) return;
  bleAuxChar->setValue((uint8_t*)json.c_str(), json.length());
  bleAuxChar->notify();
  Serial.print("[BLE->UI] ");
  Serial.println(json);
}

void sendBLECommand(uint32_t cmd) {
  if (!bleClientConnected || bleCmdChar == nullptr) return;
  uint8_t payload[4];
  payload[0] = (uint8_t)(cmd & 0xFF);
  payload[1] = (uint8_t)((cmd >> 8) & 0xFF);
  payload[2] = (uint8_t)((cmd >> 16) & 0xFF);
  payload[3] = (uint8_t)((cmd >> 24) & 0xFF);
  bleCmdChar->setValue(payload, 4);
  bleCmdChar->notify();
}

// =====================================================================
// ===================   CHANNEL 1: HORIZONTAL EOG (A0)  ================
// =====================================================================

const int EOG_PIN = A0;

const unsigned long EOG_SAMPLE_INTERVAL_MS = 5; // ~200 Hz
unsigned long eogLastSampleTime = 0;

float eogBaseline = 0;
const float EOG_BASELINE_ALPHA = 0.002;
float eogSmoothed = 0;
const float EOG_SMOOTH_ALPHA = 0.3;
bool eogBaselineInitialized = false;

EOGCalibPhase eogCalibPhase = EOGCALIB_WAITING;
unsigned long eogCalibPhaseStartTime = 0;

// Hold time for each of the straight/right/left calibration steps.
// Reused both in the elapsed-time checks below AND in the "durationMs"
// field sent to the calibrator UI, so the on-screen countdown can never
// drift out of sync with the board's actual timing.
const unsigned long EOG_STEP_HOLD_MS = 4000;

// While true, the current EOG calibration step (straight/right/left) is
// frozen: no hold-timer elapsed check and no peak sampling happens.
// Cleared by handleGoSignal() when the UI's "GO" write arrives, right
// after it finishes speaking that step's announcement.
bool awaitingGoEOG = false;

float POSITIVE_THRESHOLD = 0.0;
float NEGATIVE_THRESHOLD = 0.0;

float maxPositivePeak = 0.0;
float maxNegativePeak = 0.0;

unsigned long eogBlinkGuardUntil = 0;
const unsigned long EOG_BLINK_GUARD_MS = 400;

unsigned long eegBlinkGuardUntil = 0;
const unsigned long EEG_GUARD_AFTER_EOG_MS = 350;

// ---- Momentary pivot control (debounce + timed hold, no latch) ----
const unsigned long PIVOT_DEBOUNCE_MS = 800; // min gap between new pivot triggers
const unsigned long PIVOT_HOLD_MS     = 800; // how long a pivot command is asserted

unsigned long lastPivotTriggerTime = 0;
bool pivotPending = false;
unsigned long pivotReleaseTime = 0;

TurnState currentSteerState = STEER_STRAIGHT;
DriveState currentDriveState = DRIVE_STOPPED;

extern EEGCalibPhase eegCalibPhase;
extern bool awaitingGoBlink;
extern unsigned long blinkCalibStartTime;
extern const unsigned long BLINK_CALIB_MS;

void resumeDriveCommand() {
  switch (currentDriveState) {
    case DRIVE_FORWARD:
      Serial.println("--- Resuming FORWARD ---");
      pixels.setPixelColor(1, pixels.Color(0, 20, 0)); // Green
      pixels.show();
      sendBLECommand(3);
      break;
    case DRIVE_REVERSE:
      Serial.println("--- Resuming REVERSE ---");
      pixels.setPixelColor(1, pixels.Color(20, 10, 0)); // Yellow
      pixels.show();
      sendBLECommand(4);
      break;
    case DRIVE_STOPPED:
    default:
      sendBLECommand(0);
      break;
  }
}

void triggerPivot(TurnState direction) {
  currentSteerState = direction;
  lastPivotTriggerTime = millis();
  eegBlinkGuardUntil = millis() + EEG_GUARD_AFTER_EOG_MS; // guard blink detector from EOG crosstalk

  if (direction == STEER_LEFT) {
    Serial.println("<<< PIVOT: LEFT <<<");
    pixels.setPixelColor(0, pixels.Color(0, 0, 20)); // Blue LED
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.show();
    sendBLECommand(1); // Command 1 = Pivot Left
  } else if (direction == STEER_RIGHT) {
    Serial.println(">>> PIVOT: RIGHT >>>");
    pixels.setPixelColor(5, pixels.Color(0, 0, 20)); // Blue LED
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    sendBLECommand(2); // Command 2 = Pivot Right
  }

  pivotPending = true;
  pivotReleaseTime = millis() + PIVOT_HOLD_MS;
}

void releasePivotIfDue() {
  if (!pivotPending) return;
  if ((long)(millis() - pivotReleaseTime) < 0) return;

  pivotPending = false;
  currentSteerState = STEER_STRAIGHT;

  Serial.println("--- PIVOT RELEASED -> STRAIGHT ---");
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(5, pixels.Color(0, 0, 0));
  pixels.show();

  resumeDriveCommand();
}

void resetToRestState() {
  currentSteerState = STEER_STRAIGHT;
  currentDriveState = DRIVE_STOPPED;
  pivotPending = false;
  
  // Clear non-connection pixels
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  pixels.setPixelColor(5, pixels.Color(0, 0, 0));

  // Maintain BLE LED status
  if (bleClientConnected) {
    pixels.setPixelColor(BLE_CONNECTED_LED_INDEX, pixels.Color(0, 20, 20));
  } else {
    pixels.setPixelColor(BLE_CONNECTED_LED_INDEX, pixels.Color(0, 0, 0));
  }

  pixels.show();
  sendBLECommand(0); // no-op if not connected, harmless if it is
}

void beginEOGCalibration() {
  currentSteerState = STEER_STRAIGHT;
  currentDriveState = DRIVE_STOPPED;
  pivotPending = false;
  lastPivotTriggerTime = 0;
  eogBlinkGuardUntil = 0;
  eegBlinkGuardUntil = 0;
  
  resetToRestState(); // Keeps BLE indicator while clearing other driving pixels

  Serial.println("\n==================================================");
  Serial.println("       STARTING COMBINED CALIBRATION (EOG + EEG)  ");
  Serial.println("==================================================");
  Serial.println("PART 1: EOG (eye movement) calibration");
  Serial.println("\n--> STEP 1: Look STRAIGHT ahead - announcing, awaiting GO...");
  eogCalibPhase = EOGCALIB_SETTLING;
  awaitingGoEOG = true; // frozen until the UI finishes speaking and sends "GO"

  // "mac" is placed immediately after "phase" (rather than at the end)
  // so that if this longer payload ever gets truncated by the BLE
  // stack, the resulting string fails JSON.parse() loudly instead of
  // silently just dropping the trailing mac field.
  sendCalStatus(String("{\"phase\":\"straight\",\"mac\":\"") + boardMacAddress +
                "\",\"durationMs\":" + EOG_STEP_HOLD_MS + ",\"awaitGo\":true}");
}

// Called when the calibrator UI writes "GO" - releases whichever step
// (EOG or blink) is currently frozen waiting on an announcement, and
// starts its hold/measurement timer from this exact moment.
void handleGoSignal() {
  if (eogCalibPhase != EOGCALIB_COMPLETE && eogCalibPhase != EOGCALIB_WAITING && awaitingGoEOG) {
    awaitingGoEOG = false;
    eogCalibPhaseStartTime = millis();
    Serial.println("[BLE] GO received - EOG step timer started.");
    return;
  }
  if (eegCalibPhase == EEGCALIB_BLINK && awaitingGoBlink) {
    awaitingGoBlink = false;
    blinkCalibStartTime = 0; // processBlinkLogic() will stamp this on its next sample
    Serial.println("[BLE] GO received - blink step timer started.");
  }
}

void handleEOG() {
  unsigned long now = millis();
  if (now - eogLastSampleTime < EOG_SAMPLE_INTERVAL_MS) return;
  eogLastSampleTime = now;

  int rawADC = analogRead(EOG_PIN);
  float rawVoltage = rawADC * (3.3 / 4095.0);

  if (!eogBaselineInitialized) {
    eogBaseline = rawVoltage;
    eogSmoothed = 0.0;
    eogBaselineInitialized = true;
  }

  eogBaseline += EOG_BASELINE_ALPHA * (rawVoltage - eogBaseline);
  float highPassed = rawVoltage - eogBaseline;
  eogSmoothed += EOG_SMOOTH_ALPHA * (highPassed - eogSmoothed);

  if (eogCalibPhase == EOGCALIB_WAITING) return;

  if (eogCalibPhase != EOGCALIB_COMPLETE) {
    // Frozen on this step - announcement is still playing on the UI,
    // nothing is measured and the hold timer hasn't started yet.
    if (awaitingGoEOG) return;

    unsigned long elapsed = now - eogCalibPhaseStartTime;

    switch (eogCalibPhase) {
      case EOGCALIB_WAITING:
        break;

      case EOGCALIB_SETTLING:
        if (elapsed >= EOG_STEP_HOLD_MS) {
          eogCalibPhase = EOGCALIB_RIGHT;
          maxPositivePeak = 0.0;
          maxNegativePeak = 0.0;
          awaitingGoEOG = true; // freeze again for the next announcement
          Serial.println("\n--> STEP 2: Look RIGHT - announcing, awaiting GO...");
          sendCalStatus(String("{\"phase\":\"right\",\"durationMs\":") + EOG_STEP_HOLD_MS + ",\"awaitGo\":true}");
        }
        break;

      case EOGCALIB_RIGHT:
        if (eogSmoothed > maxPositivePeak) maxPositivePeak = eogSmoothed;
        if (eogSmoothed < maxNegativePeak) maxNegativePeak = eogSmoothed;
        if (elapsed >= EOG_STEP_HOLD_MS) {
          eogCalibPhase = EOGCALIB_LEFT;
          awaitingGoEOG = true; // freeze again for the next announcement
          Serial.println("\n--> STEP 3: Look LEFT - announcing, awaiting GO...");
          sendCalStatus(String("{\"phase\":\"left\",\"durationMs\":") + EOG_STEP_HOLD_MS + ",\"awaitGo\":true}");
        }
        break;

      case EOGCALIB_LEFT:
        if (eogSmoothed > maxPositivePeak) maxPositivePeak = eogSmoothed;
        if (eogSmoothed < maxNegativePeak) maxNegativePeak = eogSmoothed;
        if (elapsed >= EOG_STEP_HOLD_MS) {
          POSITIVE_THRESHOLD = maxPositivePeak * 0.50;
          NEGATIVE_THRESHOLD = maxNegativePeak * 0.50;

          eogCalibPhase = EOGCALIB_COMPLETE;

          Serial.println("\n==================================================");
          Serial.println("     EOG CALIBRATION COMPLETE                     ");
          Serial.println("==================================================");
          Serial.print("Positive Threshold (+): "); Serial.println(POSITIVE_THRESHOLD, 4);
          Serial.print("Negative Threshold (-): "); Serial.println(NEGATIVE_THRESHOLD, 4);
          Serial.println("--------------------------------------------------\n");

          {
            String msg = "{\"phase\":\"eog_done\",\"posTh\":" + String(POSITIVE_THRESHOLD, 4) +
                         ",\"negTh\":" + String(NEGATIVE_THRESHOLD, 4) + "}";
            sendCalStatus(msg);
          }

          Serial.println("PART 2: EEG Blink calibration");
          Serial.println("STEP 1: BLINK 2-3 TIMES normally - announcing, awaiting GO...");
          eegCalibPhase = EEGCALIB_BLINK;
          awaitingGoBlink = true; // frozen until the UI finishes speaking and sends "GO"
          sendCalStatus(String("{\"phase\":\"blink\",\"durationMs\":") + BLINK_CALIB_MS + ",\"awaitGo\":true}");
        }
        break;

      case EOGCALIB_COMPLETE:
        break;
    }
    return;
  }

  // --- Always service a pending pivot release first (mirrors the
  //     gaming firmware's "handle pending key release" priority) ---
  releasePivotIfDue();

  // --- BLINK-ARTIFACT GUARD ---
  // Ignore new triggers for a short window right after a confirmed
  // blink, so blink artifacts on this channel don't get misread as
  // a horizontal spike.
  if (now < eogBlinkGuardUntil) return;

  // --- DEBOUNCE GATE ---
  if (now - lastPivotTriggerTime < PIVOT_DEBOUNCE_MS) return;

  // --- SPIKE EVALUATION (momentary, no latch) ---
  if (eogSmoothed > POSITIVE_THRESHOLD) {
    triggerPivot(STEER_LEFT);
  } else if (eogSmoothed < NEGATIVE_THRESHOLD) {
    triggerPivot(STEER_RIGHT);
  }
}

// =====================================================================
// ============   CHANNEL 2: EEG BLINK DETECTOR (DSP ENVELOPE)  =======
// =====================================================================

#define SAMPLE_RATE 512
#define EEG_PIN A1

#define ENVELOPE_WINDOW_MS 100
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

float highpass(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.91327599f * z1 - 0.91688335f * z2;
    output = 0.95753983f * x + -1.91507967f * z1 + 0.95753983f * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

float Notch(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.58696045f * z1 - 0.96505858f * z2;
    output = 0.96588529f * x + -1.57986211f * z1 + 0.96588529f * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2;
    float x = output - -1.62761184f * z1 - 0.96671306f * z2;
    output = 1.00000000f * x + -1.63566226f * z1 + 1.00000000f * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

float EEGFilter(float input) {
  float output = input;
  {
    static float z1, z2;
    float x = output - -1.24200128f * z1 - 0.45885207f * z2;
    output = 0.05421270f * x + 0.10842539f * z1 + 0.05421270f * z2;
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

// ---- Blink Detection Parameters ----
const unsigned long BLINK_CALIB_MS = 5000;
EEGCalibPhase eegCalibPhase = EEGCALIB_WAITING;
unsigned long blinkCalibStartTime = 0;
float blinkCalibPeak = 0;

// While true, the blink calibration step is frozen: no envelope
// sampling and no start-timer stamp happens. Cleared by
// handleGoSignal() once the UI finishes speaking the blink prompt.
bool awaitingGoBlink = false;

float BlinkThreshold = 50.0f;
const unsigned long BLINK_DEBOUNCE_MS = 250;
const unsigned long DOUBLE_BLINK_MS   = 800;
const unsigned long TRIPLE_BLINK_MS   = 600;

unsigned long lastBlinkTime   = 0;
unsigned long firstBlinkTime  = 0;
unsigned long secondBlinkTime = 0;
int blinkCount = 0;

bool blinkArmed = true;
const float BLINK_REARM_RATIO = 0.45f;

void processBlinkLogic() {
  unsigned long nowMs = millis();

  if (eegCalibPhase == EEGCALIB_BLINK) {
    // Frozen - announcement still playing, don't sample or start the
    // hold timer until the UI's "GO" write arrives.
    if (awaitingGoBlink) return;

    if (blinkCalibStartTime == 0) blinkCalibStartTime = nowMs;
    if (currentEEGEnvelope > blinkCalibPeak) blinkCalibPeak = currentEEGEnvelope;

    if (nowMs - blinkCalibStartTime >= BLINK_CALIB_MS) {
      BlinkThreshold = blinkCalibPeak * 0.55f;
      if (BlinkThreshold < 20.0f) BlinkThreshold = 20.0f;

      eegCalibPhase = EEGCALIB_DONE;
      blinkArmed = true;

      Serial.println("\n==================================================");
      Serial.println("   ALL CALIBRATION COMPLETE - SYSTEM READY        ");
      Serial.println("==================================================");
      Serial.print("Blink threshold set to: "); Serial.println(BlinkThreshold, 2);
      Serial.println("--------------------------------------------------");
      Serial.println("  EOG Left/Right  -> Momentary pivot (auto-returns straight)");
      Serial.println("  2 Blinks        -> Toggle Forward / Reverse");
      Serial.println("  3 Blinks        -> Emergency Stop");
      Serial.println("--------------------------------------------------\n");

      {
        // "mac" placed right after "phase" for the same truncation-
        // safety reason as the "straight" phase message above.
        String msg = "{\"phase\":\"ready\",\"mac\":\"" + boardMacAddress +
                     "\",\"posTh\":" + String(POSITIVE_THRESHOLD, 4) +
                     ",\"negTh\":" + String(NEGATIVE_THRESHOLD, 4) +
                     ",\"blinkTh\":" + String(BlinkThreshold, 2) + "}";
        sendCalStatus(msg);
      }
    }
    return;
  }

  if (eegCalibPhase != EEGCALIB_DONE) return;

  if (nowMs < eegBlinkGuardUntil) return;

  if (!blinkArmed) {
    if (currentEEGEnvelope < BlinkThreshold * BLINK_REARM_RATIO) {
      blinkArmed = true;
    }
  }

  if (blinkArmed && currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
    lastBlinkTime = nowMs;
    blinkArmed = false;
    eogBlinkGuardUntil = nowMs + EOG_BLINK_GUARD_MS;

    if (blinkCount == 0) {
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
      secondBlinkTime = nowMs;
      blinkCount = 2;
    }
    else if (blinkCount == 2 && (nowMs - secondBlinkTime) <= TRIPLE_BLINK_MS) {
      Serial.println("<<< TRIPLE BLINK: EMERGENCY STOP <<<");
      pixels.setPixelColor(1, pixels.Color(20, 0, 0)); // Red LED
      pixels.show();

      resetToRestState(); // steer straight, drive stopped, cancel pending pivot, send stop

      blinkCount = 0;
    }
    else {
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
  }

  // --- DOUBLE BLINK (Drive Mode Switch) ---
  // From rest (STOPPED, e.g. right after boot/(re)connect/emergency stop)
  // the first double blink always goes to FORWARD. After that it just
  // alternates FORWARD <-> REVERSE as before.
  if (blinkCount == 2 && (nowMs - secondBlinkTime) > TRIPLE_BLINK_MS) {
    if (currentDriveState != DRIVE_FORWARD) {
      currentDriveState = DRIVE_FORWARD;
      Serial.println(">>> DOUBLE BLINK: MODE -> FORWARD >>>");
    } else {
      currentDriveState = DRIVE_REVERSE;
      Serial.println(">>> DOUBLE BLINK: MODE -> REVERSE >>>");
    }

    if (currentSteerState == STEER_STRAIGHT) {
      resumeDriveCommand();
    } else {
      Serial.println("    (Currently pivoting - new drive mode resumes once pivot releases)");
    }

    blinkCount = 0;
  }

  // --- SINGLE BLINK ---
  // No dedicated action: steering is momentary now, so there's no
  // latch left to break. We still have to let the single-blink timing
  // window elapse so it isn't misread as the first half of a double
  // or triple blink.
  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
    blinkCount = 0;
  }
}

unsigned long eegLastMicros = 0;
long eegTimer = 0;

void handleEEGSampling() {
  unsigned long now = micros();
  unsigned long interval = now - eegLastMicros;
  eegLastMicros = now;

  eegTimer -= interval;
  if (eegTimer <= 0) {
    eegTimer += 1000000L / SAMPLE_RATE;

    int rawADC = analogRead(EEG_PIN);
    float filt = EEGFilter(Notch((float)rawADC));
    float filtered = highpass(filt);

    currentEEGEnvelope = updateEEGEnvelope(filtered);
    processBlinkLogic();
  }
}

// =====================================================================
// ==========================   SETUP / LOOP   =========================
// =====================================================================

const unsigned long AUTO_CALIB_DELAY_MS = 3000;
unsigned long bootTime = 0;
bool autoCalibTriggered = false;

// Triggers a full recalibration (EOG + EEG). Shared by the Serial
// handler and the BLE control characteristic (see BLECtrlCallbacksImpl
// above) so the browser-based calibrator UI can kick this off too.
void startCalibrationCycle() {
  eogCalibPhase = EOGCALIB_WAITING;
  eegCalibPhase = EEGCALIB_WAITING;
  awaitingGoEOG = false;
  awaitingGoBlink = false;
  blinkCalibStartTime = 0;
  blinkCalibPeak = 0;
  blinkCount = 0;
  lastBlinkTime = firstBlinkTime = secondBlinkTime = 0;
  blinkArmed = true;
  eogBlinkGuardUntil = 0;
  eegBlinkGuardUntil = 0;
  beginEOGCalibration();
}

void handleSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "c" || cmd == "cal" || cmd == "calibrate") {
    Serial.println("\n[CMD] Manual calibration triggered via Serial ('c').");
    startCalibrationCycle();
  } else if (cmd == "go") {
    Serial.println("[CMD] Manual GO triggered via Serial.");
    handleGoSignal();
  } else if (cmd.length() > 0) {
    Serial.println("[CMD] Unknown command. Send 'c' to (re)start calibration.");
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(100);

  pinMode(EOG_PIN, INPUT);
  pinMode(EEG_PIN, INPUT);

  pixels.begin();
  pixels.clear();
  pixels.show();

  setupBLEServer();
  bootTime = millis();
}

void loop() {
  handleSerialCommands();

  if (!autoCalibTriggered &&
      (millis() - bootTime) >= AUTO_CALIB_DELAY_MS &&
      eogCalibPhase == EOGCALIB_WAITING &&
      eegCalibPhase == EEGCALIB_WAITING) {
    autoCalibTriggered = true;
    beginEOGCalibration();
  }

  handleEOG();
  handleEEGSampling();
}