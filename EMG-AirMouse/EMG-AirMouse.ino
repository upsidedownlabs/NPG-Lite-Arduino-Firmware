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

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech
// Copyright (c) 2026 Varun Patil - vap05072006@gmail.com

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <BleCombo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <esp_bt_device.h>
#include <vector>

#define SDA_PIN        22
#define SCL_PIN        23
#define LED_MOTOR_PIN  7
#define EMG_PIN        A0

#define SENSITIVITY    0.6f
#define DEADZONE       1.5f
#define POLL_MS        10

#define SAMPLE_RATE    500
#define EMG_THRESHOLD  100

int axisX = 1;
int axisY = 0;
int signX = 1;
int signY = 1;

MPU6050 mpu6050(Wire);

// Band-Stop Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// Reference: https://github.com/upsidedownlabs/BioAmp-Filter-Designer
class NotchFilter {
private:
    struct BiquadState { float z1 = 0, z2 = 0; };
    BiquadState state0;
    BiquadState state1;

public:
    float process(float input) {
        float output = input;

        // Biquad section 0
        float x0 = output - (-1.56858163f * state0.z1) - (0.96424138f * state0.z2);
        output = 0.96508099f * x0 + -1.56202714f * state0.z1 + 0.96508099f * state0.z2;
        state0.z2 = state0.z1;
        state0.z1 = x0;

        // Biquad section 1
        float x1 = output - (-1.61100358f * state1.z1) - (0.96592171f * state1.z2);
        output = 1.00000000f * x1 + -1.61854514f * state1.z1 + 1.00000000f * state1.z2;
        state1.z2 = state1.z1;
        state1.z1 = x1;

        return output;
    }

    void reset() {
        state0.z1 = state0.z2 = 0;
        state1.z1 = state1.z2 = 0;
    }
};

// High-Pass Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// Reference: https://github.com/upsidedownlabs/BioAmp-Filter-Designer

class EMGHighPassFilter {
private:
    struct BiquadState { float z1 = 0, z2 = 0; };
    BiquadState state0;

public:
    float process(float input) {
        float output = input;

        // Biquad section 0
        float x0 = output - (-0.82523238f * state0.z1) - (0.29463653f * state0.z2);
        output = 0.52996723f * x0 + -1.05993445f * state0.z1 + 0.52996723f * state0.z2;
        state0.z2 = state0.z1;
        state0.z1 = x0;

        return output;
    }

    void reset() {
        state0.z1 = state0.z2 = 0;
    }
};

// ── Envelope Detector ────────────────────────────────────────────
class EnvelopeFilter {
private:
  std::vector<double> circularBuffer;
  double sum = 0.0;
  int dataIndex = 0;
  const int bufferSize;
public:
  EnvelopeFilter(int bufferSize) : bufferSize(bufferSize) {
    circularBuffer.resize(bufferSize, 0.0);
  }
  double getEnvelope(double absEmg) {
    sum -= circularBuffer[dataIndex];
    sum += absEmg;
    circularBuffer[dataIndex] = absEmg;
    dataIndex = (dataIndex + 1) % bufferSize;
    return (sum / bufferSize);
  }
};

NotchFilter notchEMG;
EMGHighPassFilter emgHP;
EnvelopeFilter emgEnvelope(16);

bool mouseButtonHeld = false;

// ── Motor pulse ───────────────────────────────────────────────────
void motorPulse(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_MOTOR_PIN, HIGH);
    delay(300);
    digitalWrite(LED_MOTOR_PIN, LOW);
    if (i < count - 1) delay(300);
  }
}

// ── Gyro sampling ────────────────────────────────────────────────
void sampleGyro(float out[3], int sampleMs = 800) {
  out[0] = out[1] = out[2] = 0;
  int count = 0;
  unsigned long start = millis();
  while (millis() - start < (unsigned long)sampleMs) {
    mpu6050.update();
    out[0] += mpu6050.getGyroX();
    out[1] += mpu6050.getGyroY();
    out[2] += mpu6050.getGyroZ();
    count++;
    delay(10);
  }
  if (count > 0) {
    out[0] /= count; out[1] /= count; out[2] /= count;
  }
}

int dominantAxis(float g[3]) {
  int ax = 0;
  for (int i = 1; i < 3; i++) {
    if (abs(g[i]) > abs(g[ax])) ax = i;
  }
  return ax;
}

void runCalibration() {
  Serial.println("Tilt UP now...");
  motorPulse(2);
  delay(400);

  float gUp[3];
  sampleGyro(gUp);
  motorPulse(1);
  Serial.printf("UP: X=%.2f Y=%.2f Z=%.2f\n", gUp[0], gUp[1], gUp[2]);

  int upAxis = dominantAxis(gUp);
  float upSign = (gUp[upAxis] >= 0) ? 1.0f : -1.0f;

  delay(600);
  Serial.println("Tilt LEFT now...");
  motorPulse(2);
  delay(400);

  float gLeft[3];
  sampleGyro(gLeft);
  motorPulse(1);
  Serial.printf("LEFT: X=%.2f Y=%.2f Z=%.2f\n", gLeft[0], gLeft[1], gLeft[2]);

  int leftAxis = dominantAxis(gLeft);
  float leftSign = (gLeft[leftAxis] >= 0) ? 1.0f : -1.0f;

  axisY = upAxis;
  signY = -(int)upSign;
  axisX = leftAxis;
  signX = -(int)leftSign;

  Serial.printf("MouseX → gyro[%d]*%d | MouseY → gyro[%d]*%d\n",
                axisX, signX, axisY, signY);
}

float getGyroAxis(int axis) {
  switch (axis) {
    case 0: return mpu6050.getGyroX();
    case 1: return mpu6050.getGyroY();
    case 2: return mpu6050.getGyroZ();
  }
  return 0;
}

// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(LED_MOTOR_PIN, OUTPUT);
  digitalWrite(LED_MOTOR_PIN, LOW);
  pinMode(EMG_PIN, INPUT);

  Keyboard.begin();
  Mouse.begin();
  esp_bt_dev_set_device_name("NPG Air Mouse");

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu6050.begin();
  delay(200);

  runCalibration();

  Serial.println("Calculating gyro offsets, hold still...");
  mpu6050.calcGyroOffsets(true);
  Serial.println("Done. Ready.");
}

void loop() {
  // ── EMG sampling at SAMPLE_RATE ──────────────────────────────
  static unsigned long lastEmgMicros = micros();
  static unsigned long lastMouseMillis = millis();
  static unsigned long lastPrintMillis = millis();

  unsigned long nowMicros = micros();
  if (nowMicros - lastEmgMicros >= 1000000UL / SAMPLE_RATE) {
    lastEmgMicros = nowMicros;

    int rawEMG = analogRead(EMG_PIN);
    float notched = notchEMG.process((float)rawEMG);
    float filtered = (float)emgHP.process((double)notched);
    double envelope = emgEnvelope.getEnvelope(abs(filtered));

    // Left click: hold when above threshold, release when below
    if (Keyboard.isConnected()) {
      if (envelope > EMG_THRESHOLD && !mouseButtonHeld) {
        // Mouse.click(MOUSE_LEFT);
        Mouse.press(MOUSE_LEFT);
        mouseButtonHeld = true;
      } else if (envelope <= EMG_THRESHOLD && mouseButtonHeld) {
        Mouse.release(MOUSE_LEFT);
        mouseButtonHeld = false;
      }
    }

    unsigned long nowMillis = millis();
    if (nowMillis - lastPrintMillis >= 50) {
      Serial.println(envelope);  // for threshold tuning
      lastPrintMillis = nowMillis;
    }
  }

  // ── Mouse movement ───────────────────────────────────────────
  if (!Keyboard.isConnected()) {
    return;
  }

  unsigned long nowMillis = millis();
  if (nowMillis - lastMouseMillis < POLL_MS) {
    return;
  }
  lastMouseMillis = nowMillis;

  mpu6050.update();

  float rawX = signX * getGyroAxis(axisX);
  float rawY = signY * getGyroAxis(axisY);

  if (abs(rawX) < DEADZONE) rawX = 0;
  if (abs(rawY) < DEADZONE) rawY = 0;

  int dx = constrain((int)(rawX * SENSITIVITY), -127, 127);
  int dy = constrain((int)(rawY * SENSITIVITY), -127, 127);

  if (dx != 0 || dy != 0) {
    Mouse.move((signed char)dx, (signed char)dy);
  }

}
