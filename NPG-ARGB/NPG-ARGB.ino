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

#include <Adafruit_NeoPixel.h>
#include <vector>

#define LED_PIN 22 // Pin connected to the NeoPixel data input
#define NUM_PIXELS 8
#define EMG_PIN A0 // Channel 1

#define SAMPLE_RATE 500
#define EMG_THRESHOLD 50
#define FADE_MS 300 // 0 -> max (and max -> 0) in this many milliseconds
#define LED_UPDATE_MS 10

Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Band-Stop Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: [48.0, 52.0] Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// Reference: https://github.com/upsidedownlabs/BioAmp-Filter-Designer
class NotchFilter
{
private:
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;
  BiquadState state1;

public:
  float process(float input)
  {
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

  void reset()
  {
    state0.z1 = state0.z2 = 0;
    state1.z1 = state1.z2 = 0;
  }
};

// High-Pass Butterworth IIR digital filter
// Sampling rate: 500.0 Hz, frequency: 70.0 Hz
// Filter is order 2, implemented as second-order sections (biquads)
// Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// Reference: https://github.com/upsidedownlabs/BioAmp-Filter-Designer

class EMGHighPassFilter
{
private:
  struct BiquadState
  {
    float z1 = 0, z2 = 0;
  };
  BiquadState state0;

public:
  float process(float input)
  {
    float output = input;

    // Biquad section 0
    float x0 = output - (-0.82523238f * state0.z1) - (0.29463653f * state0.z2);
    output = 0.52996723f * x0 + -1.05993445f * state0.z1 + 0.52996723f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    return output;
  }

  void reset()
  {
    state0.z1 = state0.z2 = 0;
  }
};

// Envelope Detector
class EnvelopeFilter
{
private:
  std::vector<double> circularBuffer;
  double sum = 0.0;
  int dataIndex = 0;
  const int bufferSize;

public:
  EnvelopeFilter(int bufferSize) : bufferSize(bufferSize)
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

NotchFilter notchEMG;
EMGHighPassFilter emgHP;
EnvelopeFilter emgEnvelope(16);

double envelope = 0;
float level = 0; // current LED level, 0..255

// Set all pixels to white at the given level
void showLevel(uint8_t v)
{
  uint8_t g = strip.gamma8(v); // gamma correction for a visually smooth fade
  for (int i = 0; i < NUM_PIXELS; i++)
  {
    strip.setPixelColor(i, strip.Color(g, g, g));
  }
  strip.show();
}

void setup()
{
  Serial.begin(115200);
  pinMode(EMG_PIN, INPUT);

  strip.begin();
  strip.setBrightness(255);
  strip.clear();
  strip.show();
}

void loop()
{
  static unsigned long lastEmgMicros = micros();
  static unsigned long lastLedMillis = millis();

  // EMG sampling at SAMPLE_RATE
  unsigned long nowMicros = micros();
  if (nowMicros - lastEmgMicros >= 1000000UL / SAMPLE_RATE)
  {
    lastEmgMicros = nowMicros;

    int rawEMG = analogRead(EMG_PIN);
    float notched = notchEMG.process((float)rawEMG);
    float filtered = emgHP.process(notched);
    envelope = emgEnvelope.getEnvelope(fabs(filtered));
  }

  // LED fade update
  unsigned long nowMillis = millis();
  unsigned long dt = nowMillis - lastLedMillis;
  if (dt >= LED_UPDATE_MS)
  {
    lastLedMillis = nowMillis;

    float step = 255.0f * dt / FADE_MS;
    float prev = level;

    if (envelope > EMG_THRESHOLD)
    {
      level = min(255.0f, level + step); // fade in
    }
    else
    {
      level = max(0.0f, level - step); // fade out
    }

    if ((uint8_t)level != (uint8_t)prev)
    {
      showLevel((uint8_t)level);
    }

    Serial.println(envelope); // for threshold tuning
  }
}
