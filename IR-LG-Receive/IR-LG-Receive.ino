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

// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
  Before uploading, install the “Arduino-IRremote” library by shirriff, z3t0, ArminJo via Library Manager

  References:
  - Receiver code based on the “IRrecvDumpV2” example from the Arduino-IRremote library
    (https://github.com/Arduino-IRremote/Arduino-IRremote/blob/main/examples/IRrecvDumpV2/IRrecvDumpV2.ino)
*/

#include <Arduino.h>
#include <IRremote.hpp>
#include <Adafruit_NeoPixel.h>

#define IR_RECV_PIN 5 // your IR-receiver OUT pin
#define PIXEL_PIN 15
#define BATTERY_VOLTAGE_PIN A6

Adafruit_NeoPixel pixel(6, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define BATTERY_LED 5

// Reverse the order of the lowest `bitCount` bits in `v`
// Because the library gives us LSB-first data, but we need MSB-first.
uint32_t reverseBits(uint32_t v, uint8_t bitCount)
{
  uint32_t r = 0;
  for (uint8_t i = 0; i < bitCount; i++)
  {
    r = (r << 1) | (v & 1);
    v >>= 1;
  }
  return r;
}

// ── Battery indicator (pixel 5) ──
static const unsigned long BATTERY_CHECK_INTERVAL = 10000;
static unsigned long lastBatteryCheck = -10000;
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

void setup()
{
  pixel.begin();
  pixel.clear();
  pixel.show();

  Serial.begin(115200);
  Serial.println("\n===== LG-AC: Capture Full 28-bit Codes =====");
  IrReceiver.begin(IR_RECV_PIN, DISABLE_LED_FEEDBACK); // start IR receiver
}

void loop()
{
  batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
  batteryWinCount++;
  unsigned long currentMillis = millis();
  if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL)
  {
    int pct = getCurrentBatteryPercentage();
    uint32_t color;
    if (pct <= 20)
      color = pixel.Color(20, 0, 0);
    else if (pct <= 70)
      color = pixel.Color(30, 20, 0);
    else
      color = pixel.Color(0, 20, 0);
    pixel.setPixelColor(BATTERY_LED, color);
    pixel.show();
    lastBatteryCheck = currentMillis;
  }

  if (!IrReceiver.decode())
    return; // wait for a valid code
  auto &d = IrReceiver.decodedIRData;

  // 1) Library gives us a 27-bit value in LSB-first order
  uint32_t raw27 = d.decodedRawData;
  uint8_t bits27 = d.numberOfBits;

  // 2) Reverse to MSB-first so the most significant bit is at the top
  uint32_t rev27 = reverseBits(raw27, bits27);

  // 3) Prepend the hidden header bit (value 1) to make a full 28-bit word
  uint32_t full28 = (1UL << bits27) | rev27;

  // Print the raw data so you can record the signals
  Serial.println("\n--- Captured LG-AC Frame ---");
  Serial.print("  Protocol: ");
  Serial.println((uint16_t)d.protocol == 2 ? "LG-AC" : String((uint16_t)d.protocol));
  Serial.print("  Raw27  : 0x");
  Serial.println(raw27, HEX);
  Serial.print("  Bits27 : ");
  Serial.println(bits27);
  Serial.print("  Full28 : 0x");
  Serial.println(full28, HEX);
  Serial.print("  Bits28 : 28");
  Serial.println("\n-----------------------------");

  IrReceiver.resume(); // ready to receive the next code
  delay(200);
}
