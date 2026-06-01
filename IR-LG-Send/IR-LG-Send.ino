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
  - Sender code based on the “SendDemo” example from the Arduino-IRremote library
    (https://github.com/Arduino-IRremote/Arduino-IRremote/blob/main/examples/SendDemo/SendDemo.ino)
*/

#include <Arduino.h>
#include <IRremote.hpp>
#include <Adafruit_NeoPixel.h>

#define IR_SEND_PIN 22 // your IR-LED pin (via resistor/transistor)
#define BOOT_BUTTON 9  // NPG-Lite BOOT button (active LOW)
#define PIXEL_PIN 15
#define BATTERY_VOLTAGE_PIN A6

Adafruit_NeoPixel pixel(6, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define BATTERY_LED 5

int c = 0; // Counter for maintaining ON/OFF state

// The full 28-bit MSB-first codes captured by the receiver sketch
static const uint32_t FULL_ON = 0x8800B0B;
static const uint32_t FULL_OFF = 0x88C0051;

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
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  Serial.println("\n===== LG-AC: Send Full 28-bit Codes =====");
  IrSender.begin(IR_SEND_PIN); // initialize the IR sender (RMT)
  Serial.println("Press BOOT Button to send ON/OFF");
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

  // When Boot button is pressed
  if (digitalRead(BOOT_BUTTON) == LOW)
  {
    if (c % 2 == 0)
    {
      Serial.println("→ Sending AC ON");
      // sendLG automatically uses the correct 38 kHz carrier and timing for LG
      IrSender.sendLG(FULL_ON, 28);
      Serial.println("  [Done]\n");
      delay(500);
    }
    else
    {
      Serial.println("→ Sending AC OFF");
      IrSender.sendLG(FULL_OFF, 28);
      Serial.println("  [Done]\n");
      delay(500);
    }
    c = c + 1;
  }
}
