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
//  XIAO ESP32-C6  —  FOCUS LIGHT (LED receiver)
//
//  * Joins the NPG Lite hotspot "NPG-Focus-Light"
//  * Listens for "F,<focus>,<0|1>" on UDP 4210
//  * Fades the LED on GPIO16 (D6) in/out, replies "A,<0|1>" so the
//    NPG Lite UI shows the real LED state
//  * If the NPG Lite goes silent for 1.5 s, the LED turns off (fail-safe)
//
//  Wiring:  GPIO16 (D6) --- 220Ω --- LED long leg (+)
//           LED short leg (-) ------- GND
//
//  Board:   XIAO_ESP32C6 (Seeed) on arduino-esp32 core 3.x
// =====================================================================

#include <WiFi.h>
#include <WiFiUdp.h>

const char *AP_SSID = "NPG-Focus-Light";
const char *AP_PASS = "12345678";
const uint16_t LISTEN_PORT = 4210;
const uint16_t NPG_PORT    = 4211;

#define LED_PIN          16     // D6 on XIAO ESP32-C6
#define ONBOARD_LED      15     // XIAO user LED (active LOW)
#define LINK_TIMEOUT_MS  1500
#define FADE_INTERVAL_MS 4
#define FADE_STEP        5      // 0 -> 255 in ~200 ms

WiFiUDP udp;
bool udpReady = false;
bool ledTarget = false;
int  brightness = 0;
float lastFocus = 0;
unsigned long lastPacket = 0, lastFade = 0, lastWifiTry = 0;

void setup()
{
  Serial.begin(115200);

  // XIAO ESP32-C6 RF switch: enable it and select the onboard ceramic antenna
  pinMode(3, OUTPUT);  digitalWrite(3, LOW);
  delay(100);
  pinMode(14, OUTPUT); digitalWrite(14, LOW);   // HIGH = external U.FL antenna

  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, HIGH);              // off
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                         // low latency
  WiFi.begin(AP_SSID, AP_PASS);
  lastWifiTry = millis();
  Serial.println("Connecting to NPG Lite hotspot...");
}

void handleWifi(unsigned long now)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!udpReady)
    {
      udp.begin(LISTEN_PORT);
      udpReady = true;
      Serial.printf("Connected! XIAO IP: %s\n", WiFi.localIP().toString().c_str());
    }
    return;
  }

  // Not connected
  if (udpReady)
  {
    udp.stop();
    udpReady = false;
    Serial.println("Lost NPG Lite hotspot, reconnecting...");
  }
  ledTarget = false;
  digitalWrite(ONBOARD_LED, ((now / 250) % 2) ? LOW : HIGH);   // fast blink while searching
  if (now - lastWifiTry > 8000)
  {
    lastWifiTry = now;
    WiFi.disconnect();
    WiFi.begin(AP_SSID, AP_PASS);
  }
}

void handlePackets(unsigned long now)
{
  if (!udpReady) return;
  while (udp.parsePacket() > 0)
  {
    char buf[48];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    // only the NPG Lite (the hotspot gateway) may fire; ignore other devices on the hotspot
    if (udp.remoteIP() != WiFi.gatewayIP()) continue;
    buf[n] = 0;

    float focus;
    int on;
    if (sscanf(buf, "F,%f,%d", &focus, &on) == 2)
    {
      lastPacket = now;
      lastFocus = focus;
      bool want = (on == 1);
      if (want != ledTarget)
      {
        ledTarget = want;
        Serial.printf("LED %s  (focus %.1f%%)\n", ledTarget ? "ON" : "OFF", focus);
      }
      // reply with real LED state
      udp.beginPacket(udp.remoteIP(), NPG_PORT);
      udp.printf("A,%d", ledTarget ? 1 : 0);
      udp.endPacket();
    }
  }
}

void loop()
{
  unsigned long now = millis();
  handleWifi(now);
  handlePackets(now);

  // fail-safe: no data from NPG Lite -> LED off
  if (ledTarget && now - lastPacket > LINK_TIMEOUT_MS)
  {
    ledTarget = false;
    Serial.println("No data from NPG Lite -> LED off");
  }

  if (WiFi.status() == WL_CONNECTED)
    digitalWrite(ONBOARD_LED, ledTarget ? LOW : HIGH);

  // smooth glow fade
  if (now - lastFade >= FADE_INTERVAL_MS)
  {
    lastFade = now;
    int target = ledTarget ? 255 : 0;
    if (brightness < target) brightness = min(255, brightness + FADE_STEP);
    else if (brightness > target) brightness = max(0, brightness - FADE_STEP);
    analogWrite(LED_PIN, (brightness * brightness) / 255);   // gamma: looks smoother to the eye
  }
}
