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
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech
// Copyright (c) 2026 Varun Patil - vap05072006@gmail.com

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
  Based on:
 - Arduino ToneMelody example (public domain)
 - Adafruit strandtest example (BSD license)

  Requires Adafruit NeoPixel library (install via Arduino Library Manager)
*/

#include "pitches.h"
#include <Adafruit_NeoPixel.h>
#define BUZZER_PIN 8
#define LED_MOTOR_PIN 7
#define PIXEL_PIN 15
#define BOOT_PIN 9
// How many NeoPixels are attached to the Arduino?
#define PIXEL_COUNT 6
#define BATTERY_LED 5

uint8_t npgPins[] = {5, 4, 3, 8, 21, 23, 22, 7, 16, 17, 20, 18, 19};
constexpr uint8_t TOTAL_PINS = sizeof(npgPins) / sizeof(npgPins[0]);

// Declare NeoPixel pixel object:
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// LUT for 1S LiPo (Voltage in ascending order)
const float voltageLUT[] = {
  3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82, 
  3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20
};

const int percentLUT[] = {
  0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 
  50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
};

const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

// Linear interpolation function
float interpolatePercentage(float voltage) {
  // Handle out-of-range voltages
  if (voltage <= voltageLUT[0]) return 0;
  if (voltage >= voltageLUT[lutSize - 1]) return 100;

  // Find the nearest LUT entries
  int i = 0;
  while (i < lutSize - 1 && voltage > voltageLUT[i + 1]) i++;

  // Interpolate
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<pixel.numPixels(); i++) { // For each pixel in pixel...
    pixel.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    pixel.show();                          //  Update pixel to match
    delay(wait);                           //  Pause for a moment
  }
}

void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      pixel.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of pixel in steps of 3...
      for(int c=b; c<pixel.numPixels(); c += 3) {
        pixel.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      pixel.show(); // Update pixel with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // pixel.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    pixel.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // pixel.rainbow(firstPixelHue, 1, 255, 255, true);
    pixel.show(); // Update pixel with new contents
    delay(wait);  // Pause for a moment
  }
}

void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      pixel.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of pixel in increments of 3...
      for(int c=b; c<pixel.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the pixel (pixel.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / pixel.numPixels();
        uint32_t color = pixel.gamma32(pixel.ColorHSV(hue)); // hue -> RGB
        pixel.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      pixel.show();                // Update pixel with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

bool checkShorts(uint8_t *npgPins, const uint8_t TOTAL_PINS){
  uint8_t shortCounter = 0;
  bool vccShorts[TOTAL_PINS] = {0}; // true if shorted with vcc
  bool gndShorts[TOTAL_PINS] = {0}; // true if shorted with gnd
  // Test for short with VCC
  for (int i = 0; i < TOTAL_PINS; i++){
    pinMode(npgPins[i], INPUT_PULLDOWN);
    delayMicroseconds(50);
    if (digitalRead(npgPins[i]) == HIGH){
      Serial.print("Pin-");
      Serial.print(npgPins[i]);
      Serial.println(" Connected to VCC");
      vccShorts[i] = true;
      shortCounter++;
    }
    pinMode(npgPins[i], INPUT); // reset back to floating
  }
  // Test for short with GND
  for (int i = 0; i < TOTAL_PINS; i++){
    if (vccShorts[i]) continue; // skip pins which are shorted to vcc
    pinMode(npgPins[i], INPUT_PULLUP);
    delayMicroseconds(50);
    if (digitalRead(npgPins[i]) == LOW){
      Serial.print("Pin-");
      Serial.print(npgPins[i]);
      Serial.println(" Connected to GND");
      gndShorts[i] = true;
      shortCounter++;
    }
    pinMode(npgPins[i], INPUT); // reset back to floating
  }
  // Test for shorts between pins
  for (int i = 0; i < TOTAL_PINS; i++){
    if (vccShorts[i] || gndShorts[i]) continue; // skip power shorted pins
    pinMode(npgPins[i], OUTPUT);
    digitalWrite(npgPins[i], HIGH);
    delayMicroseconds(50);
    // no need to go backwards, already checked
    for(int j = i + 1; j < TOTAL_PINS; j++){
      pinMode(npgPins[j], INPUT_PULLDOWN);
      delayMicroseconds(50);
      if (digitalRead(npgPins[j]) == HIGH && !vccShorts[j]){
        Serial.print("Pin-");
        Serial.print(npgPins[i]);
        Serial.print(" is shorted with Pin-");
        Serial.println(npgPins[j]);
        shortCounter++;
      }
    }
    digitalWrite(npgPins[i], LOW);
  }
  if (shortCounter != 0){
    Serial.print(shortCounter);
    Serial.println(" shorts found");
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pixel.begin();           // INITIALIZE NeoPixel pixel object (REQUIRED)
  pixel.show();            // Turn OFF all pixels ASAP
  pixel.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
  
  //check battery first
  int analogValue = analogRead(A6);
  float voltage = (analogValue / 1000.0) * 2; // This is for ESP32C6 v0.1
  voltage += 0.022;
  float percentage = interpolatePercentage(voltage);

  while(percentage<=10)   // Check if battery is less than 10%
  {
    static uint16_t fader = 100;
    static bool decreasing = true;
    pixel.clear();
    pixel.setPixelColor(BATTERY_LED, pixel.Color(fader, 0, 0));
    pixel.show();
    delay(20);
    if (decreasing){
      fader = fader - 2;
      if (fader < 10){
        decreasing = false;
      }
    }else {
      fader = fader + 2;
      if (fader > 100){
        decreasing = true;
      }
    }
  }

  pinMode(BOOT_PIN, INPUT_PULLUP);
  for (int i = 0; i < 6; i++){
    // just for pixel pattern => 0->5->1->4->2->3
    uint8_t cF = i%2 ? 1 : 0;
    uint8_t pX = cF ? 6 - (i+1)/2 : i/2;
    pixel.setPixelColor(pX, pixel.Color(255 * pX/5, 255*(5-pX)/5,  255 * (1 - pX)/5));
    pixel.show();
    delay(500);
    pixel.clear();
    if (digitalRead(BOOT_PIN) == LOW){
      if (!checkShorts(npgPins, TOTAL_PINS)){
        pixel.setPixelColor(0, pixel.Color(255, 0, 0));
        pixel.show();
        while (true){
          delay(100); // Stop if shorts are found
        }
      }
      break;
    }
  }
  
  pinMode(LED_MOTOR_PIN, OUTPUT);

  // Set static rainbow colors on each pixel individually
  pixel.setPixelColor(0, pixel.ColorHSV(0, 255, 255));       // Red
  pixel.setPixelColor(1, pixel.ColorHSV(10922, 255, 255));   // Orange
  pixel.setPixelColor(2, pixel.ColorHSV(21845, 255, 255));   // Yellow
  pixel.setPixelColor(3, pixel.ColorHSV(32768, 255, 255));   // Green
  pixel.setPixelColor(4, pixel.ColorHSV(43690, 255, 255));   // Blue
  pixel.setPixelColor(5, pixel.ColorHSV(54613, 255, 255));   // Purple
  
  pixel.show(); // Update the pixel with the new colors

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    digitalWrite(LED_MOTOR_PIN, HIGH);
    delay(pauseBetweenNotes/2);
    digitalWrite(LED_MOTOR_PIN, LOW);
    delay(pauseBetweenNotes/2);

    // stop the tone playing:
    noTone(BUZZER_PIN);
  }
}

void loop() {
  // Fill along the length of the pixel in various colors...
  colorWipe(pixel.Color(255,   0,   0), 50); // Red
  colorWipe(pixel.Color(  0, 255,   0), 50); // Green
  colorWipe(pixel.Color(  0,   0, 255), 50); // Blue

  // Do a theater marquee effect in various colors...
  theaterChase(pixel.Color(127, 127, 127), 50); // White, half brightness
  theaterChase(pixel.Color(127,   0,   0), 50); // Red, half brightness
  theaterChase(pixel.Color(  0,   0, 127), 50); // Blue, half brightness

  rainbow(10);             // Flowing rainbow cycle along the whole pixel
  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant// Fill along the length of the pixel in various colors...
}