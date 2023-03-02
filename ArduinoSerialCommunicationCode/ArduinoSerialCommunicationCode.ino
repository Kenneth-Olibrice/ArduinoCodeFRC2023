#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#define NEOPIXEL_PIN 8
#define NUM_LEDS 30


uint8_t* rgbFromRIO = new uint8_t[3];

Adafruit_NeoPixel communicationStrip(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB); // This is the strip the human player will use to determine the desired gamepiece.

void setup() {
  Serial.begin(9600);
  communicationStrip.begin();
  for(int i = 0; i < 3; i++) {
    rgbFromRIO[i] = 0;
  }
  communicationStrip.fill(0, 0, 30);

}

void loop() {
  if(Serial.available()>= 3) {
    Serial.readBytes(rgbFromRIO, 3);
    for(int i = 0; i < NUM_LEDS; i++) {
      communicationStrip.setPixelColor(i, rgbFromRIO[0], rgbFromRIO[1], rgbFromRIO[2]);
    }
  }
}
