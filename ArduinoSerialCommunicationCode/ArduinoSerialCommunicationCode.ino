#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#define SENSOR_ADDRESS 0x52
#define NEOPIXEL_PIN 8
#define PS_DATA_0 0x08

uint8_t* dataBuffer = new uint8_t[14]; // Proximity Sensor, Infared, Green, Blue, Red (In order of reception).
uint8_t* LED_ADDRESSES = new uint8_t[2];
uint8_t* rgbValuesToSend = new uint8_t[3];
Adafruit_NeoPixel communicationStrip(18, NEOPIXEL_PIN, NEO_GRB); // This is the strip the human player will use to determine the desired gamepiece.

void setup() {
  Serial.begin(9600);

  // Clear trash data to prevent undefined behavior.
  for(int i = 0; i < 14; i++) {
    dataBuffer[i] = 0; 
  }

  // TODO find the device addresses.
  LED_ADDRESSES[0] = 0; // This is the LED strip that indicates the currently held gamepiece.
  LED_ADDRESSES[1] = 0; // This is the LED strip that allows the driver/operator to signal to the human player which gamepiece they want placed onto the field.
  
  communicationStrip.begin();

}

/** RIO-ARDUINO PROTOCOL:
  1. RoboRIO will read information from available peripheral devices.
  2. Arduino will send any available peripheral data to the RoboRIO.
  3 When the Driver/Operator wants to signal a color, 
       they will press a button and the RoboRIO will run a command to periodically ping the Arduino with required information. 
       The arduino will acknowledge and the RoboRIO will stop pinging.
  4. The Arduino will adjust the LEDs accordingly.
*/
void loop() {
  if(Serial.available() > 0) { // RoboRIO has something to say.
    // TODO add the read function once its finished to this section.
  }

 for(int i = 0; i < 30; i++) {
    communicationStrip.setPixelColor(i, 255, 0, 129); // Bubblegum pink.
      communicationStrip.show();
    delay(30);
  }  
  for(int i = 0; i < 30; i++) {
    communicationStrip.setPixelColor(i, 0, 0, 0); // Bubblegum pink.
          communicationStrip.show();
    delay(30);
  }  
  delay(150);
}
