#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#define SENSOR_ADDRESS 0x52
#define NEOPIXEL_PIN 8
#define PS_DATA_0 0x08 // First register in the data block read. 
#define PROXIMITY_THRESHOLD 20


uint8_t* valuesFromSensor = new uint8_t[14]; // Proximity Sensor, Infared, Green, Blue, Red (In order of reception).
uint8_t* proximityRGB = new uint8_t[4];
uint16_t hue = 0;
Adafruit_NeoPixel communicationStrip(30, NEOPIXEL_PIN, NEO_GRB); // This is the strip the human player will use to determine the desired gamepiece.

void setup() {
  Serial.begin(9600);
  Wire.begin();
  communicationStrip.begin();
  writeRegister(0x0, 0b0111); // Set the read modes for the color sensor.
  writeRegister(0x04,0b01000000); // Set the color sensor resolution and measurement speed. TODO tune this value and see its effect.
  for(int i = 0; i < 14; i++) { // Clear trash data to prevent undefined behavior.
    valuesFromSensor[i] = 0; 
  }
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
  getPeripheralData(valuesFromSensor);
  pullProximityAndRGBValues(valuesFromSensor, proximityRGB);
  if(Serial.available() > 0) { // RoboRIO has something to say.
    // TODO add the read function once its finished to this section.
  }

  hue = getHue(proximityRGB);
  uint8_t* trueRGB[3];
  if(proximityRGB[3] > PROXIMITY_THRESHOLD) {
  if(hue > 32000 || hue < 3000) {
  trueRGB[0] = 148;
  trueRGB[1] = 0;
  trueRGB[2] = 211;
  } else {
    trueRGB[0] = 255;
    trueRGB[1] = 255;
    trueRGB[2] = 0; 
  }
  for(int i =0; i < 30; i++) {
    communicationStrip.setPixelColor(i, trueRGB[0], trueRGB[1], trueRGB[2]);
    // communicationStrip.setPixelColor(i, proximityRGB[0], rgbValuessToSend[1], proximityRGB[2]); // Send RGB Values
  }
  }
  Serial.print("proximity:");
  Serial.println(proximityRGB[3]);
  communicationStrip.show();
  delay(150);
}



void readRegisters(uint8_t reg, int numBytes, uint8_t* buff) {
  Wire.beginTransmission(SENSOR_ADDRESS); 
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(SENSOR_ADDRESS, numBytes);
  int currentByte = 0;
  while(Wire.available()) {
    buff[currentByte] = Wire.read();
    currentByte++;
  }
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void getPeripheralData(uint8_t* outArray) {
  readRegisters(PS_DATA_0, 14, outArray);
}

void pullProximityAndRGBValues(uint8_t* inArray, uint8_t* outArray) {
  uint16_t red = (inArray[11])|((inArray[12]<<8)&0xff);
  uint16_t green = (inArray[5])|((inArray[6]<<8)&0xff);
  uint16_t blue = (inArray[8])|((inArray[9]<<8)&0xff);
  outArray[0] = inArray[11];
  outArray[1] = inArray[5];
  outArray[2] = inArray[8];
  outArray[3] = inArray[0];
}

/**
  Index values:
  [0-1]: Proximity sensor.
  [2-4]: Infared.
  [5-7]: Green sensor data.
  [8-10]: Blue sensor data.
  [11-13]: Red sensor data.
*/
void sendDataToRoboRIO() {
  for(int i = 0; i < 4; i++) {
    Serial.write(proximityRGB[i]);
  }
}

void readRoboRIOData(uint8_t* outArray) {
}

uint16_t getHue(uint8_t* rgbArray) {
  double maxValue = max(rgbArray[0], max(rgbArray[1], rgbArray[2]));
  if(maxValue <= 0) return; // Avoid division by zero errors.
  double minValue = min(rgbArray[0], min(rgbArray[1], rgbArray[2]));
  double intermediateValue = (maxValue + minValue) / 2; // In case the following code fails, use the arithmetic average of the min and max.
  for(int i = 0; i < 3; i++) {
    if(rgbArray[i] != maxValue && rgbArray[i] != minValue) intermediateValue = rgbArray[i];
  }
  uint16_t hue = (uint16_t) (((intermediateValue - minValue) / maxValue)*65535); // Express this ratio in 16bits.
  Serial.print("Hue: ");
  Serial.println(hue);
  return hue;
}
