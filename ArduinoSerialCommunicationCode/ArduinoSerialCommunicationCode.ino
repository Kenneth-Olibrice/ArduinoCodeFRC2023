#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#define SENSOR_ADDRESS 0x52
#define NEOPIXEL_PIN 8
#define PS_DATA_0 0x08 // First register in the data block read. 
#define PROXIMITY_THRESHOLD 10


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

void loop() {
  getPeripheralData(valuesFromSensor);
  pullProximityAndRGBValues(valuesFromSensor, proximityRGB);
  if(Serial.available() > 0) { // RoboRIO has something to say.
    // TODO add the read function once its finished to this section.
  }

  hue = getHue(proximityRGB);
  double hueDegrees = (((double) (hue))/65355)*360;
  Serial.print("degrees: ");
  Serial.println(hueDegrees);
    uint8_t* trueRGB[3];
  if(proximityRGB[3] > PROXIMITY_THRESHOLD) {
  if(hueDegrees > 158) {
  trueRGB[0] = 148;
  trueRGB[1] = 0;
  trueRGB[2] = 211;
  } else {
    trueRGB[0] = 255;
    trueRGB[1] = 255;
    trueRGB[2] = 0; 
  }
  } else {
    trueRGB[0] = 0;
    trueRGB[1] = 0;
    trueRGB[2] = 0;
  }
  for(int i =0; i < 30; i++) {
    communicationStrip.setPixelColor(i, trueRGB[0], trueRGB[1], trueRGB[2]);
    // communicationStrip.setPixelColor(i, proximityRGB[0], rgbValuessToSend[1], proximityRGB[2]); // Send RGB Values
  }
  
  Serial.print("proximity:");
  Serial.println(proximityRGB[3]);
  communicationStrip.show();
  //sendDataToRoboRIO(proximityRGB, 4);
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
  outArray[0] = inArray[11];
  outArray[1] = inArray[5];
  outArray[2] = inArray[8];
  outArray[3] = inArray[0];
}

void sendDataToRoboRIO(uint8_t* data, size_t numBytes) {
  for(int i = 0; i < numBytes; i++) {
    Serial.write(data[i]);
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
