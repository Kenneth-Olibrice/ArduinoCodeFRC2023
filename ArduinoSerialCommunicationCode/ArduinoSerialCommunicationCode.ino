#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#define SENSOR_ADDRESS 0x52
#define PS_DATA_0 0x08 // First register in the data block read. 

uint8_t* valuesFromSensor = new uint8_t[14];

void setup() {
  Serial.begin(9600);
  Wire.begin();
  writeRegister(0x0, 0b0111); // Set the read modes for the color sensor.
  writeRegister(0x04,0b01000000); // Set the color sensor resolution and measurement speed. TODO tune this value and see its effect.
  for(int i = 0; i < 14; i++) { // Clear trash data to prevent undefined behavior.
    valuesFromSensor[i] = 0; 
  }
}

void loop() {
  getPeripheralData(valuesFromSensor);
  sendDataToRoboRIO(valuesFromSensor, 14);
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

