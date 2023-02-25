#include <Wire.h>
#define SENSOR_ADDRESS 0x52

uint8_t* dataBuffer = new uint8_t[12]; // RGB

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  writeRegister(0x0, 0b0110);

  // Clear trash data to prevent undefined behavior.
  for(int i = 0; i < 3; i++) {
    dataBuffer[i] = 0; 
  }

}

void loop() {
  getPeripheralData(dataBuffer);
  if(Serial.availableForWrite() >= 12) {
    sendDataToRoboRIO();
  }

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
  readRegisters(0xA, 12, outArray);
}

/**
  The RoboRIO will receive each rgb value individually delimited by \n characters.
*/
void sendDataToRoboRIO() {
  for(int i = 0; i < 12; i++) {
    Serial.write(dataBuffer[i]);
    
  }
}