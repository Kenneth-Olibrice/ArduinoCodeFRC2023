#include <Wire.h>
#define SENSOR_ADDRESS 0x52

uint8_t* dataBuffer = new uint8_t[14]; // Proximity Sensor, Infared, Green, Blue, Red (In order of reception).

void setup() {
  Serial.begin(9600);
  Wire.begin();
  writeRegister(0x0, 0b0111);
  writeRegister(0x04,0b01000000); 

  // Clear trash data to prevent undefined behavior.
  for(int i = 0; i < 14; i++) {
    dataBuffer[i] = 0; 
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
  getPeripheralData(dataBuffer);
  if(Serial.availableForWrite() >= 12) {
    sendDataToRoboRIO();
  }

  if(Serial.available() > 0) { // RoboRIO has something to say.
    interperetCommand(Serial.read());
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
  readRegisters(0x08, 14, outArray);
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
  for(int i = 0; i < 14; i++) {
    Serial.write(dataBuffer[i]);
  }
}

void interperetCommand(uint8_t command) {
  // TODO Implement this function.
}
