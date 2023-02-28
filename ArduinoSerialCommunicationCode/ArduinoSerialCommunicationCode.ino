#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#define SENSOR_ADDRESS 0x52
#define NEOPIXEL_PIN 8
#define PS_DATA_0 0x08 // First register in the data block read. 


uint8_t* dataBuffer = new uint8_t[14]; // Proximity Sensor, Infared, Green, Blue, Red (In order of reception).
uint8_t* LED_ADDRESSES = new uint8_t[2];
uint8_t* rgbValuesToSend = new uint8_t[3];
Adafruit_NeoPixel communicationStrip(30, NEOPIXEL_PIN, NEO_GRB); // This is the strip the human player will use to determine the desired gamepiece.

void setup() {
  Serial.begin(9600);
  Wire.begin();
  writeRegister(0x0, 0b0111); // Set the read modes for the color sensor.
  writeRegister(0x04,0b01000000); // Set the color sensor resolution and measurement speed. TODO tune this value and see its effect.

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
  getPeripheralData(dataBuffer);
  interperetPeripheralData(dataBuffer, rgbValuesToSend);



  if(Serial.available() > 0) { // RoboRIO has something to say.
    // TODO add the read function once its finished to this section.
  }


  uint32_t packedRGB = communicationStrip.ColorHSV(getHue(rgbValuesToSend),255, 255); // Full hue, full brightness
  // uint32_t packedRGB = communicationStrip.ColorHSV(getHue(rgbValuesToSend),255, 255); // Full hue, half brightness.
  // uint32_T packedRGB =  communicationStrip.ColorHSV(getHue(rgbValuesToSend),255, 0); // Full hue, 0 brightness (lights should be off)
  for(int i =0; i < 30; i++) {
    communicationStrip.setPixelColor(i, packedRGB);
    // communicationStrip.setPixelColor(i, rgbValuesToSend[0], rgbValuessToSend[1], rgbValuesToSend[2]); // Send RGB Values
    
  }
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

void interperetPeripheralData(uint8_t* inArray, uint8_t* outArray) {
  uint16_t red = (inArray[11])|((inArray[12]<<8)&0xff);
  uint16_t green = (inArray[5])|((inArray[6]<<8)&0xff);
  uint16_t blue = (inArray[8])|((inArray[9]<<8)&0xff);
  outArray[0] = inArray[11];
  outArray[1] = inArray[5];
  outArray[2] = inArray[8];
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

  // double hue = (uint16_t) ((intermediateValue - minValue) / maxValue)*360; // Express this ratio in degrees.
  double hue = (uint16_t) ((intermediateValue - minValue) / maxValue)*65535; // Express this ratio in 16bits.
  Serial.print("Hue: ");
  Serial.println(hue);
  return hue;
}
