#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin(); // defaults: GPIO 21 (SDA), GPIO 22 (SCL) for ESP32

  Serial.println("I2C Scanner Running...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Scan complete.");
}

void loop() {
}
