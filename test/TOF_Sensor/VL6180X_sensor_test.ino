#include <Wire.h>
#include "Adafruit_VL6180X.h"


// Create sensor object
Adafruit_VL6180X vl = Adafruit_VL6180X();
void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  Serial.println("VL6180X Test Start");
    if (!vl.begin()) {
      Serial.println("Failed to find VL6180X sensor 1 !");
      while (1);
    }
}

void loop() {
  // Read distance
    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();  

    if (status == VL6180X_ERROR_NONE) {
      Serial.print(": Distance: ");
      Serial.print(range);
      Serial.println(" mm");
    } 
    else {
      Serial.print("Error reading range: ");
      Serial.println(status);
    }

    delay(300);
}
