#include <Wire.h>
#include "Adafruit_VL6180X.h"

#define shut1 33

// Create sensor object
Adafruit_VL6180X vl = Adafruit_VL6180X();
void setup() {
  pinMode(shut1,OUTPUT);
  
  Wire.begin();
  Serial.begin(115200);  
}

void loop() {
  // Read distance
  digitalWrite(shut1,LOW);
  delay(50);
  digitalWrite(shut1,HIGH);
  delay(150);
  if (!vl.begin()) {
      Serial.println("Failed to find VL6180X sensor!");
      while (1);
    }
    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();  

    if (status == VL6180X_ERROR_NONE) {
      Serial.print(": Distance: ");
      Serial.print(range);
      Serial.println(" mm");
    } 
    // else {
    //   Serial.print("Error reading range: ");
    //   Serial.println(status);
    // }

    delay(300);
}
