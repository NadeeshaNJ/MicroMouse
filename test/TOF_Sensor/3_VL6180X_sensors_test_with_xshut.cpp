#include <Wire.h>
#include "Adafruit_VL6180X.h"

const int shut1 = 17;
const int shut2 = 16;
const int shut3 = 4;
int xshut[3] = {shut1, shut2, shut3};

Adafruit_VL6180X vl[3]; // one instance per sensor

int currentSensor = 0;
unsigned long sensorStartTime = 0;
bool sensorPowerOn = false;
bool sensorInitDone = false;
unsigned long sensorDelay = 500; // Can be adjusted

void setup() {
  Wire.begin();
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(xshut[i], OUTPUT);
    digitalWrite(xshut[i], LOW); // Shut all down
  }

}

void loop() {
  unsigned long now = millis();

  if (!sensorPowerOn) {
    // Step 1: Power ON sensor
    digitalWrite(xshut[currentSensor], HIGH);
    sensorStartTime = now;
    sensorPowerOn = true;
    sensorInitDone = false;
  } 
  else if (!sensorInitDone && (now - sensorStartTime >= sensorDelay)) {
    // Step 2: Try to initialize after delay
    if (!vl[currentSensor].begin()) {
      Serial.print("Sensor ");
      Serial.print(currentSensor + 1);
      Serial.println(": Initialization failed.");
      digitalWrite(xshut[currentSensor], LOW); // Power off
    } 
    else {
      // Step 3: Read range
      uint8_t range = vl[currentSensor].readRange();
      uint8_t status = vl[currentSensor].readRangeStatus();

      Serial.print("Sensor ");
      Serial.print(currentSensor + 1);
      if (status == VL6180X_ERROR_NONE) {
        Serial.print(": Distance: ");
        Serial.print(range);
        Serial.println(" mm");
      } 
      else if(status==7 || status==13){
        range = 200;
        Serial.print(": Distance(MAX): ");
        Serial.print(range);
        Serial.println(" mm");
      }
      else {
        Serial.print(": Error reading range. Status code: ");
        Serial.println(status);
      }

      digitalWrite(xshut[currentSensor], LOW); // Power off after reading
    }

    sensorInitDone = true;
    sensorPowerOn = false;
    currentSensor = (currentSensor + 1) % 3; // Move to next sensor
  }

}
// This code is designed to test multiple VL6180X sensors using the XSHUT pin for power control.