#include "VL6180XManager.h"

VL6180XManager::VL6180XManager(int* _xshutPins, int _count, unsigned long _delayTime) {
  xshutPins = _xshutPins;
  count = _count;
  delayTime = _delayTime;
  sensors = new Adafruit_VL6180X[_count];
}

void VL6180XManager::begin() {
  for (int i = 0; i < count; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW); // Shut all down
  }
}

void VL6180XManager::update() {
  unsigned long now = millis();

  if (!sensorPowerOn) {
    digitalWrite(xshutPins[currentSensor], HIGH);
    sensorStartTime = now;
    sensorPowerOn = true;
    sensorInitDone = false;
  }
  else if (!sensorInitDone && (now - sensorStartTime >= delayTime)) {
    if (!sensors[currentSensor].begin()) {
      Serial.print("Sensor ");
      Serial.print(currentSensor + 1);
      Serial.println(": Initialization failed.");
      //digitalWrite(xshutPins[currentSensor], LOW);
    } 
    else {
      uint8_t range = sensors[currentSensor].readRange();
      uint8_t status = sensors[currentSensor].readRangeStatus();      
      if (status == VL6180X_ERROR_NONE) {        
        Serial.print(range);        
      }
      else if (status == 7 || status == 13) {
        range = 200;                       
      }
      else {
        Serial.print("Sensor: ");
        Serial.print(currentSensor + 1);
        Serial.print(": Error reading range. Status code: ");
        Serial.println(status);
      }
      digitalWrite(xshutPins[currentSensor], LOW);
    }
    sensorInitDone = true;
    sensorPowerOn = false;
    currentSensor = (currentSensor + 1) % count;
  }
}

void VL6180XManager::updateWithSerial() {
  unsigned long now = millis();

  if (!sensorPowerOn) {
    digitalWrite(xshutPins[currentSensor], HIGH);
    sensorStartTime = now;
    sensorPowerOn = true;
    sensorInitDone = false;
  }
  else if (!sensorInitDone && (now - sensorStartTime >= delayTime)) {
    if (!sensors[currentSensor].begin()) {
      Serial.print("Sensor ");
      Serial.print(currentSensor + 1);
      Serial.println(": Initialization failed.");
      //digitalWrite(xshutPins[currentSensor], LOW);
    } 
    else {
      uint8_t range = sensors[currentSensor].readRange();
      uint8_t status = sensors[currentSensor].readRangeStatus();

      Serial.print("Sensor ");
      Serial.print(currentSensor + 1);
      if (status == VL6180X_ERROR_NONE) {
        Serial.print(": Distance: ");
        Serial.print(range);
        Serial.println(" mm");
      }
      else if (status == 7 || status == 13) {
        range = 200;
        Serial.print(": Distance(MAX): ");
        Serial.print(range);
        Serial.println(" mm");
      }
      else {
        Serial.print(": Error reading range. Status code: ");
        Serial.println(status);
      }

      digitalWrite(xshutPins[currentSensor], LOW);
    }

    sensorInitDone = true;
    sensorPowerOn = false;
    currentSensor = (currentSensor + 1) % count;
  }
}

int VL6180XManager::getDistance() const {
  if (currentSensor < count) {
    return sensors[currentSensor].readRange();
  }
  return -1; // Return -1 if no sensor is currently active
}