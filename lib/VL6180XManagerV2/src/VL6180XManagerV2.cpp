#include "VL6180XManagerV2.h"

VL6180XManagerV2::VL6180XManagerV2(int* xshutPins, int sensorCount, int* corrections) {
  _xshutPins = xshutPins;
  _sensorCount = sensorCount;
  _sensors = new Adafruit_VL6180X[_sensorCount];
  _corrections = corrections;
}

void VL6180XManagerV2::begin() {
  // Set XSHUT pins as output and disable all sensors
  for (int i = 0; i < _sensorCount; i++) {
    pinMode(_xshutPins[i], OUTPUT);
    digitalWrite(_xshutPins[i], LOW);
  }
  delay(5);

  // Power up and assign I2C addresses one-by-one
  for (int i = 0; i < _sensorCount; i++) {
    digitalWrite(_xshutPins[i], HIGH);
    delay(50); // Ensure sensor boots

    if (!_sensors[i].begin()) {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(" init failed.");
      continue;
    }

    _sensors[i].setAddress(_i2cBaseAddress + i + 1 ); // Assign new I2C address
    delay(5); // Allow time for address change
  }

  // Reconnect all sensors at new addresses
  for (int i = 0; i < _sensorCount; i++) {
    _sensors[i].begin();
  }
}

std::vector<int> VL6180XManagerV2::readAll() {
    std::vector<int> distances(_sensorCount, 0);
    for (int i = 0; i < _sensorCount; i++) {
        int range = _sensors[i].readRange();
        int status = _sensors[i].readRangeStatus();
        int correctedRange = range;

        //Serial.print("Sensor ");
        //Serial.print(i + 1);
        if (status == VL6180X_ERROR_NONE) {
            // Serial.print(": Distance = ");
            // Serial.print(range);
            // Serial.println(" mm");
            if (_corrections) correctedRange -= _corrections[i]; // apply offset correction
            if(correctedRange < 0) correctedRange = 0; // Ensure non-negative distance
            if(correctedRange > 200) correctedRange = 200; // Cap distance to max 200mm
            //Serial.print(": Corrected Distance = ");
            //Serial.print(correctedRange);
            distances[i] = correctedRange;
        } 
        else if (status == 7 || status == 13 || status == 12) {
            //Serial.println(": Distance(MAX) = 200 mm");
            distances[i] = 200;
        } 
        else {
            Serial.print("Sensor ");
            Serial.print(i+1);
            Serial.print(": Error. Status code = ");
            Serial.println(status);
            distances[i] = -1;
        }
  }
  return distances;
}
