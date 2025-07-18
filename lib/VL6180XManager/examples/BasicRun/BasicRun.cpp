#include "VL6180XManager.h"

const int xshutPins[3] = {17, 16, 4};
VL6180XManager sensorManager((int*)xshutPins, 3, 100); // pins, count, delay in ms

void setup() {
  Wire.begin();
  Serial.begin(115200);
  sensorManager.begin();
}

void loop() {
  sensorManager.updateWithSerial();

  // Do other things here, e.g., PID control or motor updates
}
