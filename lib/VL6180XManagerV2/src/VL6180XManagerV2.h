#ifndef VL6180XManagerV2_h
#define VL6180XManagerV2_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <vector>


class VL6180XManagerV2 {
public:
  VL6180XManagerV2(int* xshutPins, int sensorCount, int* corrections = nullptr);
  void begin();
  std::vector<int> readAll();

private:
  int* _xshutPins; //us uint8_t to avoid issues with large arrays(like memory issues)
  int _sensorCount;
  int _i2cBaseAddress = 0x29; // Base address for VL6180X sensors
  Adafruit_VL6180X* _sensors;
  int* _corrections;
};

#endif
