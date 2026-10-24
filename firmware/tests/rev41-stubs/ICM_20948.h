#pragma once
#include <Wire.h>
constexpr int ICM_20948_Stat_Ok = 0;
struct ICM_20948_I2C {
  int status = ICM_20948_Stat_Ok;
  void begin(FakeWire&, unsigned char) {}
  bool dataReady() { return false; }
  void getAGMT() {}
  const char* statusString() { return "OK"; }
  float temp() { return 20; }
  float accX() { return 0; }
  float accY() { return 0; }
  float accZ() { return 1000; }
  float magX() { return 1; }
  float magY() { return 0; }
  float magZ() { return 0; }
  float gyrZ() { return 0; }
};
