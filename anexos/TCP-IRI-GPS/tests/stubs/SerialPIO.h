#pragma once
#include "Arduino.h"
constexpr int NOPIN = -1;
struct SerialPIO {
  std::string output;
  SerialPIO(int, int) {}
  void begin(unsigned long, int) {}
  explicit operator bool() const { return true; }
  int availableForWrite() { return 8; }
  size_t write(uint8_t c) { output += static_cast<char>(c); return 1; }
};
