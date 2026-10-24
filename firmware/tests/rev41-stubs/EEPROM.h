#pragma once
#include <cstring>
struct FakeEEPROM {
  void begin(unsigned) {}
  template<class T> void get(int, T& value) { memset(&value, 0, sizeof(value)); }
  template<class T> void put(int, const T&) {}
  bool commit() { return true; }
};
inline FakeEEPROM EEPROM;
