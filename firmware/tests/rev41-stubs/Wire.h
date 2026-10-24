#pragma once
struct FakeWire {
  void begin() {}
  void setClock(int) {}
  void beginTransmission(unsigned char) {}
  int endTransmission() { return 0; }
};
inline FakeWire Wire;
