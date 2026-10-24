#pragma once
#include <cassert>
struct FakeSPI {
  bool active = false;
  unsigned transactions = 0;
  void beginTransaction(int) {
    assert(!active);
    active = true;
    ++transactions;
  }
  void endTransaction() { assert(active); active = false; }
};
inline FakeSPI SPI;
#define SPI_ETHERNET_SETTINGS 0
