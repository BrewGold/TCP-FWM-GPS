#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <array>
#include <cstring>

struct FakeW5500 {
  static constexpr uint16_t SSIZE = 2048, SMASK = 2047;
  struct Socket {
    uint8_t status = 0, command = 0, interrupt = 0, mode = 0;
    uint16_t free = SSIZE, txWrite = 0, txRead = 0, rxRead = 0, rxConsumed = 0;
    uint16_t rxWrite = 0, rxSize = 0, port = 0;
    bool holdCommand = false, ackSend = true, unstableFree = false;
    std::array<uint8_t, SSIZE> tx{}, rx{};
    std::vector<std::string> sends;
  };
  std::array<Socket, 8> sockets;
  unsigned reads = 0, writes = 0;
  static uint16_t SBASE(uint8_t s) { return 0x8000 + s * SSIZE; }
  static uint16_t RBASE(uint8_t s) { return 0xc000 + s * SSIZE; }
  static uint8_t getChip() { return 55; }
  void check() { assert(SPI.active); }
  uint8_t readSnSR(uint8_t s) { check(); ++reads; return sockets[s].status; }
  uint8_t readSnCR(uint8_t s) { check(); ++reads; return sockets[s].command; }
  uint8_t readSnIR(uint8_t s) { check(); ++reads; return sockets[s].interrupt; }
  uint8_t readSnMR(uint8_t s) { check(); ++reads; return sockets[s].mode; }
  uint16_t readSnTX_FSR(uint8_t s) {
    check(); ++reads;
    return sockets[s].free + (sockets[s].unstableFree ? (reads & 1) : 0);
  }
  uint16_t readSnTX_WR(uint8_t s) { check(); ++reads; return sockets[s].txWrite; }
  uint16_t readSnRX_RSR(uint8_t s) { check(); ++reads; return sockets[s].rxSize; }
  uint16_t readSnRX_RD(uint8_t s) { check(); ++reads; return sockets[s].rxRead; }
  uint16_t readSnPORT(uint8_t s) { check(); ++reads; return sockets[s].port; }
  void writeSnMR(uint8_t s, uint8_t v) { check(); ++writes; sockets[s].mode = v; }
  void writeSnIR(uint8_t s, uint8_t v) {
    check(); ++writes; sockets[s].interrupt &= ~v;
  }
  void writeSnTX_WR(uint8_t s, uint16_t v) { check(); ++writes; sockets[s].txWrite = v; }
  void writeSnRX_RD(uint8_t s, uint16_t v) { check(); ++writes; sockets[s].rxRead = v; }
  void writeSnPORT(uint8_t s, uint16_t v) { check(); ++writes; sockets[s].port = v; }
  void writeSnCR(uint8_t id, uint8_t v) {
    check(); ++writes;
    auto& s = sockets[id];
    s.command = v;
    if (s.holdCommand) return;
    s.command = 0;
    if (v == 1) { s.status = 0x13; s.txWrite = s.txRead = s.rxRead = s.rxConsumed = 0; }
    if (v == 2) s.status = 0x14;
    if (v == 0x10) { s.status = 0; s.interrupt = 0; }
    if (v == 0x20) {
      std::string data;
      while (s.txRead != s.txWrite) data += char(s.tx[s.txRead++ & SMASK]);
      s.sends.push_back(data);
      if (s.ackSend) s.interrupt |= 0x10;
    }
    if (v == 0x40) {
      s.rxSize -= uint16_t(s.rxRead - s.rxConsumed);
      s.rxConsumed = s.rxRead;
    }
  }
  void writeMR(uint8_t value) {
    check(); ++writes;
    if (value == 0x80) sockets = {};
  }
  uint8_t readMR() { check(); ++reads; return 0; }
  uint16_t write(uint16_t address, const uint8_t* data, uint16_t length) {
    check(); ++writes;
    assert(address >= 0x8000 && address < 0xc000);
    size_t socket = (address - 0x8000) / SSIZE, offset = address & SMASK;
    assert(offset + length <= SSIZE);
    memcpy(sockets[socket].tx.data() + offset, data, length);
    return length;
  }
  uint16_t read(uint16_t address, uint8_t* data, uint16_t length) {
    check(); ++reads;
    assert(address >= 0xc000);
    size_t socket = (address - 0xc000) / SSIZE, offset = address & SMASK;
    assert(offset + length <= SSIZE);
    memcpy(data, sockets[socket].rx.data() + offset, length);
    return length;
  }
  void feed(uint8_t id, const std::string& data) {
    auto& s = sockets[id];
    assert(s.rxSize + data.size() <= SSIZE);
    for (unsigned char c : data) s.rx[s.rxWrite++ & SMASK] = c;
    s.rxSize += data.size();
  }
};
inline FakeW5500 W5100;
