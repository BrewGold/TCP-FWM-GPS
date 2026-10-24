#pragma once
#include <Ethernet.h>
#include <utility/w5100.h>
#include "TcpTransport.h"

// Verified against Arduino Ethernet 2.0.2 (d5672b707a85874bf69a312b8dce11df710a14d3).
// Do not mix this driver with Ethernet socket I/O: its private RX/TX cache would
// become stale. EthernetClient is used only as a socket-number handle.
struct W5500TransportIO {
  static void begin() { SPI.beginTransaction(SPI_ETHERNET_SETTINGS); }
  static void end() { SPI.endTransaction(); }
  static size_t bufferSize() { return W5100.SSIZE; }
  static uint8_t read8(uint8_t s, Rev41::Reg8 reg) {
    switch (reg) {
      case Rev41::Status: return W5100.readSnSR(s);
      case Rev41::Command: return W5100.readSnCR(s);
      case Rev41::Interrupt: return W5100.readSnIR(s);
      default: return W5100.readSnMR(s);
    }
  }
  static void write8(uint8_t s, Rev41::Reg8 reg, uint8_t value) {
    switch (reg) {
      case Rev41::Command: W5100.writeSnCR(s, value); break;
      case Rev41::Interrupt: W5100.writeSnIR(s, value); break;
      case Rev41::Mode: W5100.writeSnMR(s, value); break;
      default: break;
    }
  }
  static uint16_t read16(uint8_t s, Rev41::Reg16 reg) {
    switch (reg) {
      case Rev41::Free: return W5100.readSnTX_FSR(s);
      case Rev41::TxWrite: return W5100.readSnTX_WR(s);
      case Rev41::RxSize: return W5100.readSnRX_RSR(s);
      case Rev41::RxRead: return W5100.readSnRX_RD(s);
      default: return W5100.readSnPORT(s);
    }
  }
  static void write16(uint8_t s, Rev41::Reg16 reg, uint16_t value) {
    switch (reg) {
      case Rev41::TxWrite: W5100.writeSnTX_WR(s, value); break;
      case Rev41::RxRead: W5100.writeSnRX_RD(s, value); break;
      case Rev41::Port: W5100.writeSnPORT(s, value); break;
      default: break;
    }
  }
  static void writeTx(uint8_t s, uint16_t pointer, const uint8_t* data, size_t length) {
    uint16_t offset = pointer & W5100.SMASK;
    size_t first = W5100.SSIZE - offset;
    if (first > length) first = length;
    // Split even on W5500: the library's virtual address maps socket selection
    // from address bits. Never let a transfer spill into another socket's block.
    W5100.write(W5100.SBASE(s) + offset, data, first);
    if (length > first) W5100.write(W5100.SBASE(s), data + first, length - first);
  }
  static void readRx(uint8_t s, uint16_t pointer, uint8_t* data, size_t length) {
    uint16_t offset = pointer & W5100.SMASK;
    size_t first = W5100.SSIZE - offset;
    if (first > length) first = length;
    W5100.read(W5100.RBASE(s) + offset, data, first);
    if (length > first) W5100.read(W5100.RBASE(s), data + first, length - first);
  }
  static bool resetChip() {
    {
      Rev41::Transaction<W5500TransportIO> transaction;
      W5100.writeMR(0x80);
    }
    // Explicit whole-chip generation boundary; yield never accesses SPI.
    delay(2);
    Rev41::Transaction<W5500TransportIO> transaction;
    return !(W5100.readMR() & 0x80);
  }
};
