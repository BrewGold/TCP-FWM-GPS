#pragma once
#include "Arduino.h"
#include <array>
constexpr int MAX_SOCK_NUM = 8, EthernetNoHardware = 0, LinkON = 1;
struct IPAddress {
  std::array<int, 4> bytes;
  IPAddress(int a, int b, int c, int d) : bytes{a, b, c, d} {}
  bool operator==(const IPAddress& other) const { return bytes == other.bytes; }
};
// Deliberately no network I/O, stop(), connected(), server, or availableForWrite:
// using Ethernet's blocking/cached socket path must fail to compile.
class EthernetClient {
  uint8_t socket = MAX_SOCK_NUM;
public:
  int timeout = 1000;
  EthernetClient() = default;
  explicit EthernetClient(uint8_t s) : socket(s) {}
  uint8_t getSocketNumber() const { return socket; }
  void setConnectionTimeout(int value) { timeout = value; }
};
struct FakeEthernet {
  int hardware = 1, link = LinkON;
  void init(int) {}
  int begin(byte*, int, int) { return 1; }
  void begin(byte*, IPAddress, IPAddress, IPAddress, IPAddress) {}
  int hardwareStatus() { return hardware; }
  int linkStatus() { return link; }
  IPAddress localIP() { return IPAddress(192, 168, 1, 22); }
  IPAddress dnsServerIP() { return IPAddress(192, 168, 1, 1); }
  IPAddress gatewayIP() { return dnsServerIP(); }
  IPAddress subnetMask() { return IPAddress(255, 255, 255, 0); }
};
inline FakeEthernet Ethernet;
