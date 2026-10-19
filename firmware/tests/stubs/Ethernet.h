#pragma once
#include "Arduino.h"
#include <array>
constexpr int MAX_SOCK_NUM = 8, EthernetNoHardware = 0, LinkON = 1;
struct IPAddress {
  std::array<int, 4> bytes;
  IPAddress(int a, int b, int c, int d) : bytes{a, b, c, d} {}
  bool operator==(const IPAddress& other) const { return bytes == other.bytes; }
};
struct EthernetClient : TestSerial {
  bool active = false;
  EthernetClient() = default;
  explicit EthernetClient(int) : active(true) {}
  explicit operator bool() const { return active; }
  bool connected() const { return active; }
  void stop() { active = false; }
  void setConnectionTimeout(int) {}
  int availableForWrite() { return 2048; }
  size_t write(const uint8_t* data, size_t size) {
    output.append(reinterpret_cast<const char*>(data), size);
    return size;
  }
};
struct EthernetServer {
  inline static uint16_t server_port[MAX_SOCK_NUM] = {};
  bool listening = false;
  explicit EthernetServer(int) {}
  void begin() { listening = true; }
  explicit operator bool() const { return listening; }
  EthernetClient accept() { return EthernetClient(); }
};
struct TestEthernet {
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
inline TestEthernet Ethernet;
