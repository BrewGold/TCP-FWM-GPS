#pragma once
#include <algorithm>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

using byte = uint8_t;
using std::min;
constexpr int LOW = 0, HIGH = 1, OUTPUT = 1, SERIAL_8N1 = 0;
inline uint32_t testMillis = 0;
inline int pinStates[32] = {};
inline std::vector<std::pair<int, int>> pinWrites;
inline uint32_t millis() { return testMillis; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int state) {
  pinStates[pin] = state;
  pinWrites.emplace_back(pin, state);
}
inline void delayMicroseconds(unsigned int) {}
void delay(unsigned long);

struct TestSerial {
  std::deque<char> input;
  std::string output;
  void begin(unsigned long) {}
  void setFIFOSize(int) {}
  int available() { return static_cast<int>(input.size()); }
  int read() {
    char c = input.front();
    input.pop_front();
    return static_cast<unsigned char>(c);
  }
  void print(const char* s) { output += s; }
  template<class T> void print(const T&) {}
  template<class T> void println(const T& value) { print(value); output += '\n'; }
  void println() { output += '\n'; }
  void feed(const std::string& s) { input.insert(input.end(), s.begin(), s.end()); }
};
inline TestSerial Serial, Serial1;
