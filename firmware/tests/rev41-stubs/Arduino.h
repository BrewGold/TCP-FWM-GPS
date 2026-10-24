#pragma once
#include "../stubs/Arduino.h"
inline uint32_t micros() { return testMillis * 1000; }
