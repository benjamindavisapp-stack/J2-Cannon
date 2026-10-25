#pragma once
#include <Arduino.h>
#define WIRE_HAS_TIMEOUT 1
class TwoWire {
 public:
  void begin() {}
  void setClock(uint32_t) {}
  void setWireTimeout(uint32_t, bool) {}
  void beginTransmission(uint8_t addr);
  size_t write(uint8_t b);
  uint8_t endTransmission();
};
extern TwoWire Wire;
