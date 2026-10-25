#pragma once
#include <Arduino.h>
struct EEPROMClass {
  uint8_t data[1024];
  EEPROMClass() { memset(data, 0xFF, sizeof(data)); }
  template <class T> T& get(int addr, T& t) { memcpy(&t, data + addr, sizeof(T)); return t; }
  template <class T> const T& put(int addr, const T& t) { memcpy(data + addr, &t, sizeof(T)); return t; }
};
extern EEPROMClass EEPROM;
