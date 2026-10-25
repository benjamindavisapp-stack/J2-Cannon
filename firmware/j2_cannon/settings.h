// Runtime settings: defaults from config.h, persisted in EEPROM, editable
// from the Pi with "SET <key> <value>" and "SAVE".
#pragma once
#include <Arduino.h>

enum { AX_YAW = 0, AX_PITCH = 1 };

struct AxisSettings {
  float spd;      // steps per degree
  float vmax;     // deg/s for auto moves
  float acc;      // deg/s^2
  float jog;      // deg/s at full stick
  float minDeg;   // soft limits, inside the hard limits
  float maxDeg;
  uint8_t inv;    // 1 = flip the DIR pin
};

struct Settings {
  uint16_t magic;
  uint8_t version;
  AxisSettings ax[2];
  uint8_t joyInvX;
  uint8_t joyInvY;
  float hyst;       // psi
  float pcomp;      // psi, fill overshoot compensation
  float uzjog;      // jog fraction before zeroing
  uint16_t stallSec;
  uint16_t maxRunSec;
  uint8_t crc;
};

extern Settings g_set;

void settingsDefaults();
bool settingsLoad();   // false if EEPROM was blank/corrupt (defaults used)
void settingsSave();
// Returns 0 on success, otherwise a short PROGMEM error string.
const char* settingsSet(const char* key, float val);
float hardMinDeg(uint8_t axis);
float hardMaxDeg(uint8_t axis);
