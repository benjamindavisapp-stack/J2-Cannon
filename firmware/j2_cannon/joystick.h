// Analog joystick (VRx, VRy) and its push button.
#pragma once
#include <Arduino.h>

enum {
  JOY_EV_CLICK = 1,       // single short press
  JOY_EV_DOUBLE = 2,      // two short presses
  JOY_EV_HOLD_START = 4,  // held past HOLD_MS
  JOY_EV_HOLD_END = 8,    // released after a hold
  JOY_EV_LONG = 16,       // held past LONG_MS
};

void joyBegin();                 // samples the resting centre - keep hands off at power-up
uint8_t joyUpdate(uint32_t now); // call every 10 ms, returns JOY_EV_* bits
float joyX();                    // -1..1 after deadband and expo
float joyY();
bool joyDeflected();
bool joyButtonDown();
