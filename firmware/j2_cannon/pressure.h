// Pressure sensing, compressor control, fault latching and shot detection.
#pragma once
#include <Arduino.h>

enum PState : uint8_t { PS_IDLE = 0, PS_FILL = 1, PS_HOLD = 2, PS_MANUAL = 3 };

enum Fault : uint8_t {
  F_NONE = 0,
  F_OVERPRESSURE = 1,
  F_SENSOR_LOW = 2,
  F_SENSOR_HIGH = 3,
  F_FILL_STALL = 4,
  F_FILL_TIMEOUT = 5,
};

void pressureBegin();
void pressureUpdate(uint32_t nowMs);   // call every 10 ms

bool pressureStartAuto(float psi);     // fill to psi, then hold there
void pressureStop();                   // compressor off, back to idle
void pressureManual(bool held);        // joystick button held / released
bool pressureClearFault();             // false if the cause is still present
void pressureLinkLost();               // cancel anything the Pi started

float pressurePsi();
float pressureTarget();
uint16_t pressureAdc();
PState pressureState();
bool compressorOn();
uint8_t pressureFault();

// One-shot event flags for the protocol layer.
bool pressureTakeShot(float& prePsi, float& postPsi);
uint8_t pressureTakeNewFault();

const char* faultNameP(uint8_t code);  // PROGMEM
const char* pstateNameP(PState s);     // PROGMEM
