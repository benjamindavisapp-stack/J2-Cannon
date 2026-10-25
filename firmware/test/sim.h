// Simulated RedBoard for running the real firmware sources on a PC.
//
// Time only moves when the firmware calls something that takes time
// (analogRead, I2C, delays) or when the test advances it. Every 50 us of
// simulated time the Timer1 ISR runs, and every step pulse it produces is
// checked against the driver timing rules and the soft limits.
#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <string>
#include <vector>

void setup();
void loop();
void TIMER1_COMPA_vect_handler();

struct Sim {
  // Plant
  double psi = 0;          // true chamber pressure
  double fillRate = 4.0;   // psi/s while the compressor runs
  double leakRate = 0.0;   // psi/s
  int adcOverride = -1;    // force the pressure ADC reading
  int joyX = 512, joyY = 512;
  bool joyBtn = false;
  bool lcdAttached = true;

  // Clock and I/O
  uint64_t t = 0;          // microseconds
  uint64_t nextTick = 50;
  uint8_t pin[20] = {0};
  std::string rx;          // bytes waiting for the firmware
  size_t rxPos = 0;
  std::string tx;          // everything the firmware sent
  bool autoHb = true;
  uint64_t lastHb = 0;

  // Watchdog
  bool wdtOn = false;
  uint64_t wdtLast = 0;
  uint64_t maxLoopGap = 0;

  // Step pulse checks (index 0 = yaw, 1 = pitch)
  long pulses[2] = {0, 0};
  uint64_t lastPulse[2] = {0, 0};
  uint64_t lastDirChange[2] = {0, 0};
  uint64_t minPulseGap[2] = {UINT64_MAX, UINT64_MAX};
  long minPos[2] = {0, 0}, maxPos[2] = {0, 0};
  std::vector<std::string> violations;
  std::vector<uint64_t> pulseTimes[2];

  // Compressor
  uint64_t compOnUs = 0;   // total on-time
  int compStarts = 0;
};

extern Sim sim;

void advance(uint64_t us);
void runMs(double ms);                 // run loop() for this long
void send(const std::string& line);
size_t txMark();                       // position in sim.tx for later searches
bool sawSince(size_t mark, const std::string& prefix);
std::string lastStatusField(const std::string& key);
double statusNum(const std::string& key);
