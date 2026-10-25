// Two stepper axes driven from a 20 kHz Timer1 interrupt.
//
// The ISR only emits step pulses at the rate it is given and refuses to step
// outside a [lo, hi] window. The planner (steppersPlan, every 2 ms from loop)
// does the trapezoidal speed profile, soft limits and position targets.
#pragma once
#include <Arduino.h>

enum AxisMode : uint8_t { AXM_HOLD = 0, AXM_POSITION = 1, AXM_VELOCITY = 2 };

void steppersBegin();
void steppersPlan();               // call every loop
void steppersEnable(bool on);      // disabling also clears both zeros
bool steppersEnabled();

bool axisMoveTo(uint8_t axis, float deg);  // false if not zeroed/enabled or outside limits
void axisJog(uint8_t axis, float frac);    // -1..1 of the jog speed
void axisStop(uint8_t axis);               // decelerate to a stop and hold
bool axisSetZero(uint8_t axis);            // only while stopped
bool axisZeroed(uint8_t axis);
bool axisMoving(uint8_t axis);
AxisMode axisMode(uint8_t axis);
float axisPosDeg(uint8_t axis);
float axisTargetDeg(uint8_t axis);
int32_t axisPosSteps(uint8_t axis);
