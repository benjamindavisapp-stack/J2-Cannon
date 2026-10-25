// 16x2 HD44780 LCD on a PCF8574 I2C backpack.
//
// Writes are incremental: lcdPrint() only updates a frame buffer and
// lcdPump() sends at most one changed character per call, so the main loop
// never blocks on I2C for more than ~0.5 ms.
#pragma once
#include <Arduino.h>

bool lcdBegin();                              // probes the usual addresses, false if no LCD
bool lcdPresent();
uint8_t lcdAddress();
void lcdSetLine(uint8_t row, const char* text);  // pads/truncates to 16 chars
void lcdPump();
