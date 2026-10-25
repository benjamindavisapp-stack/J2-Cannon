#include "lcd.h"
#include <Wire.h>

// Common PCF8574 backpack wiring: P0=RS P1=RW P2=EN P3=backlight P4..P7=D4..D7
static const uint8_t RS = 0x01;
static const uint8_t EN = 0x04;
static const uint8_t BL = 0x08;

static uint8_t s_addr;
static bool s_ok;
static bool s_stuck;
static char s_want[2][16];
static char s_shown[2][16];
static uint8_t s_cursorRow = 255, s_cursorCol = 255;
static uint8_t s_scan;  // next cell to compare in lcdPump

static void sendNibbles(uint8_t value, uint8_t mode) {
  uint8_t hi = (value & 0xF0) | mode | BL;
  uint8_t lo = (uint8_t)((value << 4) & 0xF0) | mode | BL;
  Wire.beginTransmission(s_addr);
  Wire.write(hi | EN);
  Wire.write(hi);
  Wire.write(lo | EN);
  Wire.write(lo);
  if (Wire.endTransmission() != 0) s_ok = false;  // LCD unplugged: stop talking to it
}

static void command(uint8_t c) { sendNibbles(c, 0); }

static void init4bit() {
  // HD44780 reset-by-instruction sequence (datasheet figure 24).
  uint8_t steps[3] = {0x30, 0x30, 0x30};
  for (uint8_t i = 0; i < 3; i++) {
    Wire.beginTransmission(s_addr);
    Wire.write(steps[i] | EN | BL);
    Wire.write(steps[i] | BL);
    Wire.endTransmission();
    delay(5);
  }
  Wire.beginTransmission(s_addr);
  Wire.write(0x20 | EN | BL);  // switch to 4-bit
  Wire.write(0x20 | BL);
  Wire.endTransmission();
  delay(1);
  command(0x28);  // 4-bit, 2 lines, 5x8
  command(0x0C);  // display on, no cursor
  command(0x06);  // entry mode: increment
  command(0x01);  // clear
  delay(3);
}

bool lcdBegin() {
  // A shorted or unpowered I2C bus can hang Wire on older Arduino cores, so
  // check both lines idle high (pulled up) before touching it.
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(20);
  s_stuck = digitalRead(SDA) == LOW || digitalRead(SCL) == LOW;
  if (s_stuck) {
    s_ok = false;
    return false;
  }
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // a glitched bus must not hang the loop
#endif
  static const uint8_t candidates[] = {0x27, 0x3F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
                                       0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E};
  s_ok = false;
  for (uint8_t i = 0; i < sizeof(candidates); i++) {
    Wire.beginTransmission(candidates[i]);
    if (Wire.endTransmission() == 0) {
      s_addr = candidates[i];
      s_ok = true;
      break;
    }
  }
  if (!s_ok) return false;
  delay(50);
  init4bit();
  memset(s_shown, ' ', sizeof(s_shown));  // matches the cleared display; s_want is redrawn
  if (s_want[0][0] == 0) memset(s_want, ' ', sizeof(s_want));
  s_cursorRow = s_cursorCol = 255;
  return s_ok;
}

bool lcdPresent() { return s_ok; }
bool lcdBusStuck() { return s_stuck; }
uint8_t lcdAddress() { return s_addr; }

void lcdSetLine(uint8_t row, const char* text) {
  if (row > 1) return;
  uint8_t i = 0;
  for (; i < 16 && text[i]; i++) s_want[row][i] = text[i];
  for (; i < 16; i++) s_want[row][i] = ' ';
}

void lcdPump() {
  if (!s_ok) return;
  for (uint8_t n = 0; n < 32; n++) {
    uint8_t cell = s_scan;
    uint8_t row = cell >> 4, col = cell & 15;
    if (s_want[row][col] == s_shown[row][col]) {
      s_scan = (cell + 1) & 31;
      continue;
    }
    if (row != s_cursorRow || col != s_cursorCol) {
      command(0x80 | (row ? 0x40 : 0x00) | col);
      s_cursorRow = row;
      s_cursorCol = col;
      return;  // one I2C transaction per call; the character goes next call
    }
    sendNibbles(s_want[row][col], RS);
    s_shown[row][col] = s_want[row][col];
    s_cursorCol++;
    s_scan = (cell + 1) & 31;
    return;
  }
}
