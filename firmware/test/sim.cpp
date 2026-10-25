#include "sim.h"
#include <EEPROM.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "config.h"
#include "settings.h"
#include "stepper.h"

Sim sim;
volatile uint8_t PORTB;
volatile uint8_t TCCR1A, TCCR1B, TIMSK1;
volatile uint16_t OCR1A;
HardwareSerial Serial;
TwoWire Wire;
EEPROMClass EEPROM;

static const uint8_t PUL_MASK[2] = {_BV(5), _BV(2)};
static const uint8_t DIR_PIN[2] = {PIN_YAW_DIR, PIN_PITCH_DIR};

static void violation(const std::string& msg) {
  if (sim.violations.size() < 20) sim.violations.push_back(msg + " at t=" + std::to_string(sim.t) + "us");
}

static void isrTick() {
  uint8_t before = PORTB;
  long pos0[2] = {axisPosSteps(0), axisPosSteps(1)};
  TIMER1_COMPA_vect_handler();
  uint8_t after = PORTB;
  for (int i = 0; i < 2; i++) {
    long pos1 = axisPosSteps(i);
    bool rise = !(before & PUL_MASK[i]) && (after & PUL_MASK[i]);
    long d = pos1 - pos0[i];
    if (rise) {
      sim.pulses[i]++;
      if (d != 1 && d != -1) violation("pulse without a single position step");
      bool dirHigh = sim.pin[DIR_PIN[i]] == HIGH;
      bool expectHigh = (d > 0) != (g_set.ax[i].inv != 0);
      if (dirHigh != expectHigh) violation("DIR pin does not match step direction");
      if (sim.t - sim.lastDirChange[i] < 5) violation("DIR setup time < 5 us");
      if (sim.lastPulse[i]) {
        uint64_t gap = sim.t - sim.lastPulse[i];
        if (gap < sim.minPulseGap[i]) sim.minPulseGap[i] = gap;
        if (gap < 100) violation("step rate above 10 kHz");
      }
      sim.lastPulse[i] = sim.t;
      sim.pulseTimes[i].push_back(sim.t);
    } else if (d != 0) {
      violation("position changed without a pulse");
    }
    if (pos1 < sim.minPos[i]) sim.minPos[i] = pos1;
    if (pos1 > sim.maxPos[i]) sim.maxPos[i] = pos1;
  }
}

void advance(uint64_t us) {
  uint64_t end = sim.t + us;
  while (sim.t < end) {
    uint64_t stepEnd = sim.nextTick < end ? sim.nextTick : end;
    double sec = (stepEnd - sim.t) * 1e-6;
    if (sim.pin[PIN_COMPRESSOR]) {
      sim.psi += sim.fillRate * sec;
      sim.compOnUs += stepEnd - sim.t;
    }
    sim.psi -= sim.leakRate * sec;
    if (sim.psi < 0) sim.psi = 0;
    sim.t = stepEnd;
    if (sim.t == sim.nextTick) {
      sim.nextTick += 50;
      isrTick();
    }
  }
  if (sim.wdtOn && sim.t - sim.wdtLast > sim.maxLoopGap) sim.maxLoopGap = sim.t - sim.wdtLast;
}

void runMs(double ms) {
  uint64_t end = sim.t + (uint64_t)(ms * 1000);
  while (sim.t < end) {
    if (sim.autoHb && sim.t - sim.lastHb >= 250000) {
      sim.lastHb = sim.t;
      send("HB");
    }
    loop();
    advance(20);  // loop overhead
  }
}

void send(const std::string& line) { sim.rx += line + "\n"; }
size_t txMark() { return sim.tx.size(); }

bool sawSince(size_t mark, const std::string& prefix) {
  size_t p = mark;
  while (p < sim.tx.size()) {
    size_t e = sim.tx.find('\n', p);
    if (e == std::string::npos) e = sim.tx.size();
    if (sim.tx.compare(p, prefix.size(), prefix) == 0) return true;
    p = e + 1;
  }
  return false;
}

std::string lastStatusField(const std::string& key) {
  size_t s = sim.tx.rfind("\nS ");
  if (s == std::string::npos) return "";
  size_t e = sim.tx.find('\n', s + 1);
  if (e == std::string::npos) return "";  // incomplete line: take the previous one
  std::string line = sim.tx.substr(s + 1, e - s - 1);
  std::string k = " " + key + "=";
  size_t p = line.find(k);
  if (p == std::string::npos) return "";
  p += k.size();
  size_t q = line.find(' ', p);
  return line.substr(p, q == std::string::npos ? std::string::npos : q - p);
}

double statusNum(const std::string& key) { return atof(lastStatusField(key).c_str()); }

// ------------------------------------------------------------ Arduino API

void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t p, uint8_t v) {
  if (p >= 20) return;
  for (int i = 0; i < 2; i++) {
    if (p == DIR_PIN[i] && sim.pin[p] != v) {
      sim.lastDirChange[i] = sim.t;
      if (PORTB & PUL_MASK[i]) violation("DIR changed while PUL high");
    }
  }
  if (p == PIN_COMPRESSOR && v && !sim.pin[p]) sim.compStarts++;
  sim.pin[p] = v;
  if (p >= 8 && p <= 13) {
    if (v) PORTB |= _BV(p - 8);
    else PORTB &= ~_BV(p - 8);
  }
}

int digitalRead(uint8_t p) {
  if (p == PIN_JOY_SW) return sim.joyBtn ? LOW : HIGH;
  return p < 20 ? sim.pin[p] : 0;
}

int analogRead(uint8_t p) {
  advance(112);
  if (p == PIN_PRESSURE) {
    if (sim.adcOverride >= 0) return sim.adcOverride;
    double frac = 0.1 + 0.8 * sim.psi / 80.0;
    if (frac > 0.95) frac = 0.95;  // sensor output saturates
    return (int)lround(frac * 1023);
  }
  if (p == PIN_JOY_X) return sim.joyX;
  if (p == PIN_JOY_Y) return sim.joyY;
  return 0;
}

unsigned long millis() { return (unsigned long)(sim.t / 1000); }
unsigned long micros() { return (unsigned long)sim.t; }
void delay(unsigned long ms) { advance((uint64_t)ms * 1000); }
void delayMicroseconds(unsigned int us) { advance(us); }

char* dtostrf(double val, signed char width, unsigned char prec, char* out) {
  sprintf(out, "%*.*f", width, prec, val);
  return out;
}

char* ltoa(long val, char* out, int) {
  sprintf(out, "%ld", val);
  return out;
}

int HardwareSerial::available() { return (int)(sim.rx.size() - sim.rxPos); }
int HardwareSerial::read() { return sim.rxPos < sim.rx.size() ? (uint8_t)sim.rx[sim.rxPos++] : -1; }
size_t HardwareSerial::write(uint8_t c) {
  sim.tx.push_back((char)c);
  return 1;
}
int HardwareSerial::availableForWrite() { return 63; }

static uint8_t s_wireAddr;
static int s_wireBytes;
void TwoWire::beginTransmission(uint8_t a) {
  s_wireAddr = a;
  s_wireBytes = 1;
}
size_t TwoWire::write(uint8_t) {
  s_wireBytes++;
  return 1;
}
uint8_t TwoWire::endTransmission() {
  advance(s_wireBytes * 90 + 20);  // 100 kHz I2C
  return (sim.lcdAttached && s_wireAddr == 0x27) ? 0 : 2;
}

void wdt_enable(int) {
  sim.wdtOn = true;
  sim.wdtLast = sim.t;
}
void wdt_disable() { sim.wdtOn = false; }
void wdt_reset() {
  if (sim.wdtOn && sim.t - sim.wdtLast > sim.maxLoopGap) sim.maxLoopGap = sim.t - sim.wdtLast;
  sim.wdtLast = sim.t;
}
