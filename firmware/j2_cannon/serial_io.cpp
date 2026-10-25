#include "serial_io.h"

static char s_tx[256];
static uint8_t s_head, s_tail;  // uint8_t indices wrap with the 256-byte buffer
static char s_rx[64];
static uint8_t s_rxLen;
static bool s_rxOverflow;

void ioBegin(unsigned long baud) { Serial.begin(baud); }

uint8_t ioFree() { return (uint8_t)(255 - (uint8_t)(s_head - s_tail)); }

void ioPump() {
  while (s_head != s_tail && Serial.availableForWrite() > 0) Serial.write((uint8_t)s_tx[s_tail++]);
}

void ioPut(char c) {
  if (ioFree() == 0) Serial.write((uint8_t)s_tx[s_tail++]);  // rare: wait on the UART
  s_tx[s_head++] = c;
}

void ioStr(const char* s) {
  while (*s) ioPut(*s++);
}

void ioStrP(const char* p) {
  char c;
  while ((c = pgm_read_byte(p++))) ioPut(c);
}

void ioFloat(float v, uint8_t prec) {
  char b[16];
  dtostrf(v, 0, prec, b);
  ioStr(b);
}

void ioInt(long v) {
  char b[12];
  ltoa(v, b, 10);
  ioStr(b);
}

void ioEol() { ioPut('\n'); }

void ioPoll(LineHandler handler) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (s_rxLen && !s_rxOverflow) {
        s_rx[s_rxLen] = 0;
        handler(s_rx);
      }
      s_rxLen = 0;
      s_rxOverflow = false;
    } else if (s_rxLen < sizeof(s_rx) - 1) {
      s_rx[s_rxLen++] = c;
    } else {
      s_rxOverflow = true;  // drop the whole over-long line
    }
  }
}

void LineBuf::ch(char c) {
  if (len < sizeof(buf) - 1) {
    buf[len++] = c;
    buf[len] = 0;
  }
}

void LineBuf::str(const char* s) {
  while (*s) ch(*s++);
}

void LineBuf::strP(const char* p) {
  char c;
  while ((c = pgm_read_byte(p++))) ch(c);
}

void LineBuf::flt(float v, uint8_t prec) {
  char b[16];
  dtostrf(v, 0, prec, b);
  str(b);
}

void LineBuf::num(long v) {
  char b[12];
  ltoa(v, b, 10);
  str(b);
}

bool ioQueueLine(const LineBuf& line) {
  if (ioFree() < line.len + 1) return false;
  for (uint8_t i = 0; i < line.len; i++) s_tx[s_head++] = line.buf[i];
  s_tx[s_head++] = '\n';
  return true;
}
