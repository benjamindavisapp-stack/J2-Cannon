// Line-based serial I/O with a TX queue, so status lines never block the loop.
#pragma once
#include <Arduino.h>

typedef void (*LineHandler)(char* line);

void ioBegin(unsigned long baud);
void ioPoll(LineHandler handler);  // read bytes, call handler per complete line
void ioPump();                     // move queued bytes into Serial without blocking
uint8_t ioFree();                  // bytes free in the TX queue

void ioPut(char c);                // blocks only if the queue is full
void ioStr(const char* s);
void ioStrP(const char* p);        // PROGMEM string
void ioFloat(float v, uint8_t prec);
void ioInt(long v);
void ioEol();
void ioFlush();                    // block until everything queued has gone out

// Small fixed buffer for composing a line before queueing it.
struct LineBuf {
  char buf[150];
  uint8_t len;
  void clear() { len = 0; buf[0] = 0; }
  void str(const char* s);
  void strP(const char* p);
  void flt(float v, uint8_t prec);
  void num(long v);
  void ch(char c);
};
bool ioQueueLine(const LineBuf& line);  // false (dropped) if it does not fit
