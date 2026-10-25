// Host-side stand-in for the Arduino core, used only by the firmware tests.
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define strcmp_P strcmp
#define strcpy_P strcpy
#define strncpy_P strncpy
#define strcat_P strcat
#define _BV(b) (1u << (b))
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define ISR(vec) void vec##_handler()

extern volatile uint8_t PORTB;
extern volatile uint8_t TCCR1A, TCCR1B, TIMSK1;
extern volatile uint16_t OCR1A;
#define WGM12 3
#define CS11 1
#define OCIE1A 1

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
char* dtostrf(double val, signed char width, unsigned char prec, char* out);
char* ltoa(long val, char* out, int base);

class HardwareSerial {
 public:
  void begin(unsigned long) {}
  int available();
  int read();
  size_t write(uint8_t c);
  int availableForWrite();
};
extern HardwareSerial Serial;
