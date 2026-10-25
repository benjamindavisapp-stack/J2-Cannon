#include "joystick.h"
#include "config.h"
#include "settings.h"

static const int DEADBAND = 70;          // ADC counts around centre
static const uint32_t HOLD_MS = 400;
static const uint32_t LONG_MS = 3000;
static const uint32_t DOUBLE_GAP_MS = 400;
static const uint32_t RELEASE_MS = 100;  // stick must rest this long to count as released

static int s_cx = 512, s_cy = 512;
static float s_x, s_y;
static bool s_deflected;
static uint32_t s_restSince;

static bool s_rawBtn, s_btn;
static uint8_t s_btnStable;
static uint32_t s_downAt, s_lastClickAt;
static bool s_holdActive, s_longFired;
static uint8_t s_clicks;

static int calibrate(uint8_t pin) {
  long sum = 0;
  for (uint8_t i = 0; i < 16; i++) sum += analogRead(pin);
  int c = (int)(sum / 16);
  return (c < 400 || c > 624) ? 512 : c;  // stick held at power-up: fall back to nominal
}

void joyBegin() {
  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  s_cx = calibrate(PIN_JOY_X);
  s_cy = calibrate(PIN_JOY_Y);
}

static float shape(int raw, int centre, bool inv) {
  int d = raw - centre;
  int span = d > 0 ? 1023 - centre : centre;
  int m = abs(d);
  if (m <= DEADBAND || span <= DEADBAND) return 0;
  float f = (float)(m - DEADBAND) / (float)(span - DEADBAND);
  if (f > 1) f = 1;
  f = 0.25f * f + 0.75f * f * f * f;  // fine control near centre
  if (d < 0) f = -f;
  return inv ? -f : f;
}

uint8_t joyUpdate(uint32_t now) {
  uint8_t ev = 0;
  int rx = analogRead(PIN_JOY_X);
  int ry = analogRead(PIN_JOY_Y);
  s_x = shape(rx, s_cx, g_set.joyInvX);
  s_y = shape(ry, s_cy, g_set.joyInvY);
  if (s_x != 0 || s_y != 0) {
    s_deflected = true;
    s_restSince = now;
  } else if (s_deflected && now - s_restSince >= RELEASE_MS) {
    s_deflected = false;
  }

  // Button: debounce over two samples (20 ms).
  bool raw = digitalRead(PIN_JOY_SW) == LOW;
  if (raw != s_rawBtn) {
    s_rawBtn = raw;
    s_btnStable = 0;
  } else if (s_btnStable < 2 && ++s_btnStable == 2 && raw != s_btn) {
    s_btn = raw;
    if (raw) {
      s_downAt = now;
      s_holdActive = false;
      s_longFired = false;
    } else if (s_holdActive) {
      ev |= JOY_EV_HOLD_END;
      s_clicks = 0;
    } else {
      s_clicks++;
      s_lastClickAt = now;
      if (s_clicks >= 2) {
        ev |= JOY_EV_DOUBLE;
        s_clicks = 0;
      }
    }
  }
  if (s_btn) {
    if (!s_holdActive && now - s_downAt >= HOLD_MS) {
      s_holdActive = true;
      s_clicks = 0;
      ev |= JOY_EV_HOLD_START;
    }
    if (!s_longFired && now - s_downAt >= LONG_MS) {
      s_longFired = true;
      ev |= JOY_EV_LONG;
    }
  } else if (s_clicks == 1 && now - s_lastClickAt > DOUBLE_GAP_MS) {
    s_clicks = 0;
    ev |= JOY_EV_CLICK;
  }
  return ev;
}

float joyX() { return s_deflected ? s_x : 0; }
float joyY() { return s_deflected ? s_y : 0; }
bool joyDeflected() { return s_deflected; }
bool joyButtonDown() { return s_btn; }
