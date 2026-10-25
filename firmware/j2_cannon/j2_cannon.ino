// J2 Cannon - SparkFun RedBoard firmware.
//
// Owns everything timing- or safety-critical: step pulses, angle limits,
// compressor control and pressure cutoffs, joystick, LEDs and the LCD. The
// Raspberry Pi sends high-level commands over USB serial (see PROTOCOL.md);
// nothing the Pi sends can exceed the limits in config.h.
//
// Joystick: X jogs the turret, Y jogs the pitch.
//   hold button      run compressor while held (stops at the set pressure)
//   double-click     set zero for both axes at the current position
//   single click     switch LCD page
//   hold 3 s         clear a latched fault (only if its cause is gone)

#include <Arduino.h>
#include <avr/wdt.h>
#include "config.h"
#include "settings.h"
#include "stepper.h"
#include "pressure.h"
#include "joystick.h"
#include "lcd.h"
#include "serial_io.h"

enum { LED_OFF = 0, LED_RED = 1, LED_GREEN = 2 };

static uint32_t g_lastRxMs;
static bool g_link;
static bool g_joyActive;
static bool g_piJog;
static uint32_t g_piJogAt;
static uint8_t g_ledState = 255;
static uint8_t g_lcdPage;
static char g_lcdMsg[17];
static uint32_t g_lcdMsgUntil;
static uint32_t g_t10, g_tStatus, g_tLcd, g_tLcdRetry;
static bool g_fromEeprom;

// ------------------------------------------------------------------ helpers

static void ledSet(uint8_t c) {
  if (c == g_ledState) return;
  g_ledState = c;
  bool a = false, b = false;
  if (c == LED_RED) {
    a = LED_RED_IS_A;
    b = !LED_RED_IS_A;
  } else if (c == LED_GREEN) {
    a = !LED_RED_IS_A;
    b = LED_RED_IS_A;
  }
  digitalWrite(PIN_LED_A, a ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

static void lcdMessage(const char* p) {  // PROGMEM, shown on line 2 for 1.5 s
  strncpy_P(g_lcdMsg, p, 16);
  g_lcdMsg[16] = 0;
  g_lcdMsgUntil = millis() + 1500;
}

static bool anyMoving() { return axisMoving(AX_YAW) || axisMoving(AX_PITCH); }

static void stopAxes() {
  axisStop(AX_YAW);
  axisStop(AX_PITCH);
  g_piJog = false;
}

static void event(const char* p) {  // "E <p>"
  ioStrP(PSTR("E "));
  ioStrP(p);
  ioEol();
}

static void reply(bool ok, const char* cmd, const char* errP) {
  ioStrP(ok ? PSTR("OK ") : PSTR("ERR "));
  ioStr(cmd);
  if (!ok && errP) {
    ioPut(' ');
    ioStrP(errP);
  }
  ioEol();
}

static bool parseNum(const char* s, float& out) {
  if (!s) return false;
  char* end;
  double v = strtod(s, &end);
  if (end == s || *end) return false;
  out = (float)v;
  return true;
}

// ------------------------------------------------------------------ commands

static void sendConfig() {
  for (uint8_t i = 0; i < 2; i++) {
    const AxisSettings& a = g_set.ax[i];
    ioStrP(i == AX_YAW ? PSTR("CFG Y spd=") : PSTR("CFG P spd="));
    ioFloat(a.spd, 4);
    ioStrP(PSTR(" vmax="));
    ioFloat(a.vmax, 2);
    ioStrP(PSTR(" acc="));
    ioFloat(a.acc, 2);
    ioStrP(PSTR(" jog="));
    ioFloat(a.jog, 2);
    ioStrP(PSTR(" min="));
    ioFloat(a.minDeg, 2);
    ioStrP(PSTR(" max="));
    ioFloat(a.maxDeg, 2);
    ioStrP(PSTR(" inv="));
    ioInt(a.inv);
    ioEol();
  }
  ioStrP(PSTR("CFG G hyst="));
  ioFloat(g_set.hyst, 2);
  ioStrP(PSTR(" pcomp="));
  ioFloat(g_set.pcomp, 2);
  ioStrP(PSTR(" uzjog="));
  ioFloat(g_set.uzjog, 2);
  ioStrP(PSTR(" jxinv="));
  ioInt(g_set.joyInvX);
  ioStrP(PSTR(" jyinv="));
  ioInt(g_set.joyInvY);
  ioStrP(PSTR(" stall="));
  ioInt(g_set.stallSec);
  ioStrP(PSTR(" maxrun="));
  ioInt(g_set.maxRunSec);
  ioStrP(PSTR(" maxpsi="));
  ioFloat(PSI_MAX_TARGET, 1);
  ioEol();
}

static void handleLine(char* line) {
  uint32_t now = millis();
  g_lastRxMs = now;
  g_link = true;

  char* cmd = strtok(line, " ");
  if (!cmd) return;
  char* a1 = strtok(NULL, " ");
  char* a2 = strtok(NULL, " ");
  float v1 = 0, v2 = 0;

  if (!strcmp_P(cmd, PSTR("HB"))) return;

  if (!strcmp_P(cmd, PSTR("JOG"))) {  // JOG <yaw -1..1> <pitch -1..1>, repeat every <300 ms
    if (!parseNum(a1, v1) || !parseNum(a2, v2)) return reply(false, cmd, PSTR("ARGS"));
    if (g_joyActive) return reply(false, cmd, PSTR("JOY"));
    if (!steppersEnabled()) return reply(false, cmd, PSTR("DISABLED"));
    axisJog(AX_YAW, v1);
    axisJog(AX_PITCH, v2);
    g_piJog = true;
    g_piJogAt = now;
    return;  // no OK: sent at 10 Hz
  }
  if (!strcmp_P(cmd, PSTR("MOVE"))) {  // MOVE <yaw deg> <pitch deg>
    if (!parseNum(a1, v1) || !parseNum(a2, v2)) return reply(false, cmd, PSTR("ARGS"));
    if (!steppersEnabled()) return reply(false, cmd, PSTR("DISABLED"));
    if (g_joyActive) return reply(false, cmd, PSTR("JOY"));
    if (!axisZeroed(AX_YAW) || !axisZeroed(AX_PITCH)) return reply(false, cmd, PSTR("NOZERO"));
    const AxisSettings& y = g_set.ax[AX_YAW];
    const AxisSettings& p = g_set.ax[AX_PITCH];
    if (v1 < y.minDeg || v1 > y.maxDeg || v2 < p.minDeg || v2 > p.maxDeg) return reply(false, cmd, PSTR("LIMIT"));
    g_piJog = false;
    axisMoveTo(AX_YAW, v1);
    axisMoveTo(AX_PITCH, v2);
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("STOP"))) {
    stopAxes();
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("ZERO"))) {  // ZERO Y|P|B
    bool y = a1 && (*a1 == 'Y' || *a1 == 'B');
    bool p = a1 && (*a1 == 'P' || *a1 == 'B');
    if (!y && !p) return reply(false, cmd, PSTR("ARGS"));
    if ((y && axisMoving(AX_YAW)) || (p && axisMoving(AX_PITCH))) return reply(false, cmd, PSTR("MOVING"));
    if (y) axisSetZero(AX_YAW);
    if (p) axisSetZero(AX_PITCH);
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("PRESS"))) {  // PRESS <psi>: fill to psi and hold
    if (!parseNum(a1, v1)) return reply(false, cmd, PSTR("ARGS"));
    if (pressureFault()) return reply(false, cmd, PSTR("FAULT"));
    if (!pressureStartAuto(v1)) return reply(false, cmd, PSTR("RANGE"));
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("PSTOP"))) {
    pressureStop();
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("CLEAR"))) {
    if (!pressureClearFault()) return reply(false, cmd, PSTR("ACTIVE"));
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("ENABLE"))) {  // ENABLE 0|1
    if (!parseNum(a1, v1)) return reply(false, cmd, PSTR("ARGS"));
    if (v1 == 0) g_piJog = false;
    steppersEnable(v1 != 0);
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("SET"))) {  // SET <key> <value>
    if (!a1 || !parseNum(a2, v2)) return reply(false, cmd, PSTR("ARGS"));
    const char* err = settingsSet(a1, v2);
    return reply(err == 0, cmd, err);
  }
  if (!strcmp_P(cmd, PSTR("SAVE"))) {
    if (anyMoving()) return reply(false, cmd, PSTR("MOVING"));  // EEPROM writes stall the loop
    settingsSave();
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("GET"))) {
    sendConfig();
    return reply(true, cmd, 0);
  }
  if (!strcmp_P(cmd, PSTR("VER"))) {
    ioStrP(PSTR("OK VER " FW_VERSION));
    ioEol();
    return;
  }
  reply(false, cmd, PSTR("UNKNOWN"));
}

// ------------------------------------------------------------------ tasks

static void joystickTask(uint32_t now) {
  uint8_t ev = joyUpdate(now);
  if (joyDeflected() && steppersEnabled()) {
    if (!g_joyActive) {
      g_joyActive = true;
      g_piJog = false;
      event(PSTR("JOY"));  // the joystick always overrides the Pi
    }
    axisJog(AX_YAW, joyX());
    axisJog(AX_PITCH, joyY());
  } else if (g_joyActive) {
    g_joyActive = false;
    stopAxes();
  }

  if (ev & JOY_EV_HOLD_START) {
    if (pressureFault()) lcdMessage(PSTR("FAULT: HOLD 3S"));
    else pressureManual(true);
  }
  if (ev & JOY_EV_HOLD_END) pressureManual(false);
  if ((ev & JOY_EV_LONG) && pressureFault()) {
    if (pressureClearFault()) {
      event(PSTR("FAULTCLR"));
      lcdMessage(PSTR("FAULT CLEARED"));
    } else {
      lcdMessage(PSTR("FAULT STILL ON"));
    }
  }
  if (ev & JOY_EV_DOUBLE) {
    if (anyMoving()) {
      lcdMessage(PSTR("STOP TO ZERO"));
    } else {
      axisSetZero(AX_YAW);
      axisSetZero(AX_PITCH);
      event(PSTR("ZERO B"));
      lcdMessage(PSTR("ZERO SET"));
    }
  }
  if (ev & JOY_EV_CLICK) g_lcdPage ^= 1;
}

static void pressureEvents() {
  float pre, post;
  if (pressureTakeShot(pre, post)) {
    ioStrP(PSTR("E SHOT pre="));
    ioFloat(pre, 1);
    ioStrP(PSTR(" post="));
    ioFloat(post, 1);
    ioStrP(PSTR(" ya="));
    ioFloat(axisPosDeg(AX_YAW), 2);
    ioStrP(PSTR(" pa="));
    ioFloat(axisPosDeg(AX_PITCH), 2);
    ioEol();
  }
  uint8_t f = pressureTakeNewFault();
  if (f) {
    ioStrP(PSTR("E FAULT "));
    ioInt(f);
    ioPut(' ');
    ioStrP(faultNameP(f));
    ioEol();
  }
}

static void linkTask(uint32_t now) {
  if (g_piJog && now - g_piJogAt > PI_JOG_TIMEOUT_MS) stopAxes();
  if (g_link && now - g_lastRxMs > PI_LINK_TIMEOUT_MS) {
    g_link = false;
    event(PSTR("LINKLOST"));
    pressureLinkLost();
    if (axisMode(AX_YAW) == AXM_POSITION) axisStop(AX_YAW);
    if (axisMode(AX_PITCH) == AXM_POSITION) axisStop(AX_PITCH);
    g_piJog = false;
  }
}

static void ledsTask(uint32_t now) {
  PState ps = pressureState();
  if (pressureFault()) ledSet(LED_RED);
  else if (compressorOn() || ps == PS_FILL) ledSet(((now / 250) & 1) ? LED_RED : LED_OFF);
  else if (pressureTarget() > 0 && pressurePsi() >= pressureTarget() - g_set.hyst) ledSet(LED_GREEN);
  else ledSet(LED_OFF);
}

static char modeChar(AxisMode m) { return m == AXM_POSITION ? 'P' : (m == AXM_VELOCITY ? 'V' : 'H'); }

static void statusTask() {
  LineBuf l;
  l.clear();
  l.strP(PSTR("S ya="));
  l.flt(axisPosDeg(AX_YAW), 2);
  l.strP(PSTR(" pa="));
  l.flt(axisPosDeg(AX_PITCH), 2);
  l.strP(PSTR(" yt="));
  l.flt(axisTargetDeg(AX_YAW), 2);
  l.strP(PSTR(" pt="));
  l.flt(axisTargetDeg(AX_PITCH), 2);
  l.strP(PSTR(" md="));
  l.ch(modeChar(axisMode(AX_YAW)));
  l.ch(modeChar(axisMode(AX_PITCH)));
  l.strP(PSTR(" mv="));
  l.num((axisMoving(AX_YAW) ? 1 : 0) | (axisMoving(AX_PITCH) ? 2 : 0));
  l.strP(PSTR(" z="));
  l.num((axisZeroed(AX_YAW) ? 1 : 0) | (axisZeroed(AX_PITCH) ? 2 : 0));
  l.strP(PSTR(" en="));
  l.num(steppersEnabled());
  l.strP(PSTR(" psi="));
  l.flt(pressurePsi(), 1);
  l.strP(PSTR(" tgt="));
  l.flt(pressureTarget(), 1);
  l.strP(PSTR(" pst="));
  l.strP(pstateNameP(pressureState()));
  l.strP(PSTR(" c="));
  l.num(compressorOn());
  l.strP(PSTR(" f="));
  l.num(pressureFault());
  l.strP(PSTR(" j="));
  l.num(g_joyActive);
  l.strP(PSTR(" adc="));
  l.num(pressureAdc());
  ioQueueLine(l);  // dropped if the queue is busy; the next one is 100 ms away
}

static void fmtAngle(char* out, float deg) {  // 5 chars: sign + "12.3"
  out[0] = deg < 0 ? '-' : '+';
  float a = fabsf(deg);
  if (a < 99.95f) dtostrf(a, 4, 1, out + 1);
  else if (a < 9999.5f) dtostrf(a, 4, 0, out + 1);
  else strcpy_P(out + 1, PSTR("HUGE"));
  out[5] = 0;
}

static void lcdCompose(uint32_t now) {
  char l1[17], l2[17], a[8];
  if (g_lcdPage == 0) {
    l1[0] = 'Y';
    fmtAngle(a, axisPosDeg(AX_YAW));
    memcpy(l1 + 1, a, 5);
    l1[6] = ' ';
    l1[7] = 'P';
    fmtAngle(a, axisPosDeg(AX_PITCH));
    memcpy(l1 + 8, a, 5);
    l1[13] = ' ';
    const char* flag = PSTR("  ");
    if (!axisZeroed(AX_YAW) || !axisZeroed(AX_PITCH)) flag = PSTR("NZ");
    else if (!steppersEnabled()) flag = PSTR("DS");
    else if (g_joyActive) flag = PSTR("JS");
    else if (g_link) flag = PSTR("PI");
    strcpy_P(l1 + 14, flag);

    if (pressureFault()) {
      strcpy_P(l2, PSTR("FLT "));
      strncpy_P(l2 + 4, faultNameP(pressureFault()), 12);
      l2[16] = 0;
    } else {
      dtostrf(pressurePsi(), 4, 1, l2);
      uint8_t n = strlen(l2);
      l2[n++] = '/';
      if (pressureTarget() > 0) {
        ltoa(lroundf(pressureTarget()), a, 10);
        strcpy(l2 + n, a);
      } else {
        strcpy_P(l2 + n, PSTR("--"));
      }
      strcat_P(l2, PSTR("psi "));
      strcat_P(l2, pstateNameP(pressureState()));
    }
  } else {
    strcpy_P(l1, PSTR("ADC "));
    ltoa(pressureAdc(), a, 10);
    strcat(l1, a);
    strcat_P(l1, PSTR(" "));
    dtostrf(pressureAdc() * 5.0f / 1023.0f, 4, 2, a);
    strcat(l1, a);
    strcat_P(l1, PSTR("V"));
    strcpy_P(l2, g_link ? PSTR("PI LINK  EN") : PSTR("NO PI    EN"));
    l2[11] = steppersEnabled() ? '1' : '0';
    l2[12] = ' ';
    l2[13] = 0;
    strcat_P(l2, g_fromEeprom ? PSTR("EE") : PSTR("DF"));
  }
  if ((int32_t)(g_lcdMsgUntil - now) > 0) {
    strncpy(l2, g_lcdMsg, 16);
    l2[16] = 0;
  }
  lcdSetLine(0, l1);
  lcdSetLine(1, l2);
}

static void lcdTask(uint32_t now) {
  if (lcdPresent()) {
    lcdPump();
  } else if (now - g_tLcdRetry > 5000 && !anyMoving()) {  // LCD unplugged/replugged
    g_tLcdRetry = now;
    lcdBegin();
  }
}

// ------------------------------------------------------------------ main

void setup() {
  wdt_disable();
  pressureBegin();  // compressor output LOW before anything else
  pinMode(PIN_LED_A, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  ledSet(LED_OFF);
  ioBegin(SERIAL_BAUD);
  g_fromEeprom = settingsLoad();
  steppersBegin();
  joyBegin();
  lcdBegin();
  lcdSetLine(0, "J2 CANNON " FW_VERSION);
  lcdSetLine(1, "JOG, DBL-CLK=0");
  g_lcdMsgUntil = millis();
  g_tLcd = millis() + 1500;  // keep the splash up briefly

  ioStrP(PSTR("E BOOT fw=" FW_VERSION " cfg="));
  ioStrP(g_fromEeprom ? PSTR("eeprom") : PSTR("defaults"));
  ioStrP(PSTR(" lcd="));
  ioInt(lcdPresent() ? lcdAddress() : 0);
  ioEol();
  wdt_enable(WDTO_500MS);  // a hung loop resets the board, which drops the compressor
}

void loop() {
  wdt_reset();
  uint32_t now = millis();
  ioPoll(handleLine);
  steppersPlan();
  if (now - g_t10 >= 10) {
    g_t10 = now;
    pressureUpdate(now);
    joystickTask(now);
    pressureEvents();
  }
  linkTask(now);
  ledsTask(now);
  if (now - g_tStatus >= 100) {
    g_tStatus = now;
    statusTask();
  }
  if ((int32_t)(now - g_tLcd) >= 0) {
    g_tLcd = now + 200;
    lcdCompose(now);
  }
  lcdTask(now);
  ioPump();
}
