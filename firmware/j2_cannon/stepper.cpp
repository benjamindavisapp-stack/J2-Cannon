#include "stepper.h"
#include "config.h"
#include "settings.h"
#include <util/atomic.h>

// Step pins on PORTB, toggled with single sbi/cbi instructions in the ISR.
#define YAW_PUL_MASK _BV(5)    // D13
#define PITCH_PUL_MASK _BV(2)  // D10

static const uint32_t TICK_HZ = 20000;                  // ISR rate; max 10 kHz step rate
static const float INC_PER_SPS = 65536.0f / TICK_HZ;    // DDA increment per step/s
static const float MAX_STEP_RATE = 9000.0f;             // steps/s, keeps inc below 32768
static const float V_CREEP = 30.0f;                     // steps/s, slowest approach speed
static const uint32_t PLAN_PERIOD_US = 2000;
static const int32_t UNBOUNDED = 2000000000L;

// Shared with the ISR.
struct AxisIsr {
  volatile int32_t pos;
  volatile int32_t lo, hi;    // never step outside this window
  volatile uint16_t inc;      // phase increment per tick, 0 = stopped
  volatile uint16_t acc;      // phase accumulator
  volatile int8_t dir;        // +1 / -1
  volatile uint8_t pulseHigh;
};

// Planner-only state.
struct AxisRt {
  AxisMode mode;
  float v;          // signed steps/s
  float jogFrac;
  int32_t target;   // steps, position mode
  bool zeroed;
  bool approach;    // last cycle clamped the ISR window at the target
  int8_t dir;       // direction currently on the DIR pin
  uint8_t dirLevel; // level last written to the DIR pin
};

static AxisIsr s_isr[2];
static AxisRt s_ax[2];
static bool s_enabled;
static uint32_t s_lastPlanUs;

static const uint8_t PIN_DIR[2] = {PIN_YAW_DIR, PIN_PITCH_DIR};
static const uint8_t PIN_ENA[2] = {PIN_YAW_ENA, PIN_PITCH_ENA};

static inline void tickAxis(AxisIsr& s, const uint8_t pulMask) __attribute__((always_inline));
static inline void tickAxis(AxisIsr& s, const uint8_t pulMask) {
  if (s.pulseHigh) {
    PORTB &= (uint8_t)~pulMask;
    s.pulseHigh = 0;
  }
  uint16_t inc = s.inc;
  if (!inc) return;
  uint16_t old = s.acc;
  uint16_t now = old + inc;
  s.acc = now;
  if (now >= old) return;  // accumulator did not wrap: no step this tick
  // inc < 32768 means it cannot wrap on two ticks in a row, so the pulse
  // always gets at least one low tick before the next one.
  int32_t p = s.pos;
  if (s.dir > 0) {
    if (p >= s.hi) return;
    s.pos = p + 1;
  } else {
    if (p <= s.lo) return;
    s.pos = p - 1;
  }
  PORTB |= pulMask;
  s.pulseHigh = 1;
}

ISR(TIMER1_COMPA_vect) {
  tickAxis(s_isr[AX_YAW], YAW_PUL_MASK);
  tickAxis(s_isr[AX_PITCH], PITCH_PUL_MASK);
}

static int32_t readPos(uint8_t i) {
  int32_t p;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { p = s_isr[i].pos; }
  return p;
}

static uint8_t dirLevel(uint8_t i, int8_t dir) { return ((dir > 0) != (g_set.ax[i].inv != 0)) ? HIGH : LOW; }

static void writeEnable(bool on) {
  uint8_t level = on ? DRIVER_ENABLE_LEVEL : !DRIVER_ENABLE_LEVEL;
  digitalWrite(PIN_ENA[AX_YAW], level);
  digitalWrite(PIN_ENA[AX_PITCH], level);
}

void steppersBegin() {
  pinMode(PIN_YAW_PUL, OUTPUT);
  pinMode(PIN_PITCH_PUL, OUTPUT);
  digitalWrite(PIN_YAW_PUL, LOW);
  digitalWrite(PIN_PITCH_PUL, LOW);
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(PIN_DIR[i], OUTPUT);
    pinMode(PIN_ENA[i], OUTPUT);
    s_ax[i].mode = AXM_HOLD;
    s_ax[i].v = 0;
    s_ax[i].zeroed = false;
    s_ax[i].dir = 1;
    s_ax[i].dirLevel = dirLevel(i, 1);
    digitalWrite(PIN_DIR[i], s_ax[i].dirLevel);
    s_isr[i].dir = 1;
    s_isr[i].lo = -UNBOUNDED;
    s_isr[i].hi = UNBOUNDED;
  }
  s_enabled = true;
  writeEnable(true);

  // Timer1 CTC at TICK_HZ: 16 MHz / 8 = 2 MHz, /100 = 20 kHz.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS11);
    OCR1A = (uint16_t)(F_CPU / 8 / TICK_HZ - 1);
    TIMSK1 = _BV(OCIE1A);
  }
  s_lastPlanUs = micros();
}

static float approachVal(float v, float to, float dv) {
  if (v < to) {
    v += dv;
    if (v > to) v = to;
  } else if (v > to) {
    v -= dv;
    if (v < to) v = to;
  }
  return v;
}

static void applyIsr(uint8_t i, float v, bool approach, int32_t tgt, int32_t lo, int32_t hi) {
  AxisRt& a = s_ax[i];
  float rate = fabsf(v);
  if (rate > MAX_STEP_RATE) rate = MAX_STEP_RATE;
  uint16_t inc = (uint16_t)(rate * INC_PER_SPS + 0.5f);
  if (inc > 32767) inc = 32767;
  int8_t want = v > 0 ? 1 : (v < 0 ? -1 : a.dir);
  uint8_t level = dirLevel(i, want);  // also picks up a changed INV setting
  if (want != a.dir || level != a.dirLevel) {
    // Speed passes through zero here. Stop, let any pulse finish, change
    // DIR, then give the driver its DIR setup time before stepping again.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { s_isr[i].inc = 0; }
    delayMicroseconds(60);
    digitalWrite(PIN_DIR[i], level);
    delayMicroseconds(10);
    a.dir = want;
    a.dirLevel = level;
  }
  if (approach) {
    if (want > 0 && tgt < hi) hi = tgt;
    if (want < 0 && tgt > lo) lo = tgt;
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    s_isr[i].lo = lo;
    s_isr[i].hi = hi;
    s_isr[i].dir = want;
    s_isr[i].inc = inc;
  }
}

static void planAxis(uint8_t i, float dt) {
  AxisRt& a = s_ax[i];
  const AxisSettings& cfg = g_set.ax[i];
  const float spd = cfg.spd;
  const float acc = cfg.acc * spd;
  float vmax = cfg.vmax * spd;
  if (vmax > MAX_STEP_RATE) vmax = MAX_STEP_RATE;
  const float dv = acc * dt;
  const int32_t pos = readPos(i);
  int32_t lo = -UNBOUNDED, hi = UNBOUNDED;
  if (a.zeroed) {
    lo = lroundf(cfg.minDeg * spd);
    hi = lroundf(cfg.maxDeg * spd);
  }
  float v = a.v;
  bool approach = false;
  int32_t tgt = 0;

  if (!s_enabled) {
    v = 0;
    a.mode = AXM_HOLD;
  } else if (a.mode == AXM_POSITION) {
    tgt = constrain(a.target, lo, hi);
    int32_t d = tgt - pos;
    if (d == 0) {
      // Arrived (the ISR stopped us exactly here), unless we are coasting
      // through too fast to stop, in which case brake and come back.
      if (a.approach || fabsf(v) <= V_CREEP) v = 0;
      else v = approachVal(v, 0, dv);
    } else if (v == 0.0f || (v > 0) == (d > 0)) {
      float vcap = sqrtf(2.0f * acc * fabsf((float)d));  // fastest speed that can still stop
      if (vcap > vmax) vcap = vmax;
      if (vcap < V_CREEP) vcap = V_CREEP;
      float sp = fabsf(v);
      approach = sp <= vcap * 1.25f + V_CREEP;  // only clamp at the target if we can stop there
      if (sp < vcap) {
        sp += dv;
        if (sp > vcap) sp = vcap;
      } else {
        sp -= 2.0f * dv;
        if (sp < vcap) sp = vcap;
      }
      v = d > 0 ? sp : -sp;
    } else {
      v = approachVal(v, 0, dv);  // heading away from a new target: brake first
    }
  } else if (a.mode == AXM_VELOCITY) {
    float jmax = cfg.jog * spd * (a.zeroed ? 1.0f : g_set.uzjog);
    if (jmax > vmax) jmax = vmax;
    float vd = a.jogFrac * jmax;
    if (a.zeroed) {  // brake so we stop at the soft limit, not on it
      if (vd > 0) {
        float room = (float)(hi - pos);
        float cap = room > 0 ? sqrtf(2.0f * acc * room) : 0;
        if (vd > cap) vd = cap;
      } else if (vd < 0) {
        float room = (float)(pos - lo);
        float cap = room > 0 ? sqrtf(2.0f * acc * room) : 0;
        if (vd < -cap) vd = -cap;
      }
    }
    v = approachVal(v, vd, dv);
  } else {
    v = approachVal(v, 0, dv);
  }
  a.v = v;
  a.approach = approach;
  applyIsr(i, v, approach, tgt, lo, hi);
}

void steppersPlan() {
  uint32_t now = micros();
  uint32_t el = now - s_lastPlanUs;
  if (el < PLAN_PERIOD_US) return;
  s_lastPlanUs = now;
  float dt = el * 1e-6f;
  if (dt > 0.02f) dt = 0.02f;  // a long stall (EEPROM write) must not cause a speed jump
  planAxis(AX_YAW, dt);
  planAxis(AX_PITCH, dt);
}

void steppersEnable(bool on) {
  if (!on) {
    for (uint8_t i = 0; i < 2; i++) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { s_isr[i].inc = 0; }
      s_ax[i].v = 0;
      s_ax[i].mode = AXM_HOLD;
      s_ax[i].zeroed = false;  // the turret can be pushed by hand while unpowered
    }
  }
  s_enabled = on;
  writeEnable(on);
}

bool steppersEnabled() { return s_enabled; }

bool axisMoveTo(uint8_t i, float deg) {
  const AxisSettings& c = g_set.ax[i];
  if (!s_enabled || !s_ax[i].zeroed) return false;
  if (deg < c.minDeg - 0.001f || deg > c.maxDeg + 0.001f) return false;
  int32_t t = lroundf(deg * c.spd);
  if (s_ax[i].mode != AXM_POSITION || t != s_ax[i].target) s_ax[i].approach = false;
  s_ax[i].target = t;
  s_ax[i].mode = AXM_POSITION;
  return true;
}

void axisJog(uint8_t i, float frac) {
  if (!s_enabled) return;
  s_ax[i].jogFrac = constrain(frac, -1.0f, 1.0f);
  s_ax[i].mode = AXM_VELOCITY;
}

void axisStop(uint8_t i) { s_ax[i].mode = AXM_HOLD; }

bool axisSetZero(uint8_t i) {
  if (s_ax[i].v != 0.0f) return false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { s_isr[i].pos = 0; }
  s_ax[i].target = 0;
  s_ax[i].mode = AXM_HOLD;
  s_ax[i].approach = false;
  s_ax[i].zeroed = true;
  return true;
}

bool axisZeroed(uint8_t i) { return s_ax[i].zeroed; }
bool axisMoving(uint8_t i) { return s_ax[i].v != 0.0f; }
AxisMode axisMode(uint8_t i) { return s_ax[i].mode; }
int32_t axisPosSteps(uint8_t i) { return readPos(i); }
float axisPosDeg(uint8_t i) { return readPos(i) / g_set.ax[i].spd; }

float axisTargetDeg(uint8_t i) {
  if (s_ax[i].mode == AXM_POSITION) return s_ax[i].target / g_set.ax[i].spd;
  return axisPosDeg(i);
}
