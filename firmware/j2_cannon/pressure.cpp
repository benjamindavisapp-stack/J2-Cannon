#include "pressure.h"
#include "config.h"
#include "settings.h"

static const uint8_t HIST_LEN = 10;          // 10 x 50 ms = 0.5 s window for shot detection
static const uint32_t SHOT_COOLDOWN_MS = 2000;
static const float FILL_STALL_PSI = 0.5f;

static float s_psi;           // filtered
static uint16_t s_adc;
static float s_target;
static PState s_state = PS_IDLE;
static uint8_t s_fault = F_NONE;
static uint8_t s_newFault = F_NONE;
static bool s_comp;
static uint32_t s_compOnAt, s_compOffAt;
static float s_stallRefPsi;
static uint32_t s_stallRefAt;
static uint8_t s_overCount, s_lowCount, s_highCount;
static float s_hist[HIST_LEN];
static uint8_t s_histIdx;
static uint32_t s_histAt;
static uint32_t s_lastShotAt;
static bool s_shotPending;
static float s_shotPre, s_shotPost;
static bool s_manualHeld;
static bool s_manualLock;  // a shot during a manual fill needs a fresh button press
static bool s_started;

static float adcToPsi(float adc) {
  float frac = adc / 1023.0f;
  return (frac - SENSOR_ZERO_FRAC) / SENSOR_SPAN_FRAC * SENSOR_PSI_FULL_SCALE;
}

static void setCompressor(bool on, uint32_t now) {
  if (on == s_comp) return;
  s_comp = on;
  digitalWrite(PIN_COMPRESSOR, on ? HIGH : LOW);
  if (on) {
    s_compOnAt = now;
    s_stallRefPsi = s_psi;
    s_stallRefAt = now;
  } else {
    s_compOffAt = now;
  }
}

static void setFault(uint8_t code, uint32_t now) {
  setCompressor(false, now);
  s_state = PS_IDLE;
  if (s_manualHeld) s_manualLock = true;
  if (s_fault == F_NONE) {
    s_fault = code;
    s_newFault = code;
  }
}

void pressureBegin() {
  digitalWrite(PIN_COMPRESSOR, LOW);
  pinMode(PIN_COMPRESSOR, OUTPUT);
  s_comp = false;
  s_compOffAt = millis();  // also enforces the minimum off time after a reset
}

static void detectShot(uint32_t now) {
  if (now - s_histAt >= 50) {
    s_histAt = now;
    s_hist[s_histIdx] = s_psi;
    s_histIdx = (s_histIdx + 1) % HIST_LEN;
  }
  if (s_adc < SENSOR_FAULT_LOW_ADC || s_adc > SENSOR_FAULT_HIGH_ADC) return;  // reading is junk, not a shot
  float peak = s_hist[0];
  for (uint8_t i = 1; i < HIST_LEN; i++)
    if (s_hist[i] > peak) peak = s_hist[i];
  if (peak < SHOT_MIN_PSI || peak - s_psi < SHOT_DROP_PSI || now - s_lastShotAt < SHOT_COOLDOWN_MS) return;

  s_shotPending = true;
  s_shotPre = peak;
  s_shotPost = s_psi;
  s_lastShotAt = now;
  for (uint8_t i = 0; i < HIST_LEN; i++) s_hist[i] = s_psi;
  // Never refill on our own after a shot.
  if (s_state == PS_MANUAL) s_manualLock = true;
  s_state = PS_IDLE;
  setCompressor(false, now);
}

void pressureUpdate(uint32_t now) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) sum += analogRead(PIN_PRESSURE);
  s_adc = (sum + 2) >> 2;
  float raw = adcToPsi(sum * 0.25f);
  if (!s_started) {
    s_started = true;
    s_psi = raw;
    for (uint8_t i = 0; i < HIST_LEN; i++) s_hist[i] = raw;
    s_histAt = now;
  } else {
    s_psi += 0.35f * (raw - s_psi);
  }

  // Faults are checked on every sample, in every state.
  if (s_adc < SENSOR_FAULT_LOW_ADC) {
    if (++s_lowCount >= 10) setFault(F_SENSOR_LOW, now);
  } else {
    s_lowCount = 0;
  }
  if (s_adc > SENSOR_FAULT_HIGH_ADC) {
    if (++s_highCount >= 10) setFault(F_SENSOR_HIGH, now);
  } else {
    s_highCount = 0;
  }
  if (raw >= PSI_CUTOFF) {
    if (++s_overCount >= 3) setFault(F_OVERPRESSURE, now);  // 3 samples = 30 ms
  } else {
    s_overCount = 0;
  }

  detectShot(now);

  bool want = false;
  if (s_fault == F_NONE) {
    switch (s_state) {
      case PS_FILL:
        if (s_psi >= s_target - g_set.pcomp) s_state = PS_HOLD;
        else want = true;
        break;
      case PS_HOLD:
        want = s_comp ? s_psi < s_target - g_set.pcomp : s_psi < s_target - g_set.hyst;
        break;
      case PS_MANUAL: {
        float cap = s_target > 0 ? s_target : PSI_MAX_TARGET;
        want = s_manualHeld && (s_comp ? s_psi < cap - g_set.pcomp : s_psi < cap - g_set.hyst);
        break;
      }
      default:
        break;
    }
  }
  if (want && !s_comp && now - s_compOffAt < COMPRESSOR_MIN_OFF_MS) want = false;
  setCompressor(want, now);

  if (s_comp) {
    if (s_psi >= s_stallRefPsi + FILL_STALL_PSI) {
      s_stallRefPsi = s_psi;
      s_stallRefAt = now;
    } else if (now - s_stallRefAt > g_set.stallSec * 1000UL) {
      setFault(F_FILL_STALL, now);
    }
    if (s_comp && now - s_compOnAt > g_set.maxRunSec * 1000UL) setFault(F_FILL_TIMEOUT, now);
  }
}

bool pressureStartAuto(float psi) {
  if (s_fault != F_NONE || psi < 1.0f || psi > PSI_MAX_TARGET) return false;
  s_target = psi;
  if (s_state != PS_MANUAL) s_state = s_psi >= psi - g_set.pcomp ? PS_HOLD : PS_FILL;
  return true;
}

void pressureStop() {
  s_state = PS_IDLE;
  setCompressor(false, millis());
}

void pressureManual(bool held) {
  if (held) {
    if (s_fault != F_NONE || s_manualLock) return;
    s_manualHeld = true;
    s_state = PS_MANUAL;
  } else {
    s_manualHeld = false;
    s_manualLock = false;
    if (s_state == PS_MANUAL) {
      s_state = PS_IDLE;
      setCompressor(false, millis());
    }
  }
}

bool pressureClearFault() {
  if (s_fault == F_NONE) return true;
  if (s_adc < SENSOR_FAULT_LOW_ADC || s_adc > SENSOR_FAULT_HIGH_ADC || s_psi >= PSI_CUTOFF) return false;
  s_fault = F_NONE;
  s_overCount = s_lowCount = s_highCount = 0;
  return true;
}

void pressureLinkLost() {
  if (s_state == PS_FILL || s_state == PS_HOLD) pressureStop();
}

float pressurePsi() { return s_psi; }
float pressureTarget() { return s_target; }
uint16_t pressureAdc() { return s_adc; }
PState pressureState() { return s_state; }
bool compressorOn() { return s_comp; }
uint8_t pressureFault() { return s_fault; }

bool pressureTakeShot(float& pre, float& post) {
  if (!s_shotPending) return false;
  s_shotPending = false;
  pre = s_shotPre;
  post = s_shotPost;
  return true;
}

uint8_t pressureTakeNewFault() {
  uint8_t f = s_newFault;
  s_newFault = F_NONE;
  return f;
}

const char* faultNameP(uint8_t code) {
  switch (code) {
    case F_OVERPRESSURE: return PSTR("OVERPRESSURE");
    case F_SENSOR_LOW: return PSTR("SENSOR_LOW");
    case F_SENSOR_HIGH: return PSTR("SENSOR_HIGH");
    case F_FILL_STALL: return PSTR("FILL_STALL");
    case F_FILL_TIMEOUT: return PSTR("FILL_TIMEOUT");
    default: return PSTR("NONE");
  }
}

const char* pstateNameP(PState s) {
  switch (s) {
    case PS_FILL: return PSTR("FILL");
    case PS_HOLD: return PSTR("HOLD");
    case PS_MANUAL: return PSTR("MAN");
    default: return PSTR("IDLE");
  }
}
