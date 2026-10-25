#include "settings.h"
#include "config.h"
#include <EEPROM.h>

static const uint16_t SETTINGS_MAGIC = 0x4A32;  // "J2"
static const uint8_t SETTINGS_VERSION = 1;
static const int SETTINGS_ADDR = 0;

Settings g_set;

float hardMinDeg(uint8_t axis) { return axis == AX_YAW ? YAW_HARD_MIN_DEG : PITCH_HARD_MIN_DEG; }
float hardMaxDeg(uint8_t axis) { return axis == AX_YAW ? YAW_HARD_MAX_DEG : PITCH_HARD_MAX_DEG; }

static uint8_t calcCrc(const Settings& s) {
  const uint8_t* p = (const uint8_t*)&s;
  uint8_t crc = 0;
  for (size_t i = 0; i < offsetof(Settings, crc); i++) {
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

void settingsDefaults() {
  memset(&g_set, 0, sizeof(g_set));
  g_set.magic = SETTINGS_MAGIC;
  g_set.version = SETTINGS_VERSION;
  AxisSettings& y = g_set.ax[AX_YAW];
  y.spd = DEF_YAW_STEPS_PER_DEG;
  y.vmax = DEF_YAW_VMAX_DPS;
  y.acc = DEF_YAW_ACCEL_DPS2;
  y.jog = DEF_YAW_JOG_DPS;
  y.minDeg = YAW_HARD_MIN_DEG;
  y.maxDeg = YAW_HARD_MAX_DEG;
  AxisSettings& p = g_set.ax[AX_PITCH];
  p.spd = DEF_PITCH_STEPS_PER_DEG;
  p.vmax = DEF_PITCH_VMAX_DPS;
  p.acc = DEF_PITCH_ACCEL_DPS2;
  p.jog = DEF_PITCH_JOG_DPS;
  p.minDeg = PITCH_HARD_MIN_DEG;
  p.maxDeg = PITCH_HARD_MAX_DEG;
  g_set.hyst = DEF_PSI_HYST;
  g_set.pcomp = DEF_PSI_OVERSHOOT;
  g_set.uzjog = DEF_UNZEROED_JOG_FRAC;
  g_set.stallSec = DEF_FILL_STALL_S;
  g_set.maxRunSec = DEF_FILL_MAX_RUN_S;
}

static bool inRange(float v, float lo, float hi) { return v >= lo && v <= hi; }

// Re-check everything loaded from EEPROM against the same bounds SET uses.
static bool settingsSane() {
  for (uint8_t i = 0; i < 2; i++) {
    const AxisSettings& a = g_set.ax[i];
    if (!inRange(a.spd, 1, 5000) || !inRange(a.vmax, 0.1f, 180) || !inRange(a.acc, 0.1f, 720) ||
        !inRange(a.jog, 0.1f, 180) || a.inv > 1) return false;
    if (a.minDeg < hardMinDeg(i) || a.maxDeg > hardMaxDeg(i) || a.minDeg >= a.maxDeg) return false;
  }
  return inRange(g_set.hyst, 0.2f, 5) && inRange(g_set.pcomp, 0, 5) && inRange(g_set.uzjog, 0.05f, 1) &&
         g_set.joyInvX <= 1 && g_set.joyInvY <= 1 && inRange(g_set.stallSec, 5, 120) &&
         inRange(g_set.maxRunSec, 10, 900);
}

bool settingsLoad() {
  EEPROM.get(SETTINGS_ADDR, g_set);
  if (g_set.magic == SETTINGS_MAGIC && g_set.version == SETTINGS_VERSION && g_set.crc == calcCrc(g_set) &&
      settingsSane()) return true;
  settingsDefaults();
  return false;
}

void settingsSave() {
  g_set.crc = calcCrc(g_set);
  EEPROM.put(SETTINGS_ADDR, g_set);  // put() only rewrites bytes that changed
}

// Keys: Y.SPD Y.VMAX Y.ACC Y.JOG Y.MIN Y.MAX Y.INV (and P.*),
//       G.HYST G.PCOMP G.UZJOG G.JXINV G.JYINV G.STALL G.MAXRUN
const char* settingsSet(const char* key, float val) {
  if ((key[0] == 'Y' || key[0] == 'P') && key[1] == '.') {
    uint8_t i = key[0] == 'Y' ? AX_YAW : AX_PITCH;
    AxisSettings& a = g_set.ax[i];
    const char* k = key + 2;
    if (!strcmp_P(k, PSTR("SPD"))) {
      if (!inRange(val, 1, 5000)) return PSTR("RANGE");
      a.spd = val;
    } else if (!strcmp_P(k, PSTR("VMAX"))) {
      if (!inRange(val, 0.1f, 180)) return PSTR("RANGE");
      a.vmax = val;
    } else if (!strcmp_P(k, PSTR("ACC"))) {
      if (!inRange(val, 0.1f, 720)) return PSTR("RANGE");
      a.acc = val;
    } else if (!strcmp_P(k, PSTR("JOG"))) {
      if (!inRange(val, 0.1f, 180)) return PSTR("RANGE");
      a.jog = val;
    } else if (!strcmp_P(k, PSTR("MIN"))) {
      if (val < hardMinDeg(i)) val = hardMinDeg(i);  // never wider than the hard limit
      if (val >= a.maxDeg) return PSTR("RANGE");
      a.minDeg = val;
    } else if (!strcmp_P(k, PSTR("MAX"))) {
      if (val > hardMaxDeg(i)) val = hardMaxDeg(i);
      if (val <= a.minDeg) return PSTR("RANGE");
      a.maxDeg = val;
    } else if (!strcmp_P(k, PSTR("INV"))) {
      a.inv = val != 0;
    } else {
      return PSTR("KEY");
    }
    return 0;
  }
  if (key[0] == 'G' && key[1] == '.') {
    const char* k = key + 2;
    if (!strcmp_P(k, PSTR("HYST"))) {
      if (!inRange(val, 0.2f, 5)) return PSTR("RANGE");
      g_set.hyst = val;
    } else if (!strcmp_P(k, PSTR("PCOMP"))) {
      if (!inRange(val, 0, 5)) return PSTR("RANGE");
      g_set.pcomp = val;
    } else if (!strcmp_P(k, PSTR("UZJOG"))) {
      if (!inRange(val, 0.05f, 1)) return PSTR("RANGE");
      g_set.uzjog = val;
    } else if (!strcmp_P(k, PSTR("JXINV"))) {
      g_set.joyInvX = val != 0;
    } else if (!strcmp_P(k, PSTR("JYINV"))) {
      g_set.joyInvY = val != 0;
    } else if (!strcmp_P(k, PSTR("STALL"))) {
      if (!inRange(val, 5, 120)) return PSTR("RANGE");
      g_set.stallSec = (uint16_t)val;
    } else if (!strcmp_P(k, PSTR("MAXRUN"))) {
      if (!inRange(val, 10, 900)) return PSTR("RANGE");
      g_set.maxRunSec = (uint16_t)val;
    } else {
      return PSTR("KEY");
    }
    return 0;
  }
  return PSTR("KEY");
}
