// Scenario tests for the RedBoard firmware, run on a PC against sim.cpp.
// Each test runs in its own forked process so firmware statics start fresh.
#include <math.h>
#include <sys/wait.h>
#include <unistd.h>
#include <functional>
#include "config.h"
#include "settings.h"
#include "sim.h"
#include "stepper.h"

static int g_fail;
#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
      g_fail++;                                                         \
    }                                                                   \
  } while (0)
#define CHECK_NEAR(a, b, tol)                                                          \
  do {                                                                                 \
    double _a = (a), _b = (b);                                                         \
    if (fabs(_a - _b) > (tol)) {                                                       \
      printf("    FAIL %s:%d: %s = %g, expected %g\n", __FILE__, __LINE__, #a, _a, _b); \
      g_fail++;                                                                        \
    }                                                                                  \
  } while (0)

static long stepsFor(int axis, double deg) { return lround(deg * g_set.ax[axis].spd); }

static void boot() {
  setup();
  runMs(100);
}

// Peak acceleration (steps/s^2) seen in the pulse train, from step counts in 100 ms windows.
static double peakAccel(int axis) {
  const std::vector<uint64_t>& p = sim.pulseTimes[axis];
  if (p.size() < 10) return 0;
  const uint64_t W = 100000;
  std::vector<double> rate;
  size_t j = 0;
  for (uint64_t w = p.front(); w + W <= p.back(); w += W) {
    size_t n = 0;
    while (j < p.size() && p[j] < w + W) {
      j++;
      n++;
    }
    rate.push_back(n / (W * 1e-6));
  }
  double peak = 0;
  for (size_t k = 2; k < rate.size(); k++)  // two windows apart to smooth DDA jitter
    peak = fmax(peak, fabs(rate[k] - rate[k - 2]) / (2 * W * 1e-6));
  return peak;
}

static void bootZeroed() {
  boot();
  send("ZERO B");
  runMs(50);
}

static void checkClean() {
  for (auto& v : sim.violations) printf("    VIOLATION %s\n", v.c_str());
  CHECK(sim.violations.empty());
  CHECK(sim.maxLoopGap < 500000);  // watchdog would have reset the board
}

// ---------------------------------------------------------------- tests

static void t_boot_safe() {
  boot();
  CHECK(sawSince(0, "E BOOT fw=" FW_VERSION));
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  CHECK(sim.pin[PIN_LED_A] == LOW && sim.pin[PIN_LED_B] == LOW);
  CHECK(sim.pin[PIN_YAW_ENA] == DRIVER_ENABLE_LEVEL);
  CHECK(lastStatusField("z") == "0");
  CHECK(lastStatusField("pst") == "IDLE");
  checkClean();
}

static void t_move_requires_zero() {
  boot();
  size_t m = txMark();
  send("MOVE 10 10");
  runMs(50);
  CHECK(sawSince(m, "ERR MOVE NOZERO"));
  CHECK(sim.pulses[0] == 0 && sim.pulses[1] == 0);
  checkClean();
}

static void t_move_exact() {
  bootZeroed();
  size_t m = txMark();
  send("MOVE 30 20");
  runMs(20);
  CHECK(sawSince(m, "OK MOVE"));
  runMs(8000);
  CHECK(axisPosSteps(AX_YAW) == stepsFor(AX_YAW, 30));
  CHECK(axisPosSteps(AX_PITCH) == stepsFor(AX_PITCH, 20));
  CHECK(sim.pulses[0] == stepsFor(AX_YAW, 30));  // no back-and-forth
  CHECK(sim.pulses[1] == stepsFor(AX_PITCH, 20));
  CHECK(!axisMoving(AX_YAW) && !axisMoving(AX_PITCH));
  CHECK_NEAR(statusNum("ya"), 30, 0.05);
  CHECK(lastStatusField("mv") == "0");
  // Reverse back past zero.
  send("MOVE -15 -10");
  runMs(10000);
  CHECK(axisPosSteps(AX_YAW) == stepsFor(AX_YAW, -15));
  CHECK(axisPosSteps(AX_PITCH) == stepsFor(AX_PITCH, -10));
  checkClean();
}

static void t_speed_limit() {
  bootZeroed();
  send("MOVE 90 0");
  runMs(12000);
  double maxRate = g_set.ax[AX_YAW].vmax * g_set.ax[AX_YAW].spd;
  double minGapUs = 1e6 / maxRate;
  CHECK(sim.minPulseGap[0] >= minGapUs * 0.9);
  CHECK(axisPosSteps(AX_YAW) == stepsFor(AX_YAW, 90));
  double accel = g_set.ax[AX_YAW].acc * g_set.ax[AX_YAW].spd;
  CHECK(peakAccel(AX_YAW) < accel * 1.3);
  checkClean();
}

static void t_step_rate_cap() {
  bootZeroed();
  send("SET Y.SPD 400");  // 400 steps/deg * 30 deg/s = 12 kHz, above the 9 kHz cap
  send("SET Y.VMAX 30");
  send("SET Y.ACC 200");
  send("MOVE 45 0");
  runMs(6000);
  CHECK(sim.minPulseGap[0] >= 100);  // DDA gaps alternate 100/150 us around the 111 us average
  CHECK(axisPosSteps(AX_YAW) == 400L * 45);
  checkClean();
}

static void t_move_outside_limits_rejected() {
  bootZeroed();
  size_t m = txMark();
  send("MOVE 95 0");
  send("MOVE 0 -50");
  send("MOVE 0 91");
  runMs(100);
  CHECK(sawSince(m, "ERR MOVE LIMIT"));
  CHECK(sim.pulses[0] == 0 && sim.pulses[1] == 0);
  checkClean();
}

static void t_retarget_mid_move() {
  bootZeroed();
  send("MOVE 80 0");
  runMs(2500);  // well underway at full speed
  send("MOVE -20 0");
  runMs(15000);
  CHECK(axisPosSteps(AX_YAW) == stepsFor(AX_YAW, -20));
  CHECK(sim.maxPos[0] <= stepsFor(AX_YAW, 90));
  checkClean();
}

static void t_jog_stops_at_soft_limit() {
  bootZeroed();
  sim.joyX = 1023;  // full right
  runMs(15000);
  CHECK(sawSince(0, "E JOY"));
  CHECK(sim.maxPos[0] <= stepsFor(AX_YAW, YAW_HARD_MAX_DEG));
  CHECK(axisPosSteps(AX_YAW) >= stepsFor(AX_YAW, YAW_HARD_MAX_DEG) - 2);
  sim.joyX = 512;
  sim.joyY = 0;  // full down on pitch
  runMs(15000);
  CHECK(sim.minPos[1] >= stepsFor(AX_PITCH, PITCH_HARD_MIN_DEG));
  CHECK(axisPosSteps(AX_PITCH) <= stepsFor(AX_PITCH, PITCH_HARD_MIN_DEG) + 2);
  checkClean();
}

static void t_narrowed_limits() {
  bootZeroed();
  size_t m = txMark();
  send("SET Y.MAX 30");
  send("SET Y.MIN -200");  // clamped to the hard limit, not widened
  runMs(50);
  CHECK(sawSince(m, "OK SET"));
  CHECK_NEAR(g_set.ax[AX_YAW].maxDeg, 30, 1e-6);
  CHECK_NEAR(g_set.ax[AX_YAW].minDeg, YAW_HARD_MIN_DEG, 1e-6);
  sim.joyX = 1023;
  runMs(8000);
  CHECK(sim.maxPos[0] <= stepsFor(AX_YAW, 30));
  checkClean();
}

static void t_unzeroed_jog_has_no_limits() {
  boot();
  sim.joyX = 1023;
  runMs(30000);
  // Before zeroing, position is unknown, so no soft limit applies (and speed is reduced).
  CHECK(axisPosSteps(AX_YAW) > stepsFor(AX_YAW, 100));
  checkClean();
}

static void t_pi_jog_times_out() {
  bootZeroed();
  send("JOG 1 0");
  runMs(250);
  CHECK(axisMoving(AX_YAW));
  runMs(3000);  // no more JOG lines: must stop on its own
  CHECK(!axisMoving(AX_YAW));
  checkClean();
}

static void t_joystick_overrides_auto_move() {
  bootZeroed();
  send("MOVE 60 0");
  runMs(1000);
  sim.joyY = 1023;
  runMs(300);
  CHECK(sawSince(0, "E JOY"));
  CHECK(lastStatusField("md")[0] == 'V');
  size_t m = txMark();
  send("MOVE 10 10");
  runMs(50);
  CHECK(sawSince(m, "ERR MOVE JOY"));
  sim.joyY = 512;
  runMs(3000);
  CHECK(!axisMoving(AX_YAW) && !axisMoving(AX_PITCH));
  checkClean();
}

static void t_fill_and_hold() {
  boot();
  runMs(3000);  // compressor minimum off-time after boot
  sim.psi = 5;
  send("PRESS 40");
  runMs(1000);
  CHECK(sim.pin[PIN_COMPRESSOR] == HIGH);
  CHECK(lastStatusField("pst") == "FILL");
  bool sawRed = false;
  for (int i = 0; i < 8; i++) {
    runMs(130);
    if (sim.pin[PIN_LED_A] == (LED_RED_IS_A ? HIGH : LOW) && sim.pin[PIN_LED_B] != sim.pin[PIN_LED_A]) sawRed = true;
  }
  CHECK(sawRed);
  runMs(12000);  // 35 psi at 4 psi/s
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  CHECK(lastStatusField("pst") == "HOLD");
  CHECK(sim.psi >= 39.5 && sim.psi < 41.5);
  // Green: pin B high when red is on pin A.
  CHECK(sim.pin[PIN_LED_A] == (LED_RED_IS_A ? LOW : HIGH));
  CHECK(sim.pin[PIN_LED_B] == (LED_RED_IS_A ? HIGH : LOW));
  // Slow leak: tops off once it drops below target - hysteresis.
  sim.leakRate = 0.2;
  int starts = sim.compStarts;
  runMs(20000);
  CHECK(sim.compStarts > starts);
  CHECK(sim.psi > 38.5 && sim.psi < 41.5);
  checkClean();
}

static void t_shot_detected_no_refill() {
  boot();
  runMs(3000);
  send("PRESS 50");
  runMs(16000);
  CHECK(lastStatusField("pst") == "HOLD");
  size_t m = txMark();
  sim.psi = 12;  // fire
  runMs(500);
  CHECK(sawSince(m, "E SHOT pre=5"));
  CHECK(lastStatusField("pst") == "IDLE");
  int starts = sim.compStarts;
  runMs(10000);
  CHECK(sim.compStarts == starts);  // never refills by itself
  CHECK(sim.pin[PIN_LED_A] == LOW && sim.pin[PIN_LED_B] == LOW);
  checkClean();
}

static void t_overpressure_fault() {
  boot();
  runMs(3000);
  send("PRESS 75");
  runMs(1000);
  CHECK(sim.pin[PIN_COMPRESSOR] == HIGH);
  sim.psi = 79;  // e.g. sensor lag or a regulator failure
  runMs(200);
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  CHECK(sawSince(0, "E FAULT 1 OVERPRESSURE"));
  CHECK(sim.pin[PIN_LED_A] == (LED_RED_IS_A ? HIGH : LOW));  // solid red
  size_t m = txMark();
  send("CLEAR");
  runMs(50);
  CHECK(sawSince(m, "ERR CLEAR ACTIVE"));
  m = txMark();
  send("PRESS 40");
  runMs(50);
  CHECK(sawSince(m, "ERR PRESS FAULT"));
  sim.psi = 60;
  runMs(500);
  m = txMark();
  send("CLEAR");
  runMs(50);
  CHECK(sawSince(m, "OK CLEAR"));
  checkClean();
}

static void t_press_above_max_rejected() {
  boot();
  size_t m = txMark();
  send("PRESS 80");
  runMs(50);
  CHECK(sawSince(m, "ERR PRESS RANGE"));
  checkClean();
}

static void t_sensor_unplugged() {
  boot();
  runMs(3000);
  send("PRESS 40");
  runMs(500);
  sim.adcOverride = 0;
  runMs(300);
  CHECK(sawSince(0, "E FAULT 2 SENSOR_LOW"));
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  checkClean();
}

static void t_fill_stall() {
  boot();
  runMs(3000);
  sim.fillRate = 0;  // compressor runs but nothing happens (leak, unplugged hose)
  send("PRESS 40");
  runMs(1000);
  CHECK(sim.pin[PIN_COMPRESSOR] == HIGH);
  runMs(DEF_FILL_STALL_S * 1000 + 1000);
  CHECK(sawSince(0, "E FAULT 4 FILL_STALL"));
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  checkClean();
}

static void t_link_loss_stops_compressor_and_moves() {
  bootZeroed();
  runMs(3000);
  send("PRESS 60");
  send("MOVE 80 0");
  runMs(1000);
  CHECK(sim.pin[PIN_COMPRESSOR] == HIGH);
  sim.autoHb = false;  // Pi crashes
  runMs(1500);
  CHECK(sawSince(0, "E LINKLOST"));
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  CHECK(lastStatusField("pst") == "IDLE");
  runMs(5000);
  CHECK(!axisMoving(AX_YAW));
  CHECK(axisPosSteps(AX_YAW) < stepsFor(AX_YAW, 80));
  checkClean();
}

static void t_joystick_button_manual_fill() {
  boot();
  runMs(3000);
  sim.joyBtn = true;
  runMs(1500);
  CHECK(sim.pin[PIN_COMPRESSOR] == HIGH);
  CHECK(lastStatusField("pst") == "MAN");
  sim.joyBtn = false;
  runMs(250);
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  CHECK(lastStatusField("pst") == "IDLE");
  // With no target set, a long hold stops at the 75 psi maximum.
  runMs(3000);
  sim.joyBtn = true;
  runMs(25000);
  CHECK(sim.psi < PSI_MAX_TARGET + 1);
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  sim.joyBtn = false;
  runMs(100);
  checkClean();
}

static void t_double_click_zero() {
  boot();
  sim.joyX = 900;
  runMs(1000);
  sim.joyX = 512;
  runMs(2000);
  CHECK(axisPosSteps(AX_YAW) != 0);
  for (int i = 0; i < 2; i++) {
    sim.joyBtn = true;
    runMs(80);
    sim.joyBtn = false;
    runMs(80);
  }
  runMs(200);
  CHECK(sawSince(0, "E ZERO B"));
  CHECK(axisPosSteps(AX_YAW) == 0);
  CHECK(lastStatusField("z") == "3");
  checkClean();
}

static void t_disable_clears_zero() {
  bootZeroed();
  send("ENABLE 0");
  runMs(150);
  CHECK(sim.pin[PIN_YAW_ENA] == !DRIVER_ENABLE_LEVEL);
  CHECK(lastStatusField("z") == "0");
  size_t m = txMark();
  send("MOVE 10 0");
  runMs(50);
  CHECK(sawSince(m, "ERR MOVE DISABLED"));
  send("ENABLE 1");
  runMs(50);
  CHECK(sim.pin[PIN_YAW_ENA] == DRIVER_ENABLE_LEVEL);
  checkClean();
}

static void t_settings_roundtrip() {
  boot();
  size_t m = txMark();
  send("SET Y.SPD 55.5");
  send("SET P.VMAX 12");
  send("SET G.HYST 1.5");
  send("SET Y.SPD 0");      // out of range
  send("SET Z.FOO 1");      // unknown key
  send("SAVE");
  send("GET");
  runMs(200);
  CHECK(sawSince(m, "ERR SET RANGE"));
  CHECK(sawSince(m, "ERR SET KEY"));
  CHECK(sawSince(m, "OK SAVE"));
  CHECK(sawSince(m, "CFG Y spd=55.5000"));
  CHECK(sawSince(m, "CFG G hyst=1.50"));
  Settings saved;
  memcpy(&saved, &g_set, sizeof(Settings));
  settingsDefaults();
  CHECK(settingsLoad());
  CHECK_NEAR(g_set.ax[AX_YAW].spd, 55.5, 1e-6);
  CHECK_NEAR(g_set.ax[AX_PITCH].vmax, 12, 1e-6);
  CHECK(memcmp(&saved, &g_set, sizeof(Settings)) == 0);
  checkClean();
}

static void t_inverted_dir() {
  boot();
  send("SET Y.INV 1");
  send("ZERO B");
  runMs(50);
  send("MOVE 10 0");
  runMs(4000);
  CHECK(axisPosSteps(AX_YAW) == stepsFor(AX_YAW, 10));
  checkClean();  // includes the DIR-pin-vs-direction check with inversion
}

static void t_no_lcd() {
  sim.lcdAttached = false;
  boot();
  CHECK(sawSince(0, "E BOOT fw=" FW_VERSION " cfg=defaults lcd=0"));
  send("ZERO B");
  send("MOVE 5 5");
  runMs(3000);
  CHECK(axisPosSteps(AX_YAW) == stepsFor(AX_YAW, 5));
  checkClean();
}

static void t_garbage_input() {
  boot();
  size_t m = txMark();
  send("MOVE abc 1");
  send(std::string(200, 'X'));  // over-long line is dropped
  send("FOO");
  send("PRESS");
  runMs(100);
  CHECK(sawSince(m, "ERR MOVE ARGS"));
  CHECK(sawSince(m, "ERR FOO UNKNOWN"));
  CHECK(sawSince(m, "ERR PRESS ARGS"));
  CHECK(sim.pin[PIN_COMPRESSOR] == LOW);
  checkClean();
}

// ---------------------------------------------------------------- runner

struct Test {
  const char* name;
  void (*fn)();
};

static const Test TESTS[] = {
    {"boot_safe", t_boot_safe},
    {"move_requires_zero", t_move_requires_zero},
    {"move_exact", t_move_exact},
    {"speed_limit", t_speed_limit},
    {"step_rate_cap", t_step_rate_cap},
    {"move_outside_limits_rejected", t_move_outside_limits_rejected},
    {"retarget_mid_move", t_retarget_mid_move},
    {"jog_stops_at_soft_limit", t_jog_stops_at_soft_limit},
    {"narrowed_limits", t_narrowed_limits},
    {"unzeroed_jog_has_no_limits", t_unzeroed_jog_has_no_limits},
    {"pi_jog_times_out", t_pi_jog_times_out},
    {"joystick_overrides_auto_move", t_joystick_overrides_auto_move},
    {"fill_and_hold", t_fill_and_hold},
    {"shot_detected_no_refill", t_shot_detected_no_refill},
    {"overpressure_fault", t_overpressure_fault},
    {"press_above_max_rejected", t_press_above_max_rejected},
    {"sensor_unplugged", t_sensor_unplugged},
    {"fill_stall", t_fill_stall},
    {"link_loss_stops_compressor_and_moves", t_link_loss_stops_compressor_and_moves},
    {"joystick_button_manual_fill", t_joystick_button_manual_fill},
    {"double_click_zero", t_double_click_zero},
    {"disable_clears_zero", t_disable_clears_zero},
    {"settings_roundtrip", t_settings_roundtrip},
    {"inverted_dir", t_inverted_dir},
    {"no_lcd", t_no_lcd},
    {"garbage_input", t_garbage_input},
};

int main(int argc, char** argv) {
  int failed = 0, run = 0;
  for (const Test& t : TESTS) {
    if (argc > 1 && strcmp(argv[1], t.name) != 0) continue;
    run++;
    fflush(stdout);
    pid_t pid = fork();
    if (pid == 0) {
      t.fn();
      fflush(stdout);
      _exit(g_fail ? 1 : 0);
    }
    int status = 0;
    waitpid(pid, &status, 0);
    bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
    printf("%s %s\n", ok ? "PASS" : "FAIL", t.name);
    if (!ok) failed++;
  }
  printf("\n%d/%d passed\n", run - failed, run);
  return failed ? 1 : 0;
}
