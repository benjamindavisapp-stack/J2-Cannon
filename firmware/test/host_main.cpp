// "Virtual RedBoard": runs the real firmware in real time behind a pseudo
// terminal, so the Pi app can connect to it like a serial port.
//
//   ./fw_host            prints the pty path (e.g. /dev/pts/5) on stdout
//   kill -USR1 <pid>     fires the cannon (chamber pressure drops)
//
// The pressure plant fills at 4 psi/s while the compressor pin is high.
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include "sim.h"

static volatile sig_atomic_t g_fire = 0;
static void onFire(int) { g_fire = 1; }

static double wallMs() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

int main() {
  int m = posix_openpt(O_RDWR | O_NOCTTY);
  if (m < 0 || grantpt(m) || unlockpt(m)) {
    perror("pty");
    return 1;
  }
  // Keep a slave fd open so writes never fail with EIO before the app connects.
  int keep = open(ptsname(m), O_RDWR | O_NOCTTY);
  termios t;
  tcgetattr(keep, &t);
  cfmakeraw(&t);
  tcsetattr(keep, TCSANOW, &t);
  fcntl(m, F_SETFL, O_NONBLOCK);
  printf("%s\n", ptsname(m));
  fflush(stdout);
  signal(SIGUSR1, onFire);

  sim.autoHb = false;
  sim.fillRate = 4.0;
  setup();
  size_t sent = 0;
  double start = wallMs();
  double simStart = sim.t / 1000.0;
  for (;;) {
    char buf[256];
    ssize_t n = read(m, buf, sizeof buf);
    if (n > 0) sim.rx.append(buf, n);
    if (g_fire) {
      g_fire = 0;
      sim.psi = sim.psi * 0.1;
    }
    runMs(1);
    while (sent < sim.tx.size()) {
      ssize_t w = write(m, sim.tx.data() + sent, sim.tx.size() - sent);
      if (w <= 0) {
        if (errno == EAGAIN) sent = sim.tx.size();  // nobody reading: drop it, like a real UART
        break;
      }
      sent += w;
    }
    double ahead = (sim.t / 1000.0 - simStart) - (wallMs() - start);
    if (ahead > 0) usleep((useconds_t)(ahead * 1000));
  }
}
