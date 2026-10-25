"""A stand-in for the RedBoard so the app can run with no hardware (--sim).

It speaks the same serial protocol and roughly copies the firmware's
behaviour: trapezoidal moves, soft limits, zeroing, fill/hold, faults and
shot detection. It is a convenience for trying the UI, not a test of the
real firmware (that is firmware/test).
"""

from __future__ import annotations

import math
import queue
import threading
import time
from typing import Callable, Optional

from .config import FW_MAX_PSI, FW_PITCH_LIMITS, FW_YAW_LIMITS

PSI_CUTOFF = 78.0


class _Axis:
    def __init__(self, hard):
        self.hard = hard
        self.pos = 0.0
        self.v = 0.0
        self.target = 0.0
        self.mode = "H"
        self.jog = 0.0
        self.zeroed = False
        self.vmax = 20.0
        self.acc = 30.0
        self.jog_speed = 15.0
        self.min = hard[0]
        self.max = hard[1]

    def step(self, dt: float, uzjog: float) -> None:
        a = self.acc
        if self.mode == "P":
            d = self.target - self.pos
            if abs(d) < 1e-4 and abs(self.v) < a * dt * 2:
                self.pos, self.v = self.target, 0.0
                return
            vcap = min(self.vmax, math.sqrt(2 * a * abs(d)))
            want = math.copysign(vcap, d)
        elif self.mode == "V":
            want = self.jog * min(self.jog_speed * (1 if self.zeroed else uzjog), self.vmax)
            if self.zeroed:
                if want > 0:
                    want = min(want, math.sqrt(2 * a * max(self.max - self.pos, 0)))
                elif want < 0:
                    want = max(want, -math.sqrt(2 * a * max(self.pos - self.min, 0)))
        else:
            want = 0.0
        dv = a * dt
        self.v = min(self.v + dv, want) if self.v < want else max(self.v - dv, want)
        new = self.pos + self.v * dt
        if self.mode == "P" and (self.target - self.pos) * (self.target - new) <= 0 and abs(self.v) < 3 * dv + 1:
            new, self.v = self.target, 0.0
        if self.zeroed:
            if new > self.max:
                new, self.v = self.max, 0.0
            if new < self.min:
                new, self.v = self.min, 0.0
        self.pos = new


class SimFirmware:
    def __init__(self, fill_rate: float = 4.0, land_fn: Optional[Callable[[float, float, float], float]] = None):
        self.fill_rate = fill_rate
        self.land_fn = land_fn  # (yaw, pitch, psi) -> landing distance in ft, for sim calibration
        self.out: "queue.Queue[str]" = queue.Queue()
        self.lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self.name = "sim"
        self._reset()

    # ------------------------------------------------------------ transport API

    def open(self) -> None:
        with self.lock:
            self._reset()  # like the DTR reset when the real port opens
        self.out.put("E BOOT fw=sim cfg=defaults lcd=0")
        if not self._thread:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def readline(self) -> Optional[str]:
        try:
            return self.out.get(timeout=0.1)
        except queue.Empty:
            return None

    def write(self, line: str) -> None:
        with self.lock:
            self._handle(line.strip())

    def close(self) -> None:
        pass

    # ------------------------------------------------------------ simulation

    def _reset(self) -> None:
        self.ax = {"Y": _Axis(FW_YAW_LIMITS), "P": _Axis(FW_PITCH_LIMITS)}
        self.enabled = True
        self.psi = getattr(self, "psi", 0.0)
        self.target = 0.0
        self.pstate = "IDLE"
        self.comp = False
        self.fault = 0
        self.hyst = 1.0
        self.uzjog = 0.4
        self.last_rx = time.time()
        self.link = False
        self.jog_at = 0.0
        self.pi_jog = False

    def fire(self) -> bool:
        """Simulate pulling the trigger."""
        with self.lock:
            if self.psi < 8:
                return False
            pre = self.psi
            self.psi = max(1.0, self.psi * 0.08)
            self.pstate = "IDLE"
            self.comp = False
            ya, pa = self.ax["Y"].pos, self.ax["P"].pos
            line = f"E SHOT pre={pre:.1f} post={self.psi:.1f} ya={ya:.2f} pa={pa:.2f}"
            if self.land_fn:
                line += f" sim_land={self.land_fn(ya, pa, pre):.1f}"
            self.out.put(line)
            return True

    def _run(self) -> None:
        last_status = 0.0
        t_prev = time.time()
        while not self._stop.wait(0.01):
            now = time.time()
            dt = min(now - t_prev, 0.05)
            t_prev = now
            with self.lock:
                self._tick(now, dt)
                if now - last_status >= 0.1:
                    last_status = now
                    self.out.put(self._status())

    def _tick(self, now: float, dt: float) -> None:
        if self.pi_jog and now - self.jog_at > 0.3:
            self.ax["Y"].mode = self.ax["P"].mode = "H"
            self.pi_jog = False
        if self.link and now - self.last_rx > 1.0:
            self.link = False
            self.out.put("E LINKLOST")
            if self.pstate in ("FILL", "HOLD"):
                self.pstate, self.comp = "IDLE", False
            for a in self.ax.values():
                if a.mode == "P":
                    a.mode = "H"
        if self.enabled:
            for a in self.ax.values():
                a.step(dt, self.uzjog)
        if self.comp:
            self.psi += self.fill_rate * dt
        self.psi = max(0.0, self.psi - 0.005 * dt)
        if self.psi >= PSI_CUTOFF and not self.fault:
            self.fault, self.comp, self.pstate = 1, False, "IDLE"
            self.out.put("E FAULT 1 OVERPRESSURE")
        if self.fault:
            return
        if self.pstate == "FILL":
            if self.psi >= self.target:
                self.pstate, self.comp = "HOLD", False
            else:
                self.comp = True
        elif self.pstate == "HOLD":
            self.comp = self.psi < self.target if self.comp else self.psi < self.target - self.hyst

    def _status(self) -> str:
        y, p = self.ax["Y"], self.ax["P"]
        yt = y.target if y.mode == "P" else y.pos
        pt = p.target if p.mode == "P" else p.pos
        mv = (1 if y.v else 0) | (2 if p.v else 0)
        z = (1 if y.zeroed else 0) | (2 if p.zeroed else 0)
        return (f"S ya={y.pos:.2f} pa={p.pos:.2f} yt={yt:.2f} pt={pt:.2f} md={y.mode}{p.mode} mv={mv} z={z} "
                f"en={int(self.enabled)} psi={self.psi:.1f} tgt={self.target:.1f} pst={self.pstate} "
                f"c={int(self.comp)} f={self.fault} j=0 adc={int((0.1 + 0.8 * self.psi / 80) * 1023)}")

    def _reply(self, cmd: str, err: Optional[str] = None) -> None:
        self.out.put(f"OK {cmd}" if err is None else f"ERR {cmd} {err}")

    def _handle(self, line: str) -> None:
        self.last_rx = time.time()
        self.link = True
        parts = line.split()
        if not parts:
            return
        cmd, args = parts[0], parts[1:]

        def num(i):
            return float(args[i])

        try:
            if cmd == "HB":
                return
            if cmd == "JOG":
                if not self.enabled:
                    return self._reply(cmd, "DISABLED")
                self.ax["Y"].mode = self.ax["P"].mode = "V"
                self.ax["Y"].jog, self.ax["P"].jog = max(-1, min(1, num(0))), max(-1, min(1, num(1)))
                self.pi_jog, self.jog_at = True, time.time()
                return
            if cmd == "MOVE":
                y, p = num(0), num(1)
                if not self.enabled:
                    return self._reply(cmd, "DISABLED")
                if not (self.ax["Y"].zeroed and self.ax["P"].zeroed):
                    return self._reply(cmd, "NOZERO")
                ay, ap = self.ax["Y"], self.ax["P"]
                if not (ay.min <= y <= ay.max and ap.min <= p <= ap.max):
                    return self._reply(cmd, "LIMIT")
                ay.target, ay.mode, ap.target, ap.mode = y, "P", p, "P"
                self.pi_jog = False
                return self._reply(cmd)
            if cmd == "STOP":
                for a in self.ax.values():
                    a.mode = "H"
                self.pi_jog = False
                return self._reply(cmd)
            if cmd == "ZERO":
                which = args[0]
                axes = {"Y": "Y", "P": "P", "B": "YP"}.get(which)
                if not axes:
                    return self._reply(cmd, "ARGS")
                if any(self.ax[a].v for a in axes):
                    return self._reply(cmd, "MOVING")
                for a in axes:
                    ax = self.ax[a]
                    ax.pos, ax.target, ax.mode, ax.zeroed = 0.0, 0.0, "H", True
                return self._reply(cmd)
            if cmd == "PRESS":
                psi = num(0)
                if self.fault:
                    return self._reply(cmd, "FAULT")
                if not 1 <= psi <= FW_MAX_PSI:
                    return self._reply(cmd, "RANGE")
                self.target = psi
                self.pstate = "HOLD" if self.psi >= psi else "FILL"
                return self._reply(cmd)
            if cmd == "PSTOP":
                self.pstate, self.comp = "IDLE", False
                return self._reply(cmd)
            if cmd == "CLEAR":
                if self.psi >= PSI_CUTOFF:
                    return self._reply(cmd, "ACTIVE")
                self.fault = 0
                return self._reply(cmd)
            if cmd == "ENABLE":
                self.enabled = num(0) != 0
                if not self.enabled:
                    for a in self.ax.values():
                        a.v, a.mode, a.zeroed = 0.0, "H", False
                return self._reply(cmd)
            if cmd == "SET":
                key, val = args[0], num(1)
                pre, _, name = key.partition(".")
                if pre in ("Y", "P"):
                    a = self.ax[pre]
                    if name == "VMAX":
                        a.vmax = val
                    elif name == "ACC":
                        a.acc = val
                    elif name == "JOG":
                        a.jog_speed = val
                    elif name == "MIN":
                        a.min = max(val, a.hard[0])
                    elif name == "MAX":
                        a.max = min(val, a.hard[1])
                elif key == "G.HYST":
                    self.hyst = val
                return self._reply(cmd)
            if cmd in ("SAVE", "GET"):
                return self._reply(cmd)
            if cmd == "VER":
                self.out.put("OK VER sim")
                return
            self._reply(cmd, "UNKNOWN")
        except (IndexError, ValueError):
            self._reply(cmd, "ARGS")
