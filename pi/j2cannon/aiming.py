"""Turn a target point into turret yaw, barrel pitch and chamber pressure."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from . import ballistics as bal
from .geometry import Mount, Vec, firing_yaw, plane_distance

FOOT = bal.FOOT


@dataclass
class Solution:
    feasible: bool
    reason: str = ""
    target: Optional[Vec] = None      # metres, world frame
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    psi: float = 0.0
    v0: float = 0.0                   # m/s
    flight_time: float = 0.0
    apex_m: float = 0.0
    arc: str = "low"
    strategy: str = ""
    reused_pressure: bool = False
    distance_m: float = 0.0           # horizontal, turret axis to target
    path: List[Tuple[float, float]] = field(default_factory=list)  # firing plane (x, height), metres

    def to_json(self) -> dict:
        d = {
            "feasible": self.feasible,
            "reason": self.reason,
            "yaw_deg": round(self.yaw_deg, 2),
            "pitch_deg": round(self.pitch_deg, 2),
            "psi": round(self.psi, 1),
            "v0_fps": round(self.v0 / FOOT, 1),
            "flight_time_s": round(self.flight_time, 2),
            "apex_ft": round(self.apex_m / FOOT, 1),
            "arc": self.arc,
            "strategy": self.strategy,
            "reused_pressure": self.reused_pressure,
            "distance_ft": round(self.distance_m / FOOT, 1),
            "path_ft": [(round(x / FOOT, 2), round(y / FOOT, 2)) for x, y in self.path],
        }
        if self.target:
            d["target_ft"] = [round(c / FOOT, 2) for c in self.target]
        return d


class Aimer:
    def __init__(self, mount: Mount, launcher: bal.Launcher, projectile: bal.Projectile, air_density: float,
                 yaw_limits: Tuple[float, float], pitch_limits: Tuple[float, float], min_psi: float,
                 lob_angle_deg: float, max_range_m: float):
        self.mount = mount
        self.launcher = launcher
        self.projectile = projectile
        self.k = projectile.drag_k(air_density)
        self.yaw_limits = yaw_limits
        self.pitch_limits = pitch_limits
        self.min_psi = min_psi
        self.lob_angle = math.radians(lob_angle_deg)
        self.max_range = max_range_m

    # ------------------------------------------------------------ helpers

    def launch_point(self, theta: float) -> Tuple[float, float]:
        """Muzzle (x, height) in the firing plane at elevation theta (rad)."""
        a, _, b = self.mount.muzzle
        return (a * math.cos(theta) - b * math.sin(theta),
                self.mount.pivot_height + a * math.sin(theta) + b * math.cos(theta))

    def velocity(self, psi: float) -> float:
        return self.launcher.velocity(psi, self.projectile.mass_kg)

    def pressure_for(self, v: float) -> Optional[float]:
        return self.launcher.pressure_for_velocity(v, self.projectile.mass_kg)

    def _theta_range(self) -> Tuple[float, float]:
        return math.radians(self.pitch_limits[0]), math.radians(self.pitch_limits[1])

    def _finish(self, sol: Solution, x_t: float, y_t: float) -> Solution:
        th = math.radians(sol.pitch_deg)
        x0, y0 = self.launch_point(th)
        shot = bal.fly_to_x(sol.v0, th, self.k, x0, y0, x_t, y_t, keep_path=True)
        step = max(1, len(shot.path) // 60)
        sol.path = shot.path[::step] + ([shot.path[-1]] if shot.path and (len(shot.path) - 1) % step else [])
        sol.flight_time = shot.time
        sol.apex_m = shot.apex
        return sol

    # ------------------------------------------------------------ solving

    def solve(self, target: Vec, strategy: str = "min_pressure", arc: str = "low", fixed_psi: float = 60.0,
              have_psi: Optional[float] = None, reuse: bool = True, guess: Optional["Solution"] = None) -> Solution:
        """Aim at a world point (metres).

        strategy "min_pressure": lowest pressure that reaches it (arc "high"
        lobs at lob_angle_deg instead). "fixed_pressure": fixed_psi, angle
        from the chosen arc. If reuse is set and the chamber already holds more
        than the answer needs (have_psi), re-aim for the pressure on hand
        instead, since there is no vent valve.
        """
        sol = Solution(False, target=target, arc=arc, strategy=strategy)
        dist = math.hypot(target[0], target[1])
        sol.distance_m = dist
        if dist > self.max_range:
            sol.reason = "target beyond max_range_ft"
            return sol
        s = self.mount.muzzle[1]
        yaw = firing_yaw(target, s)
        if yaw is None:
            sol.reason = "target too close"
            return sol
        if not (self.yaw_limits[0] <= yaw <= self.yaw_limits[1]):
            sol.reason = f"needs yaw {yaw:.1f} deg, outside {self.yaw_limits[0]:.0f}..{self.yaw_limits[1]:.0f}"
            return sol
        sol.yaw_deg = yaw
        x_t = plane_distance(target, s)
        y_t = target[2]
        lo, hi = self._theta_range()
        v_max = self.velocity(self.launcher.max_psi)
        guess_th = math.radians(guess.pitch_deg) if guess and guess.feasible else None

        theta = v = None
        if strategy == "fixed_pressure":
            psi = min(max(fixed_psi, self.min_psi), self.launcher.max_psi)
            v = self.velocity(psi)
            theta = bal.solve_angle(v, self.k, self.launch_point, x_t, y_t, lo, hi, arc)
            if theta is None:
                sol.reason = f"cannot reach at {psi:.0f} psi"
                return sol
            sol.psi = psi
        elif arc == "high":
            th = min(max(self.lob_angle, lo), hi)
            v = bal.required_speed(th, self.k, self.launch_point, x_t, y_t, v_max)
            if v is not None:
                theta = th
            else:  # lob angle needs too much pressure: steepest arc we can manage
                theta = bal.solve_angle(v_max, self.k, self.launch_point, x_t, y_t, lo, hi, "high")
                v = v_max
            if theta is None:
                sol.reason = f"out of range at {self.launcher.max_psi:.0f} psi"
                return sol
        else:
            r = bal.solve_min_speed(self.k, self.launch_point, x_t, y_t, lo, hi, v_max, guess_th)
            if r is None and guess_th is not None:
                r = bal.solve_min_speed(self.k, self.launch_point, x_t, y_t, lo, hi, v_max)
            if r is None or r[1] > v_max * 1.0001:
                sol.reason = f"out of range at {self.launcher.max_psi:.0f} psi"
                return sol
            theta, v = r

        if strategy != "fixed_pressure":
            psi = self.pressure_for(v)
            if psi is None:
                sol.reason = f"out of range at {self.launcher.max_psi:.0f} psi"
                return sol
            sol.psi = psi
            if psi < self.min_psi:
                # Too gentle to be repeatable: use the minimum pressure on the chosen arc.
                v = self.velocity(self.min_psi)
                theta = bal.solve_angle(v, self.k, self.launch_point, x_t, y_t, lo, hi, arc)
                if theta is None:
                    sol.reason = "target too close for minimum pressure"
                    return sol
                sol.psi = self.min_psi

        if reuse and have_psi is not None and have_psi > sol.psi + 1.0 and have_psi >= self.min_psi:
            v2 = self.velocity(min(have_psi, self.launcher.max_psi))
            th2 = bal.solve_angle(v2, self.k, self.launch_point, x_t, y_t, lo, hi, arc)
            if th2 is not None:
                theta, v, sol.psi, sol.reused_pressure = th2, v2, have_psi, True

        sol.pitch_deg = math.degrees(theta)
        sol.v0 = v
        sol.feasible = True
        return self._finish(sol, x_t, y_t)

    def trim(self, sol: Solution, actual_psi: float, current_pitch: float) -> Optional[Solution]:
        """Re-solve the elevation for the pressure actually in the chamber."""
        if not sol.feasible or sol.target is None:
            return None
        s = self.mount.muzzle[1]
        x_t = plane_distance(sol.target, s)
        y_t = sol.target[2]
        lo, hi = self._theta_range()
        v = self.velocity(actual_psi)
        options = [bal.solve_angle(v, self.k, self.launch_point, x_t, y_t, lo, hi, a) for a in ("low", "high")]
        options = [o for o in options if o is not None]
        if not options:
            return None
        theta = min(options, key=lambda o: abs(math.degrees(o) - current_pitch))
        new = Solution(True, target=sol.target, yaw_deg=sol.yaw_deg, pitch_deg=math.degrees(theta), psi=actual_psi,
                       v0=v, arc=sol.arc, strategy=sol.strategy, reused_pressure=sol.reused_pressure,
                       distance_m=sol.distance_m)
        return self._finish(new, x_t, y_t)

    def predict_landing(self, yaw_deg: float, pitch_deg: float, psi: float, landing_height_m: float = 0.0):
        """Where a shot would come down. Returns (horizontal distance from the
        turret axis in metres, world landing point)."""
        th = math.radians(pitch_deg)
        v = self.velocity(psi)
        x0, y0 = self.launch_point(th)
        xl, _, _, _ = bal.fly_to_height(v, th, self.k, x0, y0, landing_height_m)
        s = self.mount.muzzle[1]
        yaw = math.radians(yaw_deg)
        fwd = (math.cos(yaw), math.sin(yaw))
        right = (-math.sin(yaw), math.cos(yaw))
        p = (fwd[0] * xl + right[0] * s, fwd[1] * xl + right[1] * s, landing_height_m)
        return math.hypot(p[0], p[1]), p
