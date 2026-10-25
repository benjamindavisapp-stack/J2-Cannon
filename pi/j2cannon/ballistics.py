"""Exterior and interior ballistics for the cannon.

Everything here is SI (metres, seconds, kilograms, radians) except where a
name says psi or deg.

Interior: chamber air at gauge pressure P expands adiabatically into the
barrel behind the projectile. A fixed efficiency (fitted from real shots)
turns that expansion work into muzzle energy.

Exterior: point mass with quadratic drag, integrated with RK4. The shot
travels in the vertical "firing plane" along the barrel's azimuth.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional

G = 9.80665
PSI_PA = 6894.757
P_ATM = 101325.0
GAMMA = 1.4
INCH = 0.0254
FOOT = 0.3048

GOLDEN = (math.sqrt(5) - 1) / 2


@dataclass
class Launcher:
    chamber_volume_m3: float
    barrel_volume_m3: float
    efficiency: float
    max_psi: float

    def energy_j(self, psi: float) -> float:
        """Work done on the projectile by the expanding chamber air, before losses."""
        if psi <= 0:
            return 0.0
        p1 = psi * PSI_PA + P_ATM
        v1 = self.chamber_volume_m3
        v2 = v1 + self.barrel_volume_m3
        work = p1 * v1 / (GAMMA - 1) * (1 - (v1 / v2) ** (GAMMA - 1)) - P_ATM * (v2 - v1)
        return max(work, 0.0)

    def velocity(self, psi: float, mass_kg: float) -> float:
        return math.sqrt(2 * self.efficiency * self.energy_j(psi) / mass_kg)

    def pressure_for_velocity(self, v: float, mass_kg: float) -> Optional[float]:
        """Gauge psi needed for muzzle speed v, or None if above max_psi."""
        if v <= 0:
            return 0.0
        if self.velocity(self.max_psi, mass_kg) < v:
            return None
        lo, hi = 0.0, self.max_psi
        for _ in range(40):
            mid = (lo + hi) / 2
            if self.velocity(mid, mass_kg) < v:
                lo = mid
            else:
                hi = mid
        return hi


@dataclass
class Projectile:
    mass_kg: float
    diameter_m: float
    drag_coefficient: float

    def drag_k(self, air_density: float) -> float:
        """k in a_drag = -k |v| v."""
        area = math.pi * self.diameter_m ** 2 / 4
        return 0.5 * air_density * self.drag_coefficient * area / self.mass_kg


@dataclass
class Shot:
    """Result of one integrated trajectory."""

    y_at_target: float          # height at the target's horizontal distance (or a penalty if short)
    reached: bool
    time: float                 # flight time to the target distance (or until stopped)
    apex: float                 # highest point reached
    path: list = field(default_factory=list)


def _deriv(vx: float, vy: float, k: float):
    sp = math.sqrt(vx * vx + vy * vy)
    return -k * sp * vx, -k * sp * vy - G


def fly_to_x(v0: float, theta: float, k: float, x0: float, y0: float, x_target: float,
             y_target: float, keep_path: bool = False) -> Shot:
    """Integrate until the shot reaches x_target. If it is already falling below
    y_target first, stop early and return a penalty height that keeps the result
    monotonic for the root finders (the shot can only get lower from there)."""
    vx = v0 * math.cos(theta)
    vy = v0 * math.sin(theta)
    x, y, t = x0, y0, 0.0
    apex = y
    path = [(x, y)] if keep_path else None
    dist = max(x_target - x0, 0.1)
    dt = min(max(dist / max(vx, 1.0) / 120.0, 0.001), 0.02)
    if x >= x_target:
        return Shot(y, True, 0.0, y, path or [])
    while t < 30.0:
        k1x, k1y = _deriv(vx, vy, k)
        k2x, k2y = _deriv(vx + 0.5 * dt * k1x, vy + 0.5 * dt * k1y, k)
        k3x, k3y = _deriv(vx + 0.5 * dt * k2x, vy + 0.5 * dt * k2y, k)
        k4x, k4y = _deriv(vx + dt * k3x, vy + dt * k3y, k)
        nvx = vx + dt / 6 * (k1x + 2 * k2x + 2 * k3x + k4x)
        nvy = vy + dt / 6 * (k1y + 2 * k2y + 2 * k3y + k4y)
        nx = x + dt / 6 * (vx + 2 * (vx + 0.5 * dt * k1x) + 2 * (vx + 0.5 * dt * k2x) + (vx + dt * k3x))
        ny = y + dt / 6 * (vy + 2 * (vy + 0.5 * dt * k1y) + 2 * (vy + 0.5 * dt * k2y) + (vy + dt * k3y))
        if nx >= x_target:
            f = (x_target - x) / (nx - x)
            yt = y + f * (ny - y)
            if keep_path:
                path.append((x_target, yt))
            return Shot(yt, True, t + f * dt, max(apex, yt), path or [])
        x, y, vx, vy, t = nx, ny, nvx, nvy, t + dt
        apex = max(apex, y)
        if keep_path:
            path.append((x, y))
        if vy < 0 and y < y_target:
            # Falling and already below the target: report how far short it is.
            short = x_target - x
            return Shot(y - short * (1 + abs(vy) / max(vx, 0.1)), False, t, apex, path or [])
    return Shot(y - (x_target - x), False, t, apex, path or [])


def fly_to_height(v0: float, theta: float, k: float, x0: float, y0: float, y_land: float,
                  keep_path: bool = False):
    """Integrate until the shot comes down through y_land. Returns (x, time, apex, path)."""
    vx = v0 * math.cos(theta)
    vy = v0 * math.sin(theta)
    x, y, t = x0, y0, 0.0
    apex = y
    path = [(x, y)] if keep_path else []
    dt = 0.005
    while t < 60.0:
        k1x, k1y = _deriv(vx, vy, k)
        k2x, k2y = _deriv(vx + 0.5 * dt * k1x, vy + 0.5 * dt * k1y, k)
        k3x, k3y = _deriv(vx + 0.5 * dt * k2x, vy + 0.5 * dt * k2y, k)
        k4x, k4y = _deriv(vx + dt * k3x, vy + dt * k3y, k)
        nvx = vx + dt / 6 * (k1x + 2 * k2x + 2 * k3x + k4x)
        nvy = vy + dt / 6 * (k1y + 2 * k2y + 2 * k3y + k4y)
        nx = x + dt / 6 * (vx + 2 * (vx + 0.5 * dt * k1x) + 2 * (vx + 0.5 * dt * k2x) + (vx + dt * k3x))
        ny = y + dt / 6 * (vy + 2 * (vy + 0.5 * dt * k1y) + 2 * (vy + 0.5 * dt * k2y) + (vy + dt * k3y))
        if nvy < 0 and ny <= y_land < y:
            f = (y - y_land) / (y - ny)
            xl = x + f * (nx - x)
            if keep_path:
                path.append((xl, y_land))
            return xl, t + f * dt, apex, path
        x, y, vx, vy, t = nx, ny, nvx, nvy, t + dt
        apex = max(apex, y)
        if keep_path:
            path.append((x, y))
        if vy < 0 and y < y_land:  # started below the landing height and never rose above it
            return x, t, apex, path
    return x, t, apex, path


# --------------------------------------------------------------------------- solvers

LaunchPoint = Callable[[float], tuple]  # theta -> (x0, y0) of the muzzle in the firing plane


def _golden_max(f, lo: float, hi: float, tol: float):
    """Maximise a unimodal f on [lo, hi]. Returns (x, f(x))."""
    a, b = lo, hi
    c = b - GOLDEN * (b - a)
    d = a + GOLDEN * (b - a)
    fc, fd = f(c), f(d)
    while b - a > tol:
        if fc > fd:
            b, d, fd = d, c, fc
            c = b - GOLDEN * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + GOLDEN * (b - a)
            fd = f(d)
    return (c, fc) if fc > fd else (d, fd)


def _golden_min(f, lo: float, hi: float, tol: float):
    x, fx = _golden_max(lambda t: -f(t), lo, hi, tol)
    return x, -fx


def _bisect(f, lo: float, hi: float, flo: float, tol: float, max_iter: int = 60) -> float:
    """Root of f in [lo, hi] given f(lo) = flo with opposite sign to f(hi).
    Uses false position with the Illinois tweak, falling back to bisection."""
    fhi = f(hi)
    a, b, fa, fb = lo, hi, flo, fhi
    side = 0
    for _ in range(max_iter):
        if fb == fa:
            m = (a + b) / 2
        else:
            m = b - fb * (b - a) / (fb - fa)
            if not (min(a, b) < m < max(a, b)):
                m = (a + b) / 2
        fm = f(m)
        if abs(fm) < 1e-4 or abs(b - a) < tol:
            return m
        if (fm > 0) == (fb > 0):
            b, fb = m, fm
            if side == -1:
                fa /= 2
            side = -1
        else:
            a, fa = m, fm
            if side == 1:
                fb /= 2
            side = 1
    return (a + b) / 2


def height_error(v0: float, theta: float, k: float, launch: LaunchPoint, x_t: float, y_t: float) -> float:
    x0, y0 = launch(theta)
    return fly_to_x(v0, theta, k, x0, y0, x_t, y_t).y_at_target - y_t


def solve_angle(v0: float, k: float, launch: LaunchPoint, x_t: float, y_t: float,
                theta_lo: float, theta_hi: float, arc: str, tol: float = 2e-4) -> Optional[float]:
    """Elevation (rad) that puts a shot of speed v0 through (x_t, y_t), or None.

    arc="low" returns the flatter of the two solutions, "high" the lob. If only
    one exists inside [theta_lo, theta_hi] it is returned for either arc.
    """
    hi = min(theta_hi, math.radians(85))
    if hi <= theta_lo:
        return None
    f = lambda th: height_error(v0, th, k, launch, x_t, y_t)
    peak, fpeak = _golden_max(f, theta_lo, hi, math.radians(0.2))
    if fpeak < 0:
        return None
    flo, fhi = f(theta_lo), f(hi)
    low = _bisect(f, theta_lo, peak, flo, tol) if flo < 0 else None
    high = _bisect(f, hi, peak, fhi, tol) if fhi < 0 else None
    if arc == "high":
        return high if high is not None else low
    return low if low is not None else high


def required_speed(theta: float, k: float, launch: LaunchPoint, x_t: float, y_t: float,
                   v_hi: float, v_guess: Optional[float] = None) -> Optional[float]:
    """Muzzle speed that reaches (x_t, y_t) at elevation theta, or None if above v_hi."""
    f = lambda v: height_error(v, theta, k, launch, x_t, y_t)
    if f(v_hi) < 0:
        return None
    lo = 0.5
    if v_guess and 0.5 < v_guess < v_hi:
        # Bracket around the guess to save iterations when tracking.
        g_lo, g_hi = v_guess * 0.95, min(v_guess * 1.05, v_hi)
        f_lo = f(g_lo)
        if f_lo < 0 <= f(g_hi):
            return _bisect(f, g_lo, g_hi, f_lo, 1e-3)
    return _bisect(f, lo, v_hi, f(lo), 1e-3)


def solve_min_speed(k: float, launch: LaunchPoint, x_t: float, y_t: float, theta_lo: float,
                    theta_hi: float, v_hi: float, theta_guess: Optional[float] = None):
    """Elevation and speed with the lowest muzzle speed that still reaches the target.
    Returns (theta, v) or None."""
    x0, y0 = launch(0.0)
    los = math.atan2(y_t - y0, max(x_t - x0, 0.01))
    lo = max(theta_lo, los + math.radians(0.5))
    hi = min(theta_hi, math.radians(85))
    if theta_guess is not None:
        lo = max(lo, theta_guess - math.radians(6))
        hi = min(hi, theta_guess + math.radians(6))
    if hi <= lo:
        return None
    cache = {}
    big = v_hi * 10

    def speed(th):
        v = required_speed(th, k, launch, x_t, y_t, big, cache.get("last"))
        if v is None:
            return big
        cache["last"] = v
        return v

    th, v = _golden_min(speed, lo, hi, math.radians(0.05))
    if v >= big:
        return None
    return th, v
