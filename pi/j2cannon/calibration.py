"""Fitting the launcher model to real shots, and the other calibration maths."""

from __future__ import annotations

import copy
import math
from dataclasses import replace
from typing import Callable, Dict, List, Sequence, Tuple

from .aiming import Aimer
from .ballistics import FOOT
from .geometry import Mount, Vec

PARAM_BOUNDS = {
    "efficiency": (0.02, 1.0),
    "drag_coefficient": (0.05, 2.5),
}


def nelder_mead(f: Callable[[List[float]], float], x0: Sequence[float], step: float = 0.2,
                iters: int = 200, tol: float = 1e-7) -> Tuple[List[float], float]:
    n = len(x0)
    pts = [list(x0)]
    for i in range(n):
        p = list(x0)
        p[i] += step
        pts.append(p)
    vals = [f(p) for p in pts]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: vals[i])
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) < tol:
            break
        cen = [sum(p[j] for p in pts[:-1]) / n for j in range(n)]
        refl = [cen[j] + (cen[j] - pts[-1][j]) for j in range(n)]
        fr = f(refl)
        if fr < vals[0]:
            exp = [cen[j] + 2 * (cen[j] - pts[-1][j]) for j in range(n)]
            fe = f(exp)
            pts[-1], vals[-1] = (exp, fe) if fe < fr else (refl, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = refl, fr
        else:
            con = [cen[j] + 0.5 * (pts[-1][j] - cen[j]) for j in range(n)]
            fc = f(con)
            if fc < vals[-1]:
                pts[-1], vals[-1] = con, fc
            else:
                for i in range(1, n + 1):
                    pts[i] = [pts[0][j] + 0.5 * (pts[i][j] - pts[0][j]) for j in range(n)]
                    vals[i] = f(pts[i])
    best = min(range(n + 1), key=lambda i: vals[i])
    return pts[best], vals[best]


def _with_params(aimer: Aimer, params: Dict[str, float]) -> Aimer:
    a = copy.copy(aimer)
    if "efficiency" in params:
        a.launcher = replace(aimer.launcher, efficiency=params["efficiency"])
    if "drag_coefficient" in params:
        proj = replace(aimer.projectile, drag_coefficient=params["drag_coefficient"])
        a.projectile = proj
        a.k = aimer.k * params["drag_coefficient"] / aimer.projectile.drag_coefficient
    return a


def predict_shot(aimer: Aimer, shot: dict) -> float:
    """Predicted landing distance (ft) of a logged shot."""
    dist, _ = aimer.predict_landing(shot["yaw_deg"], shot["pitch_deg"], shot["psi"],
                                    shot.get("landing_height_ft", 0.0) * FOOT)
    return dist / FOOT


def usable_shots(shots: List[dict]) -> List[dict]:
    return [s for s in shots if s.get("measured_ft") and s.get("use", True)]


def fit_model(aimer: Aimer, shots: List[dict], params: Sequence[str]) -> dict:
    """Least-squares fit of efficiency and/or drag to measured landing distances.

    Error is relative (a 2 ft miss at 20 ft counts as much as 10 ft at 100 ft).
    Parameters are searched in log space and clamped to PARAM_BOUNDS.
    """
    data = usable_shots(shots)
    params = [p for p in params if p in PARAM_BOUNDS]
    if not params:
        raise ValueError("nothing to fit")
    if len(data) < len(params):
        raise ValueError(f"need at least {len(params)} measured shot(s), have {len(data)}")
    start = {"efficiency": aimer.launcher.efficiency, "drag_coefficient": aimer.projectile.drag_coefficient}

    def unpack(x):
        out = {}
        for name, val in zip(params, x):
            lo, hi = PARAM_BOUNDS[name]
            out[name] = min(max(math.exp(val), lo), hi)
        return out

    def cost(x):
        a = _with_params(aimer, unpack(x))
        err = 0.0
        for s in data:
            pred = predict_shot(a, s)
            err += ((pred - s["measured_ft"]) / s["measured_ft"]) ** 2
        return err / len(data)

    x0 = [math.log(start[p]) for p in params]
    best, _ = nelder_mead(cost, x0, step=0.3, iters=120 if len(params) > 1 else 60, tol=1e-9)
    fitted = unpack(best)
    a = _with_params(aimer, fitted)
    residuals = []
    for s in data:
        pred = predict_shot(a, s)
        residuals.append({"id": s["id"], "measured_ft": s["measured_ft"], "predicted_ft": round(pred, 1),
                          "error_ft": round(pred - s["measured_ft"], 1)})
    rms = math.sqrt(sum(r["error_ft"] ** 2 for r in residuals) / len(residuals))
    spread = max(s["pitch_deg"] for s in data) - min(s["pitch_deg"] for s in data)
    note = ""
    if "drag_coefficient" in params and spread < 10:
        note = "Shots span under 10 deg of elevation, so drag and efficiency are hard to tell apart. " \
               "Add shots at a clearly different angle."
    return {"params": {k: round(v, 4) for k, v in fitted.items()}, "rms_ft": round(rms, 2),
            "residuals": residuals, "note": note}


def corrected_gear_ratio(old_ratio: float, commanded_deg: float, measured_deg: float) -> float:
    """The axis moved measured_deg when told to move commanded_deg."""
    if abs(measured_deg) < 0.5 or abs(commanded_deg) < 0.5:
        raise ValueError("move at least a few degrees")
    if (measured_deg > 0) != (commanded_deg > 0):
        raise ValueError("measured direction is opposite: flip 'invert' for this axis in config.toml instead")
    return old_ratio * commanded_deg / measured_deg


def boresight(mount: Mount, yaw_deg: float, pitch_deg: float, marker: Vec, nx: float, ny: float) -> Tuple[float, float]:
    """Camera yaw/pitch offsets (deg) that make `marker` appear where it was clicked."""
    cy, cp = mount.cam_yaw_off, mount.cam_pitch_off

    def resid(cy_, cp_):
        m = replace(mount, cam_yaw_off=cy_, cam_pitch_off=cp_)
        pr = m.project(yaw_deg, pitch_deg, marker)
        if pr is None:
            raise ValueError("marker is behind the camera")
        return pr[0] - nx, pr[1] - ny

    for _ in range(20):
        r0 = resid(cy, cp)
        if abs(r0[0]) < 1e-6 and abs(r0[1]) < 1e-6:
            break
        h = 0.01
        ry = resid(cy + h, cp)
        rp = resid(cy, cp + h)
        j = ((ry[0] - r0[0]) / h, (rp[0] - r0[0]) / h), ((ry[1] - r0[1]) / h, (rp[1] - r0[1]) / h)
        det = j[0][0] * j[1][1] - j[0][1] * j[1][0]
        if abs(det) < 1e-12:
            break
        dy = (r0[0] * j[1][1] - r0[1] * j[0][1]) / det
        dp = (j[0][0] * r0[1] - j[1][0] * r0[0]) / det
        cy -= dy
        cp -= dp
    if abs(cy) > 30 or abs(cp) > 30:
        raise ValueError("offset came out over 30 deg - check the marker coordinates")
    return cy, cp

