"""Load config.toml, apply calibration overrides, and build the model objects."""

from __future__ import annotations

import copy
import json
import math
import os
import threading
from typing import Any, Dict, List, Tuple

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # pragma: no cover - older Raspberry Pi OS
    import tomli as tomllib

from .ballistics import FOOT, INCH, Launcher, Projectile
from .geometry import Mount

# Firmware hard limits (firmware/j2_cannon/config.h). The Pi never asks for more.
FW_YAW_LIMITS = (-90.0, 90.0)
FW_PITCH_LIMITS = (-45.0, 90.0)
FW_MAX_PSI = 75.0


def deep_merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


class Config:
    """config.toml plus the overrides and shot log in data/calibration.json."""

    def __init__(self, path: str, data_dir: str | None = None):
        self.path = os.path.abspath(path)
        self.data_dir = data_dir or os.path.join(os.path.dirname(self.path), "data")
        self.cal_path = os.path.join(self.data_dir, "calibration.json")
        self._lock = threading.RLock()
        with open(self.path, "rb") as f:
            self.base = tomllib.load(f)
        self.cal: Dict[str, Any] = {"overrides": {}, "shots": []}
        if os.path.exists(self.cal_path):
            with open(self.cal_path) as f:
                loaded = json.load(f)
            self.cal["overrides"] = loaded.get("overrides", {})
            self.cal["shots"] = loaded.get("shots", [])
        self.data = deep_merge(self.base, self.cal["overrides"])

    # ------------------------------------------------------------ access

    def __getitem__(self, section: str) -> Dict[str, Any]:
        return self.data[section]

    def get(self, section: str, key: str, default=None):
        return self.data.get(section, {}).get(key, default)

    def set_override(self, section: str, key: str, value) -> None:
        with self._lock:
            self.cal["overrides"].setdefault(section, {})[key] = value
            self.data = deep_merge(self.base, self.cal["overrides"])
            self.save()

    def shots(self) -> List[Dict[str, Any]]:
        return self.cal["shots"]

    def save(self) -> None:
        with self._lock:
            os.makedirs(self.data_dir, exist_ok=True)
            tmp = self.cal_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.cal, f, indent=2)
            os.replace(tmp, self.cal_path)

    # ------------------------------------------------------------ derived models

    def steps_per_deg(self, axis: str) -> float:
        a = self.data[axis]
        return a["motor_steps_per_rev"] * a["microsteps"] * a["gear_ratio"] / 360.0

    def limits(self, axis: str) -> Tuple[float, float]:
        hard = FW_YAW_LIMITS if axis == "turret" else FW_PITCH_LIMITS
        a = self.data[axis]
        return max(a["min_deg"], hard[0]), min(a["max_deg"], hard[1])

    def max_psi(self) -> float:
        return min(self.data["pressure"]["max_psi"], FW_MAX_PSI)

    def mount(self) -> Mount:
        g = self.data["geometry"]
        ll = self.data["limelight"]
        return Mount(
            pivot_height=g["pivot_height_ft"] * FOOT,
            muzzle=(g["muzzle_forward_in"] * INCH, g["muzzle_right_in"] * INCH, g["muzzle_up_in"] * INCH),
            camera=(g["camera_forward_in"] * INCH, g["camera_right_in"] * INCH, g["camera_up_in"] * INCH),
            cam_yaw_off=g["camera_yaw_offset_deg"],
            cam_pitch_off=g["camera_pitch_offset_deg"],
            hfov=ll["hfov_deg"],
            vfov=ll["vfov_deg"],
        )

    def launcher(self) -> Launcher:
        l = self.data["launcher"]
        chamber = sum(math.pi / 4 * (d * INCH) ** 2 * (length * INCH) for length, d in l["chamber"])
        barrel = math.pi / 4 * (l["barrel_bore_in"] * INCH) ** 2 * (l["barrel_length_in"] * INCH)
        return Launcher(chamber, barrel, l["efficiency"], self.max_psi())

    def projectile(self) -> Projectile:
        p = self.data["projectile"]
        return Projectile(p["mass_g"] / 1000.0, p["diameter_in"] * INCH, p["drag_coefficient"])

    def firmware_settings(self) -> List[Tuple[str, float]]:
        """SET commands that make the RedBoard match this config."""
        out = []
        for axis, pre in (("turret", "Y"), ("pitch", "P")):
            a = self.data[axis]
            lo, hi = self.limits(axis)
            hard = FW_YAW_LIMITS if axis == "turret" else FW_PITCH_LIMITS
            out += [
                (f"{pre}.SPD", self.steps_per_deg(axis)),
                # Open the window fully first so MIN < MAX holds whatever was stored before.
                (f"{pre}.MIN", hard[0]),
                (f"{pre}.MAX", hard[1]),
                (f"{pre}.VMAX", a["max_speed_dps"]),
                (f"{pre}.ACC", a["accel_dps2"]),
                (f"{pre}.JOG", a["jog_speed_dps"]),
                (f"{pre}.MIN", lo),
                (f"{pre}.MAX", hi),
                (f"{pre}.INV", 1 if a["invert"] else 0),
            ]
        p = self.data["pressure"]
        j = self.data["joystick"]
        out += [
            ("G.HYST", p["hold_hysteresis_psi"]),
            ("G.PCOMP", p["fill_overshoot_psi"]),
            ("G.STALL", p["fill_stall_s"]),
            ("G.MAXRUN", p["fill_max_run_s"]),
            ("G.JXINV", 1 if j["invert_x"] else 0),
            ("G.JYINV", 1 if j["invert_y"] else 0),
        ]
        return out
