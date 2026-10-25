"""The cannon's brain: turns UI requests into firmware commands.

Modes:
  manual  joystick / on-screen jog only
  aim     moving to a fixed solution, then pressurising
  track   re-solving from Limelight detections a few times a second

"Armed" means the app may start the compressor for the current solution.
A shot disarms it; nothing refills until you aim again or press Re-arm.
"""

from __future__ import annotations

import collections
import logging
import math
import threading
import time
from typing import Optional

from . import calibration
from .aiming import Aimer, Solution
from .ballistics import FOOT
from .config import Config
from .geometry import (Mount, add, camera_frame_to_local, intersect_height, matvec, point_at_distance)

log = logging.getLogger(__name__)

ARRIVE_DEG = 0.2


class Controller:
    def __init__(self, cfg: Config, link, camera, sim_fw=None):
        self.cfg = cfg
        self.link = link
        self.camera = camera
        self.sim_fw = sim_fw
        self.lock = threading.RLock()
        self.mode = "manual"
        self.phase = ""
        self.armed = False
        self.target = None
        self.target_source = ""
        self.solution: Optional[Solution] = None
        self.messages = collections.deque(maxlen=12)
        self.track_method = "ground"
        self._track_point = None
        self._track_seen = 0.0
        self._last_track = 0.0
        self._last_trim = 0.0
        self._sent = None          # (yaw, pitch) last MOVE we sent
        self.axis_test = None
        self.fit_result = None
        self.shots_version = 0
        b = cfg["ballistics"]
        a = cfg["aim"]
        self.strategy = {
            "strategy": b["strategy"],
            "fixed_psi": float(b["fixed_pressure_psi"]),
            "arc": b["arc"],
            "reuse": bool(b["reuse_pressure"]),
            "auto_pressurize": bool(a["auto_pressurize"]),
            "trim": bool(a["trim_to_actual_pressure"]),
        }
        self.rebuild()
        self._stop = threading.Event()

    def rebuild(self) -> None:
        """Rebuild the models after a config/calibration change."""
        cfg = self.cfg
        self.mount: Mount = cfg.mount()
        self.aimer = Aimer(self.mount, cfg.launcher(), cfg.projectile(), cfg["ballistics"]["air_density_kg_m3"],
                           cfg.limits("turret"), cfg.limits("pitch"), cfg["pressure"]["min_psi"],
                           cfg["ballistics"]["lob_angle_deg"], cfg["ballistics"]["max_range_ft"] * FOOT)
        if hasattr(self.camera, "mount"):
            self.camera.mount = self.mount

    def start(self) -> None:
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    # ------------------------------------------------------------ messages

    def msg(self, text: str, level: str = "info") -> None:
        log.info("%s: %s", level, text)
        self.messages.append({"t": time.time(), "text": text, "level": level})

    # ------------------------------------------------------------ firmware callbacks

    def on_connect(self, link) -> None:
        errors = link.push_settings(self.cfg.firmware_settings())
        if errors:
            self.msg("Some settings were refused by the RedBoard: " + "; ".join(errors), "error")
        else:
            self.msg(f"RedBoard connected (firmware {link.fw_version}). Jog to zero, then set zero.")

    def on_event(self, ev: dict) -> None:
        t = ev["type"]
        with self.lock:
            if t == "BOOT":
                self.mode, self.phase, self.armed = "manual", "", False
                self._sent = None
            elif t == "SHOT":
                self._record_shot(ev)
                self.armed = False
                self.phase = "fired"
                self.msg(f"Shot detected at {ev.get('pre', 0):.1f} psi. Enter where it landed on the Calibrate tab.")
            elif t == "JOY":
                if self.mode != "manual":
                    self.msg("Joystick took over; auto aim cancelled.", "warn")
                self.mode, self.phase = "manual", ""
            elif t == "FAULT":
                self.armed = False
                name = ev["args"][1] if len(ev["args"]) > 1 else "?"
                self.msg(f"FAULT: {name}. Compressor locked out until cleared.", "error")
            elif t == "FAULTCLR":
                self.msg("Fault cleared from the joystick.")
            elif t == "LINKLOST":
                self.msg("RedBoard reported the Pi link dropped; auto moves and pressurising stopped.", "warn")
            elif t == "ZERO":
                self.msg("Zero set from the joystick.")

    # ------------------------------------------------------------ helpers

    def _status(self):
        return self.link.status if self.link.fresh() else None

    def _pose(self):
        st = self._status()
        if not st:
            return None
        return float(st.get("ya", 0)), float(st.get("pa", 0))

    def _zeroed(self) -> bool:
        st = self._status()
        return bool(st) and int(st.get("z", 0)) == 3

    def _have_psi(self) -> Optional[float]:
        st = self._status()
        if not st or not self.strategy["reuse"]:
            return None
        return float(st.get("psi", 0))

    def _solve(self, point, guess=None) -> Solution:
        s = self.strategy
        return self.aimer.solve(point, s["strategy"], s["arc"], s["fixed_psi"], self._have_psi(), s["reuse"], guess)

    def _move(self, yaw: float, pitch: float):
        ok, reason = self.link.command(f"MOVE {yaw:.2f} {pitch:.2f}")
        if ok:
            self._sent = (yaw, pitch)
        return ok, reason

    def _press(self, psi: float):
        return self.link.command(f"PRESS {psi:.1f}")

    @staticmethod
    def _explain(reason: str) -> str:
        return {
            "NOZERO": "set zero for both axes first",
            "JOY": "joystick is in use",
            "LIMIT": "outside the angle limits",
            "DISABLED": "motors are disabled",
            "FAULT": "a pressure fault is active",
            "RANGE": "pressure out of range",
            "MOVING": "stop moving first",
            "TIMEOUT": "RedBoard did not answer",
        }.get(reason, reason)

    def _locate(self, det, pose, method: str):
        """World point (m) of a Limelight detection."""
        if method == "tag":
            if not det.tag_cam:
                return None, "no AprilTag pose (use an AprilTag pipeline with 3D enabled)"
            local = camera_frame_to_local(det.tag_cam)
            p = add(self.mount.camera_position(*pose), matvec(self.mount.camera_rotation(*pose), local))
            return p, ""
        origin, d = self.mount.world_ray(pose[0], pose[1], self.mount.angle_dir(det.tx, det.ty))
        h = self.cfg["aim"]["target_height_ft"] * FOOT
        p = intersect_height(origin, d, h, self.aimer.max_range)
        if p is None:
            return None, "target is above the horizon, so flat-ground range does not work"
        return p, ""

    # ------------------------------------------------------------ aiming

    def aim_at_point(self, point, source: str) -> dict:
        if not self.link.connected:
            return self._fail("RedBoard not connected")
        if not self._zeroed():
            return self._fail("Set zero for both axes first (jog to straight ahead and level, then Zero).")
        sol = self._solve(point)
        with self.lock:
            self.target, self.target_source, self.solution = point, source, sol
            self.mode, self.phase = "manual", ""
        if not sol.feasible:
            return self._fail(f"No solution: {sol.reason}")
        # Never hold the lock while waiting on the RedBoard: its reader thread
        # needs the lock to deliver events, and the reply would time out.
        ok, reason = self._move(sol.yaw_deg, sol.pitch_deg)
        if not ok:
            return self._fail(f"Move refused: {self._explain(reason)}")
        with self.lock:
            self.mode, self.phase, self.armed = "aim", "moving", True
            self._last_trim = 0.0
        if self.strategy["auto_pressurize"]:
            ok, reason = self._press(sol.psi)
            if not ok:
                self.msg(f"Could not start pressurising: {self._explain(reason)}", "error")
        extra = " (re-aimed for pressure already in the chamber)" if sol.reused_pressure else ""
        self.msg(f"Aiming at {sol.distance_m / FOOT:.0f} ft: yaw {sol.yaw_deg:.1f}, pitch {sol.pitch_deg:.1f}, "
                 f"{sol.psi:.1f} psi{extra}.")
        return {"ok": True, "solution": sol.to_json()}

    def aim_at_coords(self, forward_ft: float, right_ft: float, height_ft: float) -> dict:
        return self.aim_at_point((forward_ft * FOOT, right_ft * FOOT, height_ft * FOOT),
                                 f"coordinates ({forward_ft:g}, {right_ft:g}, {height_ft:g}) ft")

    def aim_at_click(self, nx: float, ny: float, method: str, distance_ft: Optional[float] = None,
                     height_ft: Optional[float] = None) -> dict:
        pose = self._pose()
        if pose is None:
            return self._fail("No status from the RedBoard")
        if method == "tag":
            det = self.camera.latest
            if not det.valid:
                return self._fail("No AprilTag in view")
            p, err = self._locate(det, self.link.pose_at(det.time) or pose, "tag")
        else:
            origin, d = self.mount.world_ray(pose[0], pose[1], self.mount.pixel_dir(nx, ny))
            if method == "distance":
                if not distance_ft or distance_ft <= 0:
                    return self._fail("Enter a distance for click + distance mode")
                p = point_at_distance(origin, d, distance_ft * FOOT)
                err = "could not place the click at that distance"
            else:
                h = (height_ft if height_ft is not None else self.cfg["aim"]["target_height_ft"]) * FOOT
                p = intersect_height(origin, d, h, self.aimer.max_range)
                err = "that point is above the horizon (or beyond max range); use click + distance"
        if p is None:
            return self._fail(err)
        return self.aim_at_point(p, f"click ({method})")

    def look_at_target(self) -> dict:
        if self.target is None:
            return self._fail("No target yet")
        pose = self._pose() or (0.0, 0.0)
        yaw, pitch = self.mount.look_angles(self.target, *pose)
        ylo, yhi = self.cfg.limits("turret")
        plo, phi = self.cfg.limits("pitch")
        yaw, pitch = min(max(yaw, ylo), yhi), min(max(pitch, plo), phi)
        with self.lock:
            self.mode, self.phase = "manual", ""
        ok, reason = self._move(yaw, pitch)
        return {"ok": True} if ok else self._fail(f"Move refused: {self._explain(reason)}")

    def rearm(self) -> dict:
        with self.lock:
            if self.mode == "track":
                self.armed = True
                return {"ok": True}
            if not self.solution or not self.solution.feasible:
                return self._fail("Nothing to re-arm: aim first")
            self.armed = True
            self.phase = "moving"
            sol = self.solution
        ok, reason = self._move(sol.yaw_deg, sol.pitch_deg)
        if ok:
            ok, reason = self._press(sol.psi)
        if not ok:
            return self._fail(self._explain(reason))
        with self.lock:
            self.mode = "aim"
        return {"ok": True}

    def track_start(self, method: str) -> dict:
        if not self._zeroed():
            return self._fail("Set zero for both axes first")
        with self.lock:
            self.track_method = method
            self.mode, self.phase, self.armed = "track", "searching", True
            self._track_point = None
            self._track_seen = time.time()
        self.msg(f"Tracking ({'AprilTag range' if method == 'tag' else 'flat-ground range'}).")
        return {"ok": True}

    def track_stop(self) -> dict:
        with self.lock:
            if self.mode == "track":
                self.mode, self.phase = "manual", ""
        self.link.command("STOP")
        return {"ok": True}

    def stop_all(self) -> dict:
        with self.lock:
            self.mode, self.phase, self.armed = "manual", "", False
        self.link.command("STOP")
        self.link.command("PSTOP")
        self.msg("Stopped: motion and compressor.", "warn")
        return {"ok": True}

    def _fail(self, text: str) -> dict:
        self.msg(text, "error")
        return {"ok": False, "error": text}

    # ------------------------------------------------------------ manual

    def jog(self, yaw: float, pitch: float) -> dict:
        with self.lock:
            if self.mode != "manual" and (yaw or pitch):
                self.mode, self.phase = "manual", ""
        try:
            self.link.send(f"JOG {max(-1, min(1, yaw)):.2f} {max(-1, min(1, pitch)):.2f}")
        except Exception as e:
            return {"ok": False, "error": str(e)}
        return {"ok": True}

    def simple(self, command: str, success: str = "") -> dict:
        ok, reason = self.link.command(command)
        if not ok:
            return self._fail(f"{command.split()[0]} refused: {self._explain(reason)}")
        if success:
            self.msg(success)
        return {"ok": True}

    def pressurize(self, psi: float) -> dict:
        if not 1 <= psi <= self.cfg.max_psi():
            return self._fail(f"Pressure must be 1..{self.cfg.max_psi():.0f} psi")
        return self.simple(f"PRESS {psi:.1f}", f"Filling to {psi:.1f} psi.")

    def set_strategy(self, **kw) -> dict:
        with self.lock:
            for k, v in kw.items():
                if k in self.strategy and v is not None:
                    self.strategy[k] = type(self.strategy[k])(v)
        return {"ok": True, "strategy": dict(self.strategy)}

    # ------------------------------------------------------------ loop

    def _loop(self) -> None:
        while not self._stop.wait(0.1):
            try:
                self._tick()
            except Exception:
                log.exception("controller tick failed")

    def _tick(self) -> None:
        st = self._status()
        if not st:
            return
        now = time.time()
        if self.mode == "aim" and self.solution and self.solution.feasible:
            self._tick_aim(st, now)
        elif self.mode == "track" and now - self._last_track >= 1.0 / self.cfg["aim"]["track_rate_hz"]:
            self._last_track = now
            self._tick_track(st, now)

    def _arrived(self, st, sol) -> bool:
        return (int(st.get("mv", 0)) == 0 and abs(float(st["ya"]) - sol.yaw_deg) < ARRIVE_DEG
                and abs(float(st["pa"]) - sol.pitch_deg) < ARRIVE_DEG)

    def _tick_aim(self, st, now: float) -> None:
        sol = self.solution
        if self.phase == "fired":
            return
        arrived = self._arrived(st, sol)
        pst = st.get("pst")
        psi = float(st.get("psi", 0))
        hyst = self.cfg["pressure"]["hold_hysteresis_psi"]
        at_pressure = pst == "HOLD" and psi >= float(st.get("tgt", 0)) - hyst
        if (arrived and at_pressure and self.strategy["trim"] and abs(psi - sol.psi) > 0.5
                and now - self._last_trim > 2.0):
            self._last_trim = now
            new = self.aimer.trim(sol, psi, float(st["pa"]))
            if new and abs(new.pitch_deg - sol.pitch_deg) > 0.05:
                ok, _ = self._move(new.yaw_deg, new.pitch_deg)
                if ok:
                    with self.lock:
                        self.solution = new
                    arrived = False
        with self.lock:
            if not arrived:
                self.phase = "moving"
            elif at_pressure:
                self.phase = "ready"
            elif self.armed and self.strategy["auto_pressurize"]:
                self.phase = "pressurizing"
            else:
                self.phase = "aimed"

    def _tick_track(self, st, now: float) -> None:
        det = self.camera.latest
        if not det.valid or now - det.time > 0.6:
            with self.lock:
                if now - self._track_seen > 1.0:
                    self.phase = "lost" if self._track_point else "searching"
            return
        pose = self.link.pose_at(det.time) or (float(st["ya"]), float(st["pa"]))
        p, err = self._locate(det, pose, self.track_method)
        if p is None:
            with self.lock:
                self.phase = "no range"
            return
        self._track_seen = now
        prev = self._track_point
        if prev and math.dist(prev, p) < 3.0:
            p = tuple(0.5 * a + 0.5 * b for a, b in zip(prev, p))  # light smoothing
        self._track_point = p
        sol = self._solve(p, guess=self.solution)
        with self.lock:
            self.target, self.target_source, self.solution = p, f"tracking ({self.track_method})", sol
        if not sol.feasible:
            with self.lock:
                self.phase = "out of range"
            return
        db = self.cfg["aim"]["track_deadband_deg"]
        if self._sent is None or abs(sol.yaw_deg - self._sent[0]) > db or abs(sol.pitch_deg - self._sent[1]) > db:
            self._move(sol.yaw_deg, sol.pitch_deg)
        if self.armed and self.strategy["auto_pressurize"]:
            if st.get("pst") not in ("FILL", "HOLD") or abs(float(st.get("tgt", 0)) - sol.psi) > 1.0:
                self._press(sol.psi)
        psi = float(st.get("psi", 0))
        at_pressure = st.get("pst") == "HOLD" and psi >= sol.psi - self.cfg["pressure"]["hold_hysteresis_psi"]
        with self.lock:
            self.phase = "ready" if self._arrived(st, sol) and at_pressure else "tracking"

    # ------------------------------------------------------------ shots & calibration

    def _record_shot(self, ev: dict) -> None:
        st = self.link.status or {}
        yaw = float(ev.get("ya", st.get("ya", 0)))
        pitch = float(ev.get("pa", st.get("pa", 0)))
        pre = float(ev.get("pre", 0))
        shots = self.cfg.shots()
        shot = {
            "id": max([s["id"] for s in shots], default=0) + 1,
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "yaw_deg": round(yaw, 2),
            "pitch_deg": round(pitch, 2),
            "psi": round(pre, 1),
            "post_psi": round(float(ev.get("post", 0)), 1),
            "landing_height_ft": 0.0,
            "measured_ft": ev.get("sim_land"),
            "use": True,
        }
        try:
            shot["predicted_ft"] = round(calibration.predict_shot(self.aimer, shot), 1)
        except Exception:
            shot["predicted_ft"] = None
        if self.solution and self.solution.feasible and self.target:
            shot["target_ft"] = [round(c / FOOT, 1) for c in self.target]
        shots.append(shot)
        self.cfg.save()
        self.shots_version += 1

    def update_shot(self, shot_id: int, **fields) -> dict:
        for s in self.cfg.shots():
            if s["id"] == shot_id:
                for k in ("measured_ft", "landing_height_ft", "use"):
                    if k in fields and fields[k] is not None:
                        s[k] = fields[k]
                if "landing_height_ft" in fields:
                    s["predicted_ft"] = round(calibration.predict_shot(self.aimer, s), 1)
                self.cfg.save()
                self.shots_version += 1
                return {"ok": True}
        return self._fail(f"No shot {shot_id}")

    def delete_shot(self, shot_id: int) -> dict:
        shots = self.cfg.shots()
        shots[:] = [s for s in shots if s["id"] != shot_id]
        self.cfg.save()
        self.shots_version += 1
        return {"ok": True}

    def fit(self, params) -> dict:
        try:
            res = calibration.fit_model(self.aimer, self.cfg.shots(), params)
        except ValueError as e:
            return self._fail(str(e))
        self.fit_result = res
        return {"ok": True, "result": res}

    def apply_fit(self) -> dict:
        if not self.fit_result:
            return self._fail("Run a fit first")
        p = self.fit_result["params"]
        if "efficiency" in p:
            self.cfg.set_override("launcher", "efficiency", p["efficiency"])
        if "drag_coefficient" in p:
            self.cfg.set_override("projectile", "drag_coefficient", p["drag_coefficient"])
        self.rebuild()
        for s in self.cfg.shots():
            s["predicted_ft"] = round(calibration.predict_shot(self.aimer, s), 1)
        self.cfg.save()
        self.shots_version += 1
        self.msg("Fitted model saved: " + ", ".join(f"{k} {v}" for k, v in p.items()))
        self.fit_result = None
        return {"ok": True}

    def axis_test_move(self, axis: str, degrees: float) -> dict:
        pose = self._pose()
        if pose is None or not self._zeroed():
            return self._fail("Set zero for both axes first")
        yaw, pitch = pose
        if axis == "turret":
            yaw += degrees
        else:
            pitch += degrees
        with self.lock:
            self.mode, self.phase = "manual", ""
        ok, reason = self._move(yaw, pitch)
        if not ok:
            return self._fail(f"Move refused: {self._explain(reason)}")
        self.axis_test = {"axis": axis, "commanded": degrees}
        self.msg(f"Moving the {axis} {degrees:+g} deg. Measure the real movement and enter it.")
        return {"ok": True}

    def axis_apply(self, axis: str, measured: float) -> dict:
        if not self.axis_test or self.axis_test["axis"] != axis:
            return self._fail("Do a test move on this axis first")
        old = self.cfg[axis]["gear_ratio"]
        try:
            new = calibration.corrected_gear_ratio(old, self.axis_test["commanded"], measured)
        except ValueError as e:
            return self._fail(str(e))
        self.cfg.set_override(axis, "gear_ratio", round(new, 5))
        self.rebuild()
        pre = "Y" if axis == "turret" else "P"
        errs = self.link.push_settings([(f"{pre}.SPD", self.cfg.steps_per_deg(axis))])
        if errs:
            return self._fail("Saved, but the RedBoard refused it: " + "; ".join(errs))
        self.axis_test = None
        self.msg(f"{axis} gear ratio {old:g} -> {new:.4f} ({self.cfg.steps_per_deg(axis):.2f} steps/deg).")
        return {"ok": True, "gear_ratio": new}

    def boresight(self, nx: float, ny: float, forward_ft: float, right_ft: float, height_ft: float) -> dict:
        pose = self._pose()
        if pose is None:
            return self._fail("No status from the RedBoard")
        marker = (forward_ft * FOOT, right_ft * FOOT, height_ft * FOOT)
        try:
            cy, cp = calibration.boresight(self.mount, pose[0], pose[1], marker, nx, ny)
        except ValueError as e:
            return self._fail(str(e))
        self.cfg.set_override("geometry", "camera_yaw_offset_deg", round(cy, 3))
        self.cfg.set_override("geometry", "camera_pitch_offset_deg", round(cp, 3))
        self.rebuild()
        self.msg(f"Camera offsets set: yaw {cy:+.2f} deg, pitch {cp:+.2f} deg.")
        return {"ok": True}

    def sim_fire(self) -> dict:
        if not self.sim_fw:
            return self._fail("Only available in --sim mode")
        if not self.sim_fw.fire():
            return self._fail("Simulated chamber is under 8 psi")
        return {"ok": True}

    # ------------------------------------------------------------ UI state

    def state(self) -> dict:
        st = self.link.status if self.link.connected else {}
        det = self.camera.latest
        pose = self._pose()
        with self.lock:
            target_px = None
            if self.target is not None and pose:
                target_px = self.mount.project(pose[0], pose[1], self.target)
            det_px = None
            if det.valid:
                det_px = (math.tan(math.radians(det.tx)) / math.tan(math.radians(self.mount.hfov / 2)),
                          -math.tan(math.radians(det.ty)) / math.tan(math.radians(self.mount.vfov / 2)))
            return {
                "link": {"connected": self.link.connected, "fresh": self.link.fresh(), "port": self.link.port_name,
                         "fw": self.link.fw_version, "error": self.link.last_error},
                "fw": st,
                "camera": {"online": self.camera.online, "error": self.camera.last_error,
                           "detection": det.to_json(), "det_px": det_px,
                           "stream": bool(self.camera.stream_url), "stream_url": self.camera.stream_url,
                           "hfov": self.mount.hfov, "vfov": self.mount.vfov},
                "mode": self.mode,
                "phase": self.phase,
                "armed": self.armed,
                "track_method": self.track_method,
                "target_ft": [round(c / FOOT, 1) for c in self.target] if self.target else None,
                "target_source": self.target_source,
                "target_px": target_px,
                "solution": self.solution.to_json() if self.solution else None,
                "strategy": dict(self.strategy),
                "messages": list(self.messages)[-8:],
                "limits": {"turret": self.cfg.limits("turret"), "pitch": self.cfg.limits("pitch")},
                "max_psi": self.cfg.max_psi(),
                "min_psi": self.cfg["pressure"]["min_psi"],
                "hysteresis": self.cfg["pressure"]["hold_hysteresis_psi"],
                "sim": self.sim_fw is not None,
                "axis_test": self.axis_test,
                "fit_result": self.fit_result,
                "shots_version": self.shots_version,
                "model": {
                    "turret_gear_ratio": self.cfg["turret"]["gear_ratio"],
                    "pitch_gear_ratio": self.cfg["pitch"]["gear_ratio"],
                    "turret_steps_per_deg": round(self.cfg.steps_per_deg("turret"), 3),
                    "pitch_steps_per_deg": round(self.cfg.steps_per_deg("pitch"), 3),
                    "efficiency": self.cfg["launcher"]["efficiency"],
                    "drag_coefficient": self.cfg["projectile"]["drag_coefficient"],
                    "camera_yaw_offset_deg": self.cfg["geometry"]["camera_yaw_offset_deg"],
                    "camera_pitch_offset_deg": self.cfg["geometry"]["camera_pitch_offset_deg"],
                    "max_speed_fps": round(self.aimer.velocity(self.cfg.max_psi()) / FOOT, 1),
                },
            }
