"""Unit tests for ballistics, geometry, aiming and calibration maths."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from j2cannon import ballistics as bal  # noqa: E402
from j2cannon import calibration  # noqa: E402
from j2cannon.aiming import Aimer  # noqa: E402
from j2cannon.config import Config  # noqa: E402
from j2cannon.geometry import (Mount, camera_frame_to_local, firing_yaw, intersect_height,  # noqa: E402
                               plane_distance, point_at_distance)
from j2cannon.limelight import parse_results  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config.toml")
FT = bal.FOOT


def make_aimer(over=None) -> Aimer:
    cfg = Config(CONFIG, data_dir="/nonexistent-j2-test")
    for (sec, key), val in (over or {}).items():
        cfg.data[sec][key] = val
    return Aimer(cfg.mount(), cfg.launcher(), cfg.projectile(), 1.2, cfg.limits("turret"), cfg.limits("pitch"),
                 cfg["pressure"]["min_psi"], cfg["ballistics"]["lob_angle_deg"], 400 * FT)


class BallisticsTest(unittest.TestCase):
    def test_vacuum_min_speed_is_45_degrees(self):
        r = bal.solve_min_speed(0.0, lambda th: (0.0, 0.0), 40.0, 0.0, math.radians(-45), math.radians(90), 100)
        self.assertAlmostEqual(math.degrees(r[0]), 45.0, delta=0.2)
        self.assertAlmostEqual(r[1], math.sqrt(bal.G * 40.0), delta=0.01)

    def test_vacuum_fixed_speed_matches_formula(self):
        v, x = 25.0, 40.0
        expect_low = 0.5 * math.asin(bal.G * x / v ** 2)
        expect_high = math.pi / 2 - expect_low
        lo = bal.solve_angle(v, 0.0, lambda th: (0.0, 0.0), x, 0.0, math.radians(-45), math.radians(90), "low")
        hi = bal.solve_angle(v, 0.0, lambda th: (0.0, 0.0), x, 0.0, math.radians(-45), math.radians(90), "high")
        self.assertAlmostEqual(lo, expect_low, delta=1e-3)
        self.assertAlmostEqual(hi, expect_high, delta=1e-3)

    def test_unreachable_returns_none(self):
        self.assertIsNone(bal.solve_angle(10.0, 0.0, lambda th: (0.0, 0.0), 100.0, 0.0, -0.7, 1.5, "low"))

    def test_drag_shortens_range(self):
        no_drag, *_ = bal.fly_to_height(30, math.radians(30), 0.0, 0, 0, 0)
        drag, *_ = bal.fly_to_height(30, math.radians(30), 0.005, 0, 0, 0)
        self.assertAlmostEqual(no_drag, 30 ** 2 * math.sin(math.radians(60)) / bal.G, delta=0.05)
        self.assertLess(drag, no_drag * 0.9)

    def test_pressure_velocity_round_trip(self):
        aimer = make_aimer()
        for psi in (10, 35, 70):
            v = aimer.velocity(psi)
            self.assertAlmostEqual(aimer.pressure_for(v), psi, delta=0.01)
        self.assertIsNone(aimer.pressure_for(aimer.velocity(75) * 1.01))


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.m = Mount(1.2, (0.8, 0.05, 0.1), (0.25, 0.1, 0.1), 1.5, -2.0, 54.5, 42.0)

    def test_project_and_ray_round_trip(self):
        p = (20.0, 4.0, 0.5)
        for yaw, pitch in ((0, 0), (12, 5), (-30, -10)):
            nx, ny = self.m.project(yaw, pitch, p)
            origin, d = self.m.world_ray(yaw, pitch, self.m.pixel_dir(nx, ny))
            hit = intersect_height(origin, d, 0.5, 1000)
            for a, b in zip(hit, p):
                self.assertAlmostEqual(a, b, places=6)

    def test_look_angles_centre_the_point(self):
        p = (15.0, -6.0, 3.0)
        yaw, pitch = self.m.look_angles(p)
        tx, ty = self.m.camera_angles(yaw, pitch, p)
        self.assertAlmostEqual(tx, 0, places=3)
        self.assertAlmostEqual(ty, 0, places=3)

    def test_point_at_distance(self):
        origin, d = (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)
        self.assertEqual(point_at_distance(origin, d, 10.0), (10.0, 0.0, 1.0))

    def test_firing_yaw_with_offset(self):
        target = (10.0, 2.0, 0.0)
        s = 0.3
        yaw = math.radians(firing_yaw(target, s))
        # Barrel line: s to the right of the axis, pointing along yaw. Target must lie on it.
        right = (-math.sin(yaw), math.cos(yaw))
        self.assertAlmostEqual(target[0] * right[0] + target[1] * right[1], s, places=9)
        self.assertAlmostEqual(plane_distance(target, s), math.sqrt(104 - 0.09), places=9)

    def test_camera_frame(self):
        self.assertEqual(camera_frame_to_local((1.0, 2.0, 3.0)), (3.0, 1.0, -2.0))


class AimingTest(unittest.TestCase):
    def assert_hits(self, aimer, sol, target, tol=0.02):
        th = math.radians(sol.pitch_deg)
        x0, y0 = aimer.launch_point(th)
        x_t = plane_distance(target, aimer.mount.muzzle[1])
        shot = bal.fly_to_x(sol.v0, th, aimer.k, x0, y0, x_t, target[2])
        self.assertTrue(shot.reached)
        self.assertAlmostEqual(shot.y_at_target, target[2], delta=tol)
        self.assertAlmostEqual(aimer.velocity(sol.psi), sol.v0, delta=0.01)

    def test_solutions_hit(self):
        aimer = make_aimer({("geometry", "muzzle_right_in"): 3.0, ("geometry", "muzzle_up_in"): -2.0})
        target = (40 * FT * 3, 15 * FT, 2 * FT)
        for strategy, arc in (("min_pressure", "low"), ("min_pressure", "high"),
                              ("fixed_pressure", "low"), ("fixed_pressure", "high")):
            sol = aimer.solve(target, strategy, arc, fixed_psi=50)
            self.assertTrue(sol.feasible, f"{strategy}/{arc}: {sol.reason}")
            self.assert_hits(aimer, sol, target)
        low = aimer.solve(target, "fixed_pressure", "low", fixed_psi=50)
        high = aimer.solve(target, "fixed_pressure", "high", fixed_psi=50)
        self.assertGreater(high.pitch_deg, low.pitch_deg + 10)

    def test_min_pressure_is_minimal(self):
        aimer = make_aimer()
        target = (150 * FT, 0.0, 0.0)
        sol = aimer.solve(target, "min_pressure", "low")
        self.assertIsNone(bal.solve_angle(aimer.velocity(sol.psi - 0.5), aimer.k, aimer.launch_point,
                                          plane_distance(target, 0), 0.0, -0.7, 1.5, "low"))

    def test_reuse_pressure(self):
        aimer = make_aimer()
        target = (120 * FT, 0.0, 0.0)
        fresh = aimer.solve(target, "min_pressure", "low")
        reused = aimer.solve(target, "min_pressure", "low", have_psi=fresh.psi + 15, reuse=True)
        self.assertTrue(reused.reused_pressure)
        self.assertAlmostEqual(reused.psi, fresh.psi + 15)
        self.assertLess(reused.pitch_deg, fresh.pitch_deg)
        self.assert_hits(aimer, reused, target)

    def test_limits_and_range(self):
        aimer = make_aimer()
        self.assertIn("yaw", aimer.solve((-10.0, 30.0, 0.0)).reason)          # behind-right: past +90
        self.assertIn("out of range", aimer.solve((300 * FT * 1.3, 0.0, 0.0), "fixed_pressure", fixed_psi=10).reason
                      .replace("cannot reach", "out of range"))
        self.assertIn("max_range", aimer.solve((500 * FT, 0.0, 0.0)).reason)

    def test_trim(self):
        aimer = make_aimer()
        target = (100 * FT, 0.0, 0.0)
        sol = aimer.solve(target, "fixed_pressure", "low", fixed_psi=30)
        trimmed = aimer.trim(sol, 31.0, sol.pitch_deg)
        self.assertLess(trimmed.pitch_deg, sol.pitch_deg)
        self.assert_hits(aimer, trimmed, target)


class CalibrationTest(unittest.TestCase):
    def test_fit_recovers_parameters(self):
        aimer = make_aimer()
        truth = calibration._with_params(aimer, {"efficiency": 0.22, "drag_coefficient": 0.75})
        shots = []
        for i, (pitch, psi) in enumerate(((5, 20), (15, 30), (35, 25), (60, 40), (10, 50))):
            s = {"id": i + 1, "yaw_deg": 0.0, "pitch_deg": pitch, "psi": psi, "use": True}
            s["measured_ft"] = calibration.predict_shot(truth, s)
            shots.append(s)
        res = calibration.fit_model(aimer, shots, ["efficiency", "drag_coefficient"])
        self.assertAlmostEqual(res["params"]["efficiency"], 0.22, delta=0.005)
        self.assertAlmostEqual(res["params"]["drag_coefficient"], 0.75, delta=0.02)
        self.assertLess(res["rms_ft"], 0.2)

    def test_fit_needs_shots(self):
        with self.assertRaises(ValueError):
            calibration.fit_model(make_aimer(), [], ["efficiency"])

    def test_gear_ratio(self):
        self.assertAlmostEqual(calibration.corrected_gear_ratio(12.0, 30.0, 27.0), 12.0 * 30 / 27)
        with self.assertRaises(ValueError):
            calibration.corrected_gear_ratio(12.0, 30.0, -29.0)

    def test_boresight_recovers_offsets(self):
        true = Mount(1.2, (0.8, 0, 0), (0.25, 0.05, 0.1), 2.0, -1.5, 54.5, 42.0)
        guess = Mount(1.2, (0.8, 0, 0), (0.25, 0.05, 0.1), 0.0, 0.0, 54.5, 42.0)
        marker = (15.0, 2.0, 0.3)
        nx, ny = true.project(5.0, -3.0, marker)
        cy, cp = calibration.boresight(guess, 5.0, -3.0, marker, nx, ny)
        self.assertAlmostEqual(cy, 2.0, delta=1e-3)
        self.assertAlmostEqual(cp, -1.5, delta=1e-3)


class LimelightParseTest(unittest.TestCase):
    def test_flat_results(self):
        d = parse_results({"v": 1, "tx": 3.5, "ty": -2.0, "txnc": 3.6, "tync": -2.1, "ta": 0.8, "tl": 20, "cl": 10},
                          100.0)
        self.assertTrue(d.valid)
        self.assertEqual((d.tx, d.ty), (3.6, -2.1))
        self.assertAlmostEqual(d.time, 99.97)

    def test_wrapped_and_fiducial(self):
        doc = {"Results": {"v": 1, "tx": 1, "ty": 1, "Fiducial": [
            {"fID": 3, "tx": 2.0, "ty": 1.0, "tx_nocross": 2.5, "ty_nocross": 1.5, "ta": 0.3,
             "t6t_cs": [0.5, -0.2, 6.0, 0, 0, 0]}]}}
        d = parse_results(doc, 50.0)
        self.assertEqual(d.tag_id, 3)
        self.assertEqual((d.tx, d.ty), (2.5, 1.5))
        self.assertEqual(d.tag_cam, (0.5, -0.2, 6.0))
        self.assertFalse(parse_results(doc, 50.0, tag_id=7).valid)

    def test_no_target(self):
        self.assertFalse(parse_results({"v": 0, "tx": 0, "ty": 0}, 1.0).valid)


if __name__ == "__main__":
    unittest.main()
