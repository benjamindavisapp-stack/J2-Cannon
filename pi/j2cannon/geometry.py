"""Frames, rotations and camera rays.

World frame (metres): origin on the ground directly below the turret axis.
Axes are (forward, right, up), where forward is the barrel direction at
yaw 0. Positive yaw turns right, positive pitch raises the barrel.

The barrel frame turns with yaw then pitch about the pitch axle. The
Limelight is fixed to the barrel with a small position and angle offset.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

Vec = Tuple[float, float, float]
Mat = Tuple[Vec, Vec, Vec]


def add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec, b: Vec) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec, s: float) -> Vec:
    return (a[0] * s, a[1] * s, a[2] * s)


def norm(a: Vec) -> float:
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def unit(a: Vec) -> Vec:
    n = norm(a)
    return (a[0] / n, a[1] / n, a[2] / n)


def matvec(m: Mat, v: Vec) -> Vec:
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


def matmul(a: Mat, b: Mat) -> Mat:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def transpose(m: Mat) -> Mat:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def rot_yaw(yaw: float) -> Mat:
    """Positive yaw turns forward toward right."""
    c, s = math.cos(yaw), math.sin(yaw)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def rot_pitch(pitch: float) -> Mat:
    """Positive pitch turns forward toward up."""
    c, s = math.cos(pitch), math.sin(pitch)
    return ((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))


def barrel_rotation(yaw_deg: float, pitch_deg: float) -> Mat:
    return matmul(rot_yaw(math.radians(yaw_deg)), rot_pitch(math.radians(pitch_deg)))


@dataclass
class Mount:
    """Physical layout, in metres and degrees."""

    pivot_height: float
    muzzle: Vec           # muzzle in the barrel frame, relative to the pitch axle
    camera: Vec           # Limelight lens in the barrel frame
    cam_yaw_off: float    # deg, lens aim relative to the barrel
    cam_pitch_off: float
    hfov: float           # deg
    vfov: float

    @property
    def pivot(self) -> Vec:
        return (0.0, 0.0, self.pivot_height)

    def muzzle_position(self, yaw_deg: float, pitch_deg: float) -> Vec:
        return add(self.pivot, matvec(barrel_rotation(yaw_deg, pitch_deg), self.muzzle))

    def camera_position(self, yaw_deg: float, pitch_deg: float) -> Vec:
        return add(self.pivot, matvec(barrel_rotation(yaw_deg, pitch_deg), self.camera))

    def camera_rotation(self, yaw_deg: float, pitch_deg: float) -> Mat:
        off = matmul(rot_yaw(math.radians(self.cam_yaw_off)), rot_pitch(math.radians(self.cam_pitch_off)))
        return matmul(barrel_rotation(yaw_deg, pitch_deg), off)

    # ------------------------------------------------------------ camera rays

    def pixel_dir(self, nx: float, ny: float) -> Vec:
        """Direction in the camera frame for a normalised image point.
        nx, ny in [-1, 1]; +nx is right, +ny is down (image convention)."""
        return unit((1.0, nx * math.tan(math.radians(self.hfov / 2)), -ny * math.tan(math.radians(self.vfov / 2))))

    @staticmethod
    def angle_dir(tx_deg: float, ty_deg: float) -> Vec:
        """Direction in the camera frame for Limelight tx (right +) / ty (up +)."""
        return unit((1.0, math.tan(math.radians(tx_deg)), math.tan(math.radians(ty_deg))))

    def world_ray(self, yaw_deg: float, pitch_deg: float, cam_dir: Vec):
        return self.camera_position(yaw_deg, pitch_deg), unit(matvec(self.camera_rotation(yaw_deg, pitch_deg), cam_dir))

    def project(self, yaw_deg: float, pitch_deg: float, p: Vec) -> Optional[Tuple[float, float]]:
        """Normalised image point (nx, ny) of world point p, or None if behind the lens."""
        local = matvec(transpose(self.camera_rotation(yaw_deg, pitch_deg)), sub(p, self.camera_position(yaw_deg, pitch_deg)))
        if local[0] <= 1e-6:
            return None
        nx = (local[1] / local[0]) / math.tan(math.radians(self.hfov / 2))
        ny = -(local[2] / local[0]) / math.tan(math.radians(self.vfov / 2))
        return nx, ny

    def camera_angles(self, yaw_deg: float, pitch_deg: float, p: Vec) -> Optional[Tuple[float, float]]:
        """Limelight-style (tx, ty) in degrees of world point p."""
        local = matvec(transpose(self.camera_rotation(yaw_deg, pitch_deg)), sub(p, self.camera_position(yaw_deg, pitch_deg)))
        if local[0] <= 1e-6:
            return None
        return math.degrees(math.atan2(local[1], local[0])), math.degrees(math.atan2(local[2], local[0]))

    def look_angles(self, p: Vec, yaw0: float = 0.0, pitch0: float = 0.0) -> Tuple[float, float]:
        """Yaw/pitch that put world point p in the centre of the camera image."""
        yaw, pitch = yaw0, pitch0
        for _ in range(8):
            ang = self.camera_angles(yaw, pitch, p)
            if ang is None:
                d = sub(p, self.pivot)
                yaw = math.degrees(math.atan2(d[1], d[0]))
                pitch = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
                continue
            tx, ty = ang
            yaw += tx
            pitch += ty
            if abs(tx) < 1e-3 and abs(ty) < 1e-3:
                break
        return yaw, pitch


def intersect_height(origin: Vec, d: Vec, height: float, max_range: float) -> Optional[Vec]:
    """Where a ray crosses the horizontal plane z = height (e.g. the ground)."""
    if abs(d[2]) < 1e-9:
        return None
    t = (height - origin[2]) / d[2]
    if t <= 0:
        return None
    p = add(origin, scale(d, t))
    if math.hypot(p[0], p[1]) > max_range:
        return None
    return p


def point_at_distance(origin: Vec, d: Vec, distance: float) -> Optional[Vec]:
    """Point along a ray whose horizontal distance from the turret axis is `distance`."""
    a = d[0] ** 2 + d[1] ** 2
    if a < 1e-12:
        return None
    b = 2 * (origin[0] * d[0] + origin[1] * d[1])
    c = origin[0] ** 2 + origin[1] ** 2 - distance ** 2
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    roots = sorted(((-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)))
    for t in roots:
        if t > 0:
            return add(origin, scale(d, t))
    return None


def firing_yaw(target: Vec, right_offset: float) -> Optional[float]:
    """Turret yaw (deg) that puts a barrel offset `right_offset` to the right of
    the turret axis in a vertical plane through the target."""
    r = math.hypot(target[0], target[1])
    if r <= abs(right_offset) + 1e-6:
        return None
    return math.degrees(math.atan2(target[1], target[0]) - math.asin(right_offset / r))


def plane_distance(target: Vec, right_offset: float) -> float:
    """Horizontal distance to the target measured inside the firing plane."""
    r = math.hypot(target[0], target[1])
    return math.sqrt(max(r * r - right_offset * right_offset, 0.0))


def camera_frame_to_local(xyz_cam: Sequence[float]) -> Vec:
    """Limelight camera space (x right, y down, z out of the lens) to (forward, right, up)."""
    return (xyz_cam[2], xyz_cam[0], -xyz_cam[1])
