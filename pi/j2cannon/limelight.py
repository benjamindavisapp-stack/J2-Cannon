"""Limelight 3A client: polls the JSON results and proxies the video stream.

The 3A shows up over USB as a network adapter; the camera is 172.29.0.1.
Port 5800 is the MJPEG stream, 5801 its web UI and 5807 the JSON API.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
import urllib.request
from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

from .geometry import Mount, Vec, matvec, sub, transpose

log = logging.getLogger(__name__)


@dataclass
class Detection:
    valid: bool = False
    tx: float = 0.0            # deg, right +, from the image centre
    ty: float = 0.0            # deg, up +
    ta: float = 0.0            # target area, % of image
    tag_id: Optional[int] = None
    tag_cam: Optional[Tuple[float, float, float]] = None  # tag position, camera space (x right, y down, z out), m
    latency_s: float = 0.0
    time: float = 0.0          # when the frame was captured (local clock)

    def to_json(self) -> dict:
        d = {"valid": self.valid, "age_s": round(time.time() - self.time, 2) if self.time else None}
        if self.valid:
            d.update(tx=round(self.tx, 2), ty=round(self.ty, 2), ta=round(self.ta, 2), tag_id=self.tag_id,
                     tag_range_ft=round(math.sqrt(sum(c * c for c in self.tag_cam)) / 0.3048, 1)
                     if self.tag_cam else None)
        return d


def _num(d: dict, *keys, default=0.0) -> float:
    for k in keys:
        if k in d and d[k] is not None:
            try:
                return float(d[k])
            except (TypeError, ValueError):
                pass
    return default


def parse_results(data: dict, received: float, tag_id: int = -1) -> Detection:
    """Parse a /results JSON document (current and older wrapped formats)."""
    if "Results" in data and isinstance(data["Results"], dict):
        data = data["Results"]
    det = Detection(time=received)
    latency_ms = _num(data, "tl") + _num(data, "cl")
    det.latency_s = latency_ms / 1000.0
    det.time = received - det.latency_s
    fids = data.get("Fiducial") or []
    fid = None
    for f in fids:
        if tag_id < 0 or int(_num(f, "fID", default=-1)) == tag_id:
            fid = f
            break
    if fid is not None:
        det.valid = True
        det.tx = _num(fid, "tx_nocross", "txnc", "tx")
        det.ty = _num(fid, "ty_nocross", "tync", "ty")
        det.ta = _num(fid, "ta")
        det.tag_id = int(_num(fid, "fID", default=-1))
        pose = fid.get("t6t_cs")
        if isinstance(pose, (list, tuple)) and len(pose) >= 3 and any(pose[:3]):
            det.tag_cam = (float(pose[0]), float(pose[1]), float(pose[2]))
        return det
    if tag_id >= 0 and fids:
        return det
    v = data.get("v")
    if v in (1, True, "1") or (v is None and ("tx" in data or "txnc" in data) and _num(data, "ta") > 0):
        det.valid = True
        det.tx = _num(data, "txnc", "tx")
        det.ty = _num(data, "tync", "ty")
        det.ta = _num(data, "ta")
    return det


class Limelight:
    def __init__(self, results_url: str, stream_url: str, poll_hz: float = 15, tag_id: int = -1):
        self.results_url = results_url
        self.stream_url = stream_url
        self.period = 1.0 / max(poll_hz, 1)
        self.tag_id = tag_id
        self.latest = Detection()
        self.online = False
        self.last_error = ""
        self._stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.is_set():
            t0 = time.time()
            try:
                with urllib.request.urlopen(self.results_url, timeout=0.5) as r:
                    data = json.loads(r.read().decode("utf-8", "replace"))
                self.latest = parse_results(data, time.time(), self.tag_id)
                self.online = True
                self.last_error = ""
            except Exception as e:
                if self.online or not self.last_error:
                    log.warning("Limelight: %s", e)
                self.online = False
                self.last_error = str(e)
                self.latest = Detection()
                self._stop.wait(1.0)
            self._stop.wait(max(0.0, self.period - (time.time() - t0)))


class SimLimelight:
    """Fake detections of a world point, computed from the current turret pose."""

    def __init__(self, mount: Mount, pose_fn: Callable[[], Optional[Tuple[float, float]]], target_m: Sequence[float],
                 motion: str = "none"):
        self.mount = mount
        self.pose_fn = pose_fn
        self.base = tuple(target_m)
        self.motion = motion
        self.online = True
        self.last_error = ""
        self.stream_url = None
        self.tag_id = -1
        self.t0 = time.time()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def target(self) -> Vec:
        if self.motion != "circle":
            return self.base
        a = (time.time() - self.t0) * 2 * math.pi / 40.0  # one lap every 40 s, radius 5 m
        return (self.base[0] + 5 * math.cos(a), self.base[1] + 5 * math.sin(a), self.base[2])

    @property
    def latest(self) -> Detection:
        pose = self.pose_fn()
        if pose is None:
            return Detection()
        p = self.target()
        ang = self.mount.camera_angles(pose[0], pose[1], p)
        now = time.time()
        if ang is None or abs(ang[0]) > self.mount.hfov / 2 or abs(ang[1]) > self.mount.vfov / 2:
            return Detection(time=now)
        # Camera space: x right, y down, z out of the lens.
        local = matvec(transpose(self.mount.camera_rotation(*pose)), sub(p, self.mount.camera_position(*pose)))
        return Detection(True, ang[0], ang[1], 1.0, 0, (local[1], -local[2], local[0]), 0.0, now)
