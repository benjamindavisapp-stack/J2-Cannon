"""Flask web UI and JSON API."""

from __future__ import annotations

import logging
import os
import urllib.request

from flask import Flask, Response, jsonify, request, send_from_directory

from .controller import Controller

log = logging.getLogger(__name__)
STATIC = os.path.join(os.path.dirname(__file__), "static")


def create_app(ctl: Controller) -> Flask:
    app = Flask(__name__, static_folder=None)

    def body() -> dict:
        return request.get_json(silent=True) or {}

    def num(d: dict, key: str, default=None):
        v = d.get(key, default)
        if v is None or v == "":
            return default
        return float(v)

    def out(res: dict):
        return jsonify(res), (200 if res.get("ok", True) else 400)

    @app.errorhandler(ValueError)
    def bad_value(e):
        return jsonify({"ok": False, "error": f"bad input: {e}"}), 400

    @app.get("/")
    def index():
        return send_from_directory(STATIC, "index.html")

    @app.get("/static/<path:name>")
    def static_file(name):
        return send_from_directory(STATIC, name)

    @app.get("/api/state")
    def state():
        return jsonify(ctl.state())

    # -------------------------------------------------------- aiming
    @app.post("/api/aim/coords")
    def aim_coords():
        d = body()
        return out(ctl.aim_at_coords(num(d, "forward_ft"), num(d, "right_ft", 0.0), num(d, "height_ft", 0.0)))

    @app.post("/api/aim/click")
    def aim_click():
        d = body()
        return out(ctl.aim_at_click(num(d, "nx"), num(d, "ny"), d.get("method", "ground"),
                                    num(d, "distance_ft"), num(d, "height_ft")))

    @app.post("/api/track/start")
    def track_start():
        return out(ctl.track_start(body().get("method", "ground")))

    @app.post("/api/track/stop")
    def track_stop():
        return out(ctl.track_stop())

    @app.post("/api/look")
    def look():
        return out(ctl.look_at_target())

    @app.post("/api/rearm")
    def rearm():
        return out(ctl.rearm())

    @app.post("/api/strategy")
    def strategy():
        d = body()
        return out(ctl.set_strategy(strategy=d.get("strategy"), fixed_psi=num(d, "fixed_psi"), arc=d.get("arc"),
                                    reuse=d.get("reuse"), auto_pressurize=d.get("auto_pressurize"),
                                    trim=d.get("trim")))

    # -------------------------------------------------------- manual
    @app.post("/api/stop")
    def stop():
        return out(ctl.stop_all())

    @app.post("/api/jog")
    def jog():
        d = body()
        return out(ctl.jog(num(d, "yaw", 0.0), num(d, "pitch", 0.0)))

    @app.post("/api/zero")
    def zero():
        axis = {"turret": "Y", "pitch": "P", "both": "B"}.get(body().get("axis", "both"), "B")
        return out(ctl.simple(f"ZERO {axis}", "Zero set."))

    @app.post("/api/pressure")
    def pressure():
        return out(ctl.pressurize(num(body(), "psi")))

    @app.post("/api/pressure/stop")
    def pressure_stop():
        return out(ctl.simple("PSTOP", "Compressor stopped."))

    @app.post("/api/clear")
    def clear():
        return out(ctl.simple("CLEAR", "Fault cleared."))

    @app.post("/api/enable")
    def enable():
        on = bool(body().get("on", True))
        return out(ctl.simple(f"ENABLE {1 if on else 0}", "Motors enabled." if on else
                              "Motors disabled (zero cleared: re-zero after enabling)."))

    # -------------------------------------------------------- calibration
    @app.get("/api/shots")
    def shots():
        return jsonify({"shots": ctl.cfg.shots(), "version": ctl.shots_version})

    @app.post("/api/shots/<int:shot_id>")
    def shot_update(shot_id):
        d = body()
        use = d.get("use")
        return out(ctl.update_shot(shot_id, measured_ft=num(d, "measured_ft"),
                                   landing_height_ft=num(d, "landing_height_ft"),
                                   use=None if use is None else bool(use)))

    @app.delete("/api/shots/<int:shot_id>")
    def shot_delete(shot_id):
        return out(ctl.delete_shot(shot_id))

    @app.post("/api/fit")
    def fit():
        return out(ctl.fit(body().get("params", ["efficiency"])))

    @app.post("/api/fit/apply")
    def fit_apply():
        return out(ctl.apply_fit())

    @app.post("/api/calibrate/axis/move")
    def axis_move():
        d = body()
        return out(ctl.axis_test_move(d.get("axis", "turret"), num(d, "degrees", 30.0)))

    @app.post("/api/calibrate/axis/apply")
    def axis_apply():
        d = body()
        return out(ctl.axis_apply(d.get("axis", "turret"), num(d, "measured")))

    @app.post("/api/calibrate/boresight")
    def boresight():
        d = body()
        return out(ctl.boresight(num(d, "nx"), num(d, "ny"), num(d, "forward_ft"), num(d, "right_ft", 0.0),
                                 num(d, "height_ft", 0.0)))

    @app.post("/api/sim/fire")
    def sim_fire():
        return out(ctl.sim_fire())

    # -------------------------------------------------------- video
    @app.get("/stream")
    def stream():
        """Relay the Limelight MJPEG stream, so browsers on other machines can see it."""
        url = ctl.camera.stream_url
        if not url:
            return Response("no camera stream", status=404)
        try:
            upstream = urllib.request.urlopen(url, timeout=3)
        except Exception as e:
            return Response(f"camera stream unavailable: {e}", status=503)
        ctype = upstream.headers.get("Content-Type", "multipart/x-mixed-replace")

        def gen():
            try:
                while True:
                    chunk = upstream.read(16384)
                    if not chunk:
                        break
                    yield chunk
            finally:
                upstream.close()

        return Response(gen(), content_type=ctype, headers={"Cache-Control": "no-cache"})

    return app
