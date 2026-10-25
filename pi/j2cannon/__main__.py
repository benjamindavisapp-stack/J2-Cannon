"""Run the J2 Cannon controller: python3 -m j2cannon [--sim] [--config config.toml]"""

from __future__ import annotations

import argparse
import logging
import os

from .config import Config
from .controller import Controller
from .ballistics import FOOT
from .link import FirmwareLink, SerialTransport
from .limelight import Limelight, SimLimelight
from .web import create_app

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build(cfg: Config, sim: bool):
    """Wire up link, camera and controller. Returns (controller, link, camera)."""
    sim_fw = None
    if sim:
        from .simfw import SimFirmware
        from . import calibration

        def land(yaw, pitch, psi):
            # "True" physics differs from the config so calibration has something to find.
            truth = calibration._with_params(ctl.aimer, {"efficiency": cfg["sim"]["true_efficiency"],
                                                          "drag_coefficient": cfg["sim"]["true_drag_coefficient"]})
            return truth.predict_landing(yaw, pitch, psi)[0] / FOOT

        sim_fw = SimFirmware(cfg["sim"]["fill_rate_psi_s"], land)
        transport = sim_fw
    else:
        transport = SerialTransport(cfg["serial"]["port"], cfg["serial"]["baud"])

    link = FirmwareLink(transport)
    if sim:
        def pose():
            st = link.status
            return (float(st["ya"]), float(st["pa"])) if st else None
        camera = SimLimelight(cfg.mount(), pose, [c * FOOT for c in cfg["sim"]["target_ft"]],
                              cfg["sim"]["target_motion"])
    else:
        ll = cfg["limelight"]
        camera = Limelight(ll["results_url"], ll["stream_url"], ll["poll_hz"], ll["tag_id"])
    ctl = Controller(cfg, link, camera, sim_fw)
    link.on_event = ctl.on_event
    link.on_connect = ctl.on_connect
    return ctl, link, camera


def main() -> None:
    ap = argparse.ArgumentParser(description="J2 Cannon controller")
    ap.add_argument("--config", default=os.path.join(HERE, "config.toml"))
    ap.add_argument("--sim", action="store_true", help="no hardware: simulated RedBoard and Limelight")
    ap.add_argument("--port", help="serial port (overrides config)")
    ap.add_argument("--http-port", type=int, help="web UI port (overrides config)")
    ap.add_argument("--host", help="web UI bind address (overrides config)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    data_dir = os.path.join(os.path.dirname(os.path.abspath(args.config)), "data")
    if args.sim:
        data_dir = os.path.join(data_dir, "sim")  # keep practice shots out of the real log
    cfg = Config(args.config, data_dir)
    if args.port:
        cfg.data["serial"]["port"] = args.port
    ctl, link, camera = build(cfg, args.sim)
    link.start()
    camera.start()
    ctl.start()

    host = args.host or cfg["web"]["host"]
    port = args.http_port or cfg["web"]["port"]
    logging.info("J2 Cannon UI on http://%s:%d/%s", "localhost" if host == "0.0.0.0" else host, port,
                 "  (SIMULATION)" if args.sim else "")
    app = create_app(ctl)
    app.run(host=host, port=port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
