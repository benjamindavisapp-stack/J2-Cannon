"""Link + controller tests against the Python simulator and, if it has been
built (`make -C firmware/test fw_host`), against the real firmware code
running behind a virtual serial port."""

import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from j2cannon.config import Config  # noqa: E402
from j2cannon.controller import Controller  # noqa: E402
from j2cannon.limelight import Detection  # noqa: E402
from j2cannon.link import FirmwareLink, SerialTransport, parse_line  # noqa: E402
from j2cannon.simfw import SimFirmware  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
CONFIG = os.path.join(ROOT, "pi", "config.toml")
FW_HOST = os.path.join(ROOT, "firmware", "test", "fw_host")


class NoCamera:
    online = False
    last_error = "none"
    stream_url = None
    latest = Detection()


def wait_for(pred, timeout=10.0, step=0.05):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(step)
    return False


class ParseTest(unittest.TestCase):
    def test_lines(self):
        kind, d = parse_line("S ya=1.50 pa=-2.00 md=PH mv=1 z=3 psi=40.1 pst=HOLD")
        self.assertEqual(kind, "status")
        self.assertEqual((d["ya"], d["pa"], d["md"], d["z"], d["pst"]), (1.5, -2.0, "PH", 3, "HOLD"))
        kind, d = parse_line("E FAULT 1 OVERPRESSURE")
        self.assertEqual((kind, d["type"], d["args"]), ("event", "FAULT", ["1", "OVERPRESSURE"]))
        kind, d = parse_line("E SHOT pre=50.2 post=3.1 ya=1.00 pa=20.00")
        self.assertEqual((d["type"], d["pre"]), ("SHOT", 50.2))
        self.assertEqual(parse_line("ERR MOVE NOZERO"), ("err", {"cmd": "MOVE", "reason": "NOZERO"}))
        self.assertEqual(parse_line("OK PRESS")[0], "ok")


class SystemMixin:
    """Shared end-to-end flow; subclasses provide the transport."""

    def make(self, transport):
        self.tmp = tempfile.TemporaryDirectory()
        cfg = Config(CONFIG, data_dir=self.tmp.name)
        cfg.data["aim"]["auto_pressurize"] = True
        self.link = FirmwareLink(transport)
        self.ctl = Controller(cfg, self.link, NoCamera(), getattr(self, "sim_fw", None))
        self.link.on_event = self.ctl.on_event
        self.link.on_connect = self.ctl.on_connect
        self.link.start()
        self.ctl.start()
        self.assertTrue(wait_for(lambda: self.link.fresh(), 10), "no status from firmware")
        self.assertTrue(wait_for(lambda: any("connected" in m["text"] for m in self.ctl.messages), 10),
                        [m["text"] for m in self.ctl.messages])

    def tearDown(self):
        self.ctl.stop()
        self.link.stop()
        self.tmp.cleanup()

    def fire(self):
        raise NotImplementedError

    def test_aim_pressurize_fire(self):
        ctl = self.ctl
        res = ctl.aim_at_coords(60, 5, 0)
        self.assertFalse(res["ok"])  # not zeroed yet
        self.assertTrue(ctl.simple("ZERO B")["ok"])
        self.assertTrue(wait_for(lambda: self.link.status.get("z") == 3, 3))
        ctl.set_strategy(strategy="fixed_pressure", fixed_psi=20.0, arc="low")
        res = ctl.aim_at_coords(60, 5, 0)
        self.assertTrue(res["ok"], res)
        self.assertTrue(wait_for(lambda: ctl.phase == "ready", 25), f"phase={ctl.phase} fw={self.link.status}")
        st = self.link.status
        self.assertAlmostEqual(st["ya"], ctl.solution.yaw_deg, delta=0.2)
        self.assertAlmostEqual(st["pa"], ctl.solution.pitch_deg, delta=0.2)
        self.assertGreaterEqual(st["psi"], 19.0)
        self.fire()
        self.assertTrue(wait_for(lambda: len(ctl.cfg.shots()) == 1, 5))
        shot = ctl.cfg.shots()[0]
        self.assertAlmostEqual(shot["psi"], st["psi"], delta=1.5)
        self.assertEqual(ctl.phase, "fired")
        time.sleep(1.0)
        self.assertEqual(self.link.status["pst"], "IDLE")  # no refill after a shot
        self.assertTrue(ctl.rearm()["ok"])
        self.assertTrue(wait_for(lambda: self.link.status.get("pst") in ("FILL", "HOLD"), 5))
        ctl.stop_all()
        self.assertTrue(wait_for(lambda: self.link.status.get("pst") == "IDLE", 3))

    def test_limits_enforced_by_firmware(self):
        self.assertTrue(self.ctl.simple("ZERO B")["ok"])
        ok, reason = self.link.command("MOVE 120 0")
        self.assertEqual((ok, reason), (False, "LIMIT"))
        ok, reason = self.link.command("PRESS 90")
        self.assertEqual((ok, reason), (False, "RANGE"))


class SimSystemTest(SystemMixin, unittest.TestCase):
    def setUp(self):
        self.sim_fw = SimFirmware(fill_rate=10.0)
        self.make(self.sim_fw)

    def fire(self):
        self.assertTrue(self.sim_fw.fire())


@unittest.skipUnless(os.path.exists(FW_HOST), "build firmware/test/fw_host to run against the real firmware")
class FirmwareSystemTest(SystemMixin, unittest.TestCase):
    def setUp(self):
        self.proc = subprocess.Popen([FW_HOST], stdout=subprocess.PIPE, text=True)
        port = self.proc.stdout.readline().strip()
        self.make(SerialTransport(port, 115200))

    def tearDown(self):
        super().tearDown()
        self.proc.kill()
        self.proc.wait()
        self.proc.stdout.close()

    def fire(self):
        os.kill(self.proc.pid, signal.SIGUSR1)

    def test_settings_pushed(self):
        ok, _ = self.link.command("GET")
        self.assertTrue(ok)

    def test_link_loss_stops_compressor(self):
        self.assertTrue(self.ctl.simple("ZERO B")["ok"])
        self.assertTrue(self.ctl.pressurize(40)["ok"])
        self.assertTrue(wait_for(lambda: self.link.status.get("c") == 1, 6))
        # Silence the Pi side: stop heartbeats and commands.
        events = []
        self.link.on_event = lambda e: events.append(e["type"])
        real_send = self.link.transport.write
        self.link.transport.write = lambda line: None
        self.assertTrue(wait_for(lambda: "LINKLOST" in events, 4), events)
        self.assertTrue(wait_for(lambda: self.link.status.get("c") == 0, 2))
        self.link.transport.write = real_send


if __name__ == "__main__":
    unittest.main()
