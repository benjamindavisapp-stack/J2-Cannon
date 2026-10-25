"""Serial link to the RedBoard (see firmware/PROTOCOL.md)."""

from __future__ import annotations

import collections
import glob
import logging
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

log = logging.getLogger(__name__)

# USB serial chips seen on RedBoards / Uno clones: FTDI, CH340, Arduino, SparkFun.
KNOWN_VIDS = {0x0403, 0x1A86, 0x2341, 0x1B4F, 0x10C4}


def find_port() -> Optional[str]:
    try:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        for p in ports:
            if p.vid in KNOWN_VIDS:
                return p.device
    except Exception:  # pragma: no cover - pyserial missing tools
        pass
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


class SerialTransport:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser = None

    def open(self) -> None:
        import serial
        port = find_port() if self.port == "auto" else self.port
        if not port:
            raise IOError("no RedBoard serial port found")
        self.ser = serial.Serial(port, self.baud, timeout=0.1)  # opening resets the board (DTR)
        self.name = port

    def readline(self) -> Optional[str]:
        if self.ser is None:
            raise IOError("port closed")
        raw = self.ser.readline()
        if not raw:
            return None
        return raw.decode("ascii", "replace").strip()

    def write(self, line: str) -> None:
        self.ser.write((line + "\n").encode("ascii"))

    def close(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


def parse_kv(parts: List[str]) -> Dict[str, object]:
    out: Dict[str, object] = {}
    for p in parts:
        if "=" not in p:
            continue
        k, v = p.split("=", 1)
        try:
            out[k] = int(v)
        except ValueError:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def parse_line(line: str) -> Tuple[str, Dict[str, object]]:
    """Classify one line from the firmware: ("status"|"event"|"ok"|"err"|"cfg"|"other", data)."""
    parts = line.split()
    if not parts:
        return "other", {}
    head = parts[0]
    if head == "S":
        return "status", parse_kv(parts[1:])
    if head == "E" and len(parts) > 1:
        data = parse_kv(parts[2:])
        data["type"] = parts[1]
        data["args"] = [p for p in parts[2:] if "=" not in p]
        return "event", data
    if head in ("OK", "ERR") and len(parts) > 1:
        return head.lower(), {"cmd": parts[1], "reason": " ".join(parts[2:])}
    if head == "CFG":
        return "cfg", parse_kv(parts[2:])
    return "other", {"line": line}


class FirmwareLink:
    """Keeps the serial connection alive, parses status, and runs commands."""

    def __init__(self, transport, on_event: Callable[[dict], None] = None,
                 on_connect: Callable[["FirmwareLink"], None] = None):
        self.transport = transport
        self.on_event = on_event
        self.on_connect = on_connect
        self.status: Dict[str, object] = {}
        self.status_at = 0.0
        self.connected = False
        self.fw_version = None
        self.port_name = None
        self.last_error = ""
        self.history = collections.deque(maxlen=60)  # (time, yaw, pitch) for the last ~6 s
        self._wlock = threading.Lock()
        self._waiters: Dict[str, collections.deque] = collections.defaultdict(collections.deque)
        self._wait_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        for target in (self._reader, self._heartbeat):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self) -> None:
        self._stop.set()
        self.transport.close()

    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                self.transport.open()
                self.port_name = getattr(self.transport, "name", None)
                self.last_error = ""
                self._session()
            except Exception as e:  # unplugged, permission denied, ...
                if self._stop.is_set():
                    break
                if str(e) != self.last_error:
                    log.warning("RedBoard link: %s", e)
                self.last_error = str(e)
            self.connected = False
            self.transport.close()
            self._fail_waiters("DISCONNECTED")
            self._stop.wait(2.0)

    def _session(self) -> None:
        started = time.time()
        greeted = False
        last_rx = time.time()
        while not self._stop.is_set():
            line = self.transport.readline()
            now = time.time()
            if line is None:
                if self.connected and now - last_rx > 2.0:
                    raise IOError("RedBoard stopped responding")
                if not self.connected and now - started > 6.0:
                    raise IOError("no data from RedBoard")
                continue
            last_rx = now
            kind, data = parse_line(line)
            if kind == "event" and data["type"] == "BOOT":
                self.fw_version = data.get("fw")
                greeted = True
                self._connected()
            elif not greeted and kind == "status" and now - started > 3.0:
                greeted = True  # board did not reset on open (DTR disabled)
                self._connected()
            self._dispatch(kind, data, line)

    def _connected(self) -> None:
        self.connected = True
        if self.on_connect:
            # Runs in its own thread: it waits for replies this reader must deliver.
            threading.Thread(target=self.on_connect, args=(self,), daemon=True).start()

    def _dispatch(self, kind: str, data: dict, line: str) -> None:
        if kind == "status":
            self.status = data
            self.status_at = time.time()
            if "ya" in data and "pa" in data:
                self.history.append((self.status_at, float(data["ya"]), float(data["pa"])))
        elif kind == "event":
            if self.on_event:
                try:
                    self.on_event(data)
                except Exception:
                    log.exception("event handler failed for %s", line)
        elif kind in ("ok", "err"):
            with self._wait_lock:
                q = self._waiters.get(data["cmd"])
                waiter = q.popleft() if q else None
            if waiter:
                waiter[1]["ok"] = kind == "ok"
                waiter[1]["reason"] = data["reason"]
                waiter[0].set()
            elif kind == "err":
                log.info("firmware: %s", line)

    def _heartbeat(self) -> None:
        while not self._stop.wait(0.25):
            if self.connected:
                try:
                    self.send("HB")
                except Exception:
                    pass

    def _fail_waiters(self, reason: str) -> None:
        with self._wait_lock:
            waiters = [w for q in self._waiters.values() for w in q]
            self._waiters.clear()
        for ev, res in waiters:
            res["ok"] = False
            res["reason"] = reason
            ev.set()

    # ------------------------------------------------------------ commands

    def send(self, line: str) -> None:
        with self._wlock:
            self.transport.write(line)

    def command(self, line: str, timeout: float = 1.5) -> Tuple[bool, str]:
        """Send a command and wait for its OK/ERR. Returns (ok, reason)."""
        if not self.connected:
            return False, "RedBoard not connected"
        name = line.split()[0]
        ev = threading.Event()
        res = {"ok": False, "reason": "TIMEOUT"}
        with self._wait_lock:
            self._waiters[name].append((ev, res))
        try:
            self.send(line)
        except Exception as e:
            return False, str(e)
        if not ev.wait(timeout):
            with self._wait_lock:
                q = self._waiters.get(name)
                if q and (ev, res) in q:
                    q.remove((ev, res))
            return False, "TIMEOUT"
        return res["ok"], res["reason"]

    def push_settings(self, settings: List[Tuple[str, float]]) -> List[str]:
        errors = []
        for key, val in settings:
            ok, reason = self.command(f"SET {key} {val:.6g}")
            if not ok:
                errors.append(f"{key}={val}: {reason}")
        ok, reason = self.command("SAVE")
        if not ok:
            errors.append(f"SAVE: {reason}")
        return errors

    # ------------------------------------------------------------ state

    def fresh(self) -> bool:
        return self.connected and time.time() - self.status_at < 1.0

    def pose_at(self, t: float) -> Optional[Tuple[float, float]]:
        """Turret pose (yaw, pitch) at time t, interpolated from recent status lines."""
        h = list(self.history)
        if not h:
            return None
        if t >= h[-1][0]:
            return h[-1][1], h[-1][2]
        for (t0, y0, p0), (t1, y1, p1) in zip(reversed(h[:-1]), reversed(h[1:])):
            if t0 <= t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 0
                return y0 + f * (y1 - y0), p0 + f * (p1 - p0)
        return h[0][1], h[0][2]
