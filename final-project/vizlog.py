"""
Robot-side telemetry for visualizer.py (standard library only, safe on an old Pi).

Each event is one JSON object per line: {"k": kind, "t": seconds since the phase started, ...}.
Events go to:
  - a log file in final-project/logs/ (always, when started), for replay later;
  - UDP, by default to this machine (127.0.0.1:5005), where dashboard.py listens and draws the map
    (http://<robot-ip>:8080). SNOWPLOW_VIZ=<host>:<port> sends elsewhere instead (e.g. a laptop
    running the Tk window), SNOWPLOW_VIZ=none turns UDP off. AUTO_WEB_VIEWER = True would make a
    script start the old stand-alone viewer (`visualizer.py --web --live`) itself; it is off because
    that viewer outlives the script and takes the dashboard's port. SNOWPLOW_WEBVIZ=0 also skips it.
    UDP is fire-and-forget: nothing blocks or fails if no viewer is listening.

Nothing here ever raises into the robot code. If start() was never called, every call is a no-op.
Everything printed while a phase runs is also sent as "log" events, so the viewer shows the console.
"""
import json
import os
import socket
import subprocess
import sys
import time

DEFAULT_UDP = "127.0.0.1:5005"   # the web viewer on this machine
AUTO_WEB_VIEWER = False          # True: start `visualizer.py --web --live` if it isn't running (old viewer)
WEB_PORT = 8080
POSE_HZ = 15.0
US_HZ = 8.0
FLUSH_SEC = 0.5

_active = False
_t0 = 0.0
_file = None
_buf = []
_last_flush = 0.0
_sock = None
_addr = None
_last_pose = -1.0
_last_us = -1.0
_stdout = None
_web_checked = False
path = None


def _viewer_running():
    try:
        with socket.create_connection(("127.0.0.1", WEB_PORT), timeout=0.2):
            return True
    except OSError:
        return False


def ensure_web_viewer():
    """Starts the web viewer once (low priority, survives this program) and prints where to open it."""
    global _web_checked
    if _web_checked or os.environ.get("SNOWPLOW_WEBVIZ", "1") == "0":
        return
    _web_checked = True
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        if not _viewer_running():
            subprocess.Popen([sys.executable, os.path.join(here, "visualizer.py"), "--web", str(WEB_PORT),
                              "--live", "--no-save"],
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True, cwd=here)
            t0 = time.time()
            while time.time() - t0 < 4.0 and not _viewer_running():   # so the first events aren't lost
                time.sleep(0.1)
        sys.path.insert(0, here)
        from visualizer import local_addresses
        urls = [f"http://{ip}:{WEB_PORT}" for ip in local_addresses()] or [f"http://localhost:{WEB_PORT}"]
        print("[VIZ] live view: " + "  or  ".join(urls))
    except Exception as e:
        print(f"[VIZ] web viewer not started: {e}")


class _Tee:
    """Copies printed lines into "log" events."""

    def __init__(self, inner):
        self.inner = inner
        self.pending = ""

    def write(self, s):
        self.inner.write(s)
        try:
            self.pending += s
            while "\n" in self.pending:
                line, self.pending = self.pending.split("\n", 1)
                if line.strip():
                    emit("log", msg=line.rstrip())
        except Exception:
            pass
        return len(s)

    def flush(self):
        self.inner.flush()

    def __getattr__(self, name):
        return getattr(self.inner, name)


def start(phase, log_dir=None, udp=None, **meta):
    """Begins a new log for `phase`. meta (s1, s2, footprint, ...) is sent as the first event."""
    global _active, _t0, _file, _buf, _last_flush, _sock, _addr, _last_pose, _last_us, _stdout, path
    close()
    try:
        _t0 = time.time()
        _buf, _last_flush, _last_pose, _last_us = [], _t0, -1.0, -1.0
        log_dir = log_dir or os.environ.get("SNOWPLOW_VIZ_LOGDIR") or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "logs")
        if log_dir != "none":
            os.makedirs(log_dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            path = os.path.join(log_dir, f"phase{phase}_{stamp}.jsonl")
            _file = open(path, "w")
        udp = udp or os.environ.get("SNOWPLOW_VIZ") or DEFAULT_UDP
        if AUTO_WEB_VIEWER and udp == DEFAULT_UDP:
            ensure_web_viewer()
        if udp and udp != "none":
            host, port = udp.rsplit(":", 1)
            _addr = (host, int(port))
            _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            _sock.setblocking(False)
        _active = True
        if _stdout is None and not isinstance(sys.stdout, _Tee):
            _stdout = sys.stdout
            sys.stdout = _Tee(sys.stdout)
        core = sys.modules.get("snowplow_core")
        if core is not None and "table" not in meta:      # board size for the viewers
            meta["table"] = [core.TILE_X_MIN, core.TILE_X_MAX, core.TILE_Y_MIN, core.TILE_Y_MAX]
        emit("meta", phase=phase, wall=time.strftime("%Y-%m-%d %H:%M:%S"), **meta)
    except Exception as e:  # never break the robot over telemetry
        _active = False
        print(f"[VIZ] telemetry disabled: {e}")


def emit(_kind, **data):
    global _last_flush
    if not _active:
        return
    try:
        now = time.time()
        rec = {"k": _kind, "t": round(now - _t0, 3)}
        rec.update(data)
        line = json.dumps(rec, separators=(",", ":"))
        if _file is not None:
            _buf.append(line)
            if now - _last_flush >= FLUSH_SEC:
                _file.write("\n".join(_buf) + "\n")
                _file.flush()
                _buf.clear()
                _last_flush = now
        if _sock is not None:
            try:
                _sock.sendto(line.encode(), _addr)
            except (BlockingIOError, OSError):
                pass
    except Exception:
        pass


def pose(x, y, theta, force=False):
    """Odometry pose, throttled to POSE_HZ."""
    global _last_pose
    if not _active:
        return
    now = time.time()
    if force or now - _last_pose >= 1.0 / POSE_HZ:
        _last_pose = now
        emit("pose", x=round(x, 2), y=round(y, 2), th=round(theta, 1))


def us(angle_deg, dist_cm, force=False):
    """Ultrasonic reading at angle_deg relative to the robot heading (+ = right), throttled."""
    global _last_us
    if not _active or angle_deg is None:
        return
    now = time.time()
    if force or now - _last_us >= 1.0 / US_HZ:
        _last_us = now
        emit("us", a=round(angle_deg, 1), d=round(dist_cm, 1))


def tiles(seq):
    return [list(t) for t in seq]


def close():
    global _active, _file, _sock, _stdout
    try:
        if _file is not None:
            if _buf:
                _file.write("\n".join(_buf) + "\n")
                _buf.clear()
            _file.close()
    except Exception:
        pass
    _file = None
    if _sock is not None:
        try:
            _sock.close()
        except Exception:
            pass
    _sock = None
    if _stdout is not None:
        sys.stdout = _stdout
        _stdout = None
    _active = False
