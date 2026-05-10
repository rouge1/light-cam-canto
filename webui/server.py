"""Proxy server for the 8-bit live-status webui.

Serves static files from `webui/` and exposes /api/* endpoints that
proxy through host.cam_status.CamStatusClient (which keeps the cookie
auth machinery + the Phase 1-5 CGI clients in one place).

Endpoints (matched by webui/index.html):
  GET  /api/{cam}/grid    → CamStatusClient.get_brightness_grid()  (Phase 1)
  GET  /api/{cam}/ae      → CamStatusClient.get_ae_freeze()        (Phase 2)
  POST /api/{cam}/ae      body `value=0|1`                         (Phase 3)
  GET  /api/{cam}/proc    → CamStatusClient.get_proc_status()      (Phase 4)
  POST /api/{cam}/led     body `lamp=ir850|ir940&value=0|1`        (Phase 5)
  GET  /api/{cam}/status  → CamStatusClient.get()                  (cal-status.cgi)
  POST /api/cal/start     run host.cal_procedure --no-interactive in a thread
  GET  /api/cal/status    → {running, finished, ok, log: [recent lines]}

cam ∈ {cam1, cam2}; resolved to dacam1/dacam2 then to IPs at startup.

Usage:
  conda activate light
  python -m webui.server                 # default port 8765
  python -m webui.server --port 9000
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
from collections import deque
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from host.cam_status import CamStatusClient, resolve_ip  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

WEBUI_DIR = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# Coalescing cache for slow per-cam endpoints. uhttpd on the cams is a
# single-process server (`-t 0`) — every CGI hit forks+execs a shell on
# slow MIPS hardware (~2s per call). When several browser tabs poll the
# same endpoint simultaneously, requests serialize at uhttpd and pile up.
# A short-TTL coalescing cache here means concurrent identical requests
# share a single in-flight fetch, and back-to-back polls within the TTL
# return the cached response instantly without touching the cam.
# Per-key lock prevents thundering-herd refresh from many tabs at once.
# ---------------------------------------------------------------------------

_CACHE_TTL = {              # seconds — tuned per-endpoint freshness needs
    "status":   1.5,        # status changes via 1Hz heartbeat — 1.5s OK
    "ae":       2.0,
    "proc":     5.0,        # process counts rarely change
    "leds":     1.0,        # GPIO state — keep tight for live-LED, but ≥ poll
                            # period (800ms) so back-to-back polls hit cache
                            # rather than serializing through the cam CGI.
    "grid":     0.4,        # 2.5Hz grid polling — cap at ~half period
    "snapshot": 1.0,        # JPEG — 1Hz feels live enough
    "cal-info": 2.0,        # mostly stable saved cal
    "events":   2.0,
    "monocal-status": 1.0,  # webui polls this aggressively during runs
}

_cache: dict = {}           # key → (timestamp, value)
_cache_locks: dict = {}     # key → threading.Lock
_cache_lock = threading.Lock()  # guards the dicts themselves

def _cache_get_or_fetch(key: str, ttl: float, fetch_fn):
    """Return cached value if fresh, else call fetch_fn() under a per-key
    lock so concurrent callers wait for one fetch instead of stampeding."""
    now = time.monotonic()
    cached = _cache.get(key)
    if cached and now - cached[0] < ttl:
        return cached[1]
    with _cache_lock:
        lock = _cache_locks.setdefault(key, threading.Lock())
    with lock:
        cached = _cache.get(key)
        if cached and time.monotonic() - cached[0] < ttl:
            return cached[1]
        value = fetch_fn()
        _cache[key] = (time.monotonic(), value)
        return value


def _cache_invalidate(*keys: str):
    """Drop cached entries so the next read fetches fresh from the cam.
    Call after any POST that mutates state the cam exposes via a cached GET
    (e.g. POST /api/{cam}/ae must evict {cam}:ae + {cam}:status, otherwise the
    webui's read-after-write round-trip sees a stale TTL'd value)."""
    for k in keys:
        _cache.pop(k, None)


# ---------------------------------------------------------------------------
# Per-cam alive probe (TCP connect to port 80, 0.5s budget, 3s cached).
# When a cam is rebooting or unplugged, every per-cam endpoint would otherwise
# fire its own 3s curl in parallel (8s pre-2026-05-09). With Chrome's 6-slot
# per-origin connection limit, that locks the whole page until the cams come
# back. The probe fast-fails downstream curls on a known-down cam so the
# connection slots churn quickly and the LEDs poll keeps the UI responsive.
# ---------------------------------------------------------------------------

_PROBE_TTL = 3.0
_PROBE_CONNECT_S = 0.5
_probe_cache: dict = {}     # ip → (timestamp, alive_bool)
_probe_lock = threading.Lock()


def _probe_alive(ip: str) -> bool:
    now = time.monotonic()
    cached = _probe_cache.get(ip)
    if cached and now - cached[0] < _PROBE_TTL:
        return cached[1]
    with _probe_lock:
        cached = _probe_cache.get(ip)
        if cached and time.monotonic() - cached[0] < _PROBE_TTL:
            return cached[1]
        alive = False
        s = None
        try:
            s = socket.create_connection((ip, 80), timeout=_PROBE_CONNECT_S)
            alive = True
        except (OSError, socket.timeout):
            alive = False
        finally:
            if s is not None:
                try: s.close()
                except OSError: pass
        _probe_cache[ip] = (time.monotonic(), alive)
        return alive


def _cam_get_or_fetch(cam: str, key: str, ttl: float, fetch_fn):
    """Like _cache_get_or_fetch, but short-circuits to None when the cam is
    known unreachable. Falls back to last-good cached value if any."""
    client = CLIENTS.get(cam)
    if client is None or not _probe_alive(client.ip):
        cached = _cache.get(key)
        return cached[1] if cached else None
    return _cache_get_or_fetch(key, ttl, fetch_fn)

CALIBRATION_JSON_PATH = os.path.join(REPO_ROOT, "host", "calibration.json")


def _read_cam2_sees_cam1_pixel() -> tuple[int, int] | None:
    """Resolve cam2's pixel for cam1's TX, used as cam2's --pixel arg in
    monocal so cam2's RX can decode cam1's CAL_ACK + CAL_DONE over light.

    Source of truth is cam2's own /opt/etc/calibration.json, served via
    /x/saved-cal.cgi — every successful cal flow (laptop-driven, monocal
    protocol path, monocal bootstrap, watchdog) writes to it. The laptop's
    host/calibration.json is a fallback for the very first run before any
    on-cam cal exists.

    Without this preference order, the laptop's stale host/calibration.json
    wins after a bootstrap-path monocal — the bootstrap updates the cam
    file but doesn't sync back to the laptop, so subsequent triggers send
    the OLD coords to cam2 and cam2's ROI misses cam1's TX completely
    (decode_fail → re-bootstrap loop). This was the "every monocal goes to
    bootstrap" bug observed 2026-05-09."""
    cam2 = CLIENTS.get("cam2")
    if cam2 is not None:
        live = cam2.get_saved_cal()
        if live is not None:
            pixel = live.get("tx_pixel")
            if isinstance(pixel, list) and len(pixel) == 2:
                try:
                    return int(pixel[0]), int(pixel[1])
                except (TypeError, ValueError):
                    pass
    # Fallback: pre-monocal initial cal from cal_procedure.
    try:
        with open(CALIBRATION_JSON_PATH) as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return None
    section = data.get("cam2_sees_cam1") or {}
    pixel = section.get("tx_pixel")
    if not (isinstance(pixel, list) and len(pixel) == 2):
        return None
    try:
        return int(pixel[0]), int(pixel[1])
    except (TypeError, ValueError):
        return None

CAM_HOSTS = {"cam1": "dacam1", "cam2": "dacam2"}
CLIENTS: dict[str, CamStatusClient] = {}


def init_clients():
    """Resolve aliases to IPs and build one client per cam at startup."""
    for cam, host in CAM_HOSTS.items():
        try:
            ip = resolve_ip(host)
            CLIENTS[cam] = CamStatusClient(ip)
            print(f"  {cam} → {host} → {ip}")
        except RuntimeError as e:
            print(f"  {cam} → {host} → UNRESOLVED ({e})")


# ============================================================
# Calibration sequence orchestration (single-shot, single-thread)
# ============================================================

CAL_STATE = {
    "lock": threading.Lock(),
    "running": False,
    "finished": False,
    "ok": None,
    "log": deque(maxlen=200),
    "started_at": None,
    "ended_at": None,
}


def _cal_log(line: str):
    """Thread-safe append to the live log."""
    with CAL_STATE["lock"]:
        CAL_STATE["log"].append(line)


def _cal_run():
    """Worker: kill cam1 daemon → cal_procedure --no-interactive → restart."""
    CAL_STATE["log"].clear()
    CAL_STATE["started_at"] = time.time()
    CAL_STATE["finished"] = False
    CAL_STATE["ok"] = None
    ok = False
    try:
        _cal_log("▸ killing cam1 daemon-listen (to free LED control)...")
        try:
            subprocess.run(
                ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", "dacam1",
                 "killall -9 irlink 2>/dev/null; "
                 "echo 0 > /run/prudynt/ae_freeze 2>/dev/null"],
                timeout=15, capture_output=True,
            )
        except subprocess.TimeoutExpired:
            # Best-effort kill — cam1 may be loaded; cal_procedure will surface
            # any actual conflict downstream. Don't abort the run for this.
            _cal_log("▸ WARNING: daemon-kill SSH timed out (cam1 busy?) — continuing")
        _cal_log("▸ daemon down · launching host.cal_procedure --no-interactive")

        proc = subprocess.Popen(
            [sys.executable, "-u", "-m", "host.cal_procedure", "--no-interactive"],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        for line in proc.stdout:
            _cal_log(line.rstrip())
        rc = proc.wait(timeout=300)
        _cal_log(f"▸ cal_procedure exit code: {rc}")

        _cal_log("▸ restarting cam1 daemon-listen with fresh coords...")
        from host.aim_assist import restart_cam1_daemon  # lazy import
        try:
            restart_cam1_daemon("dacam1")
            _cal_log("▸ cam1 daemon-listen up")
        except Exception as e:
            _cal_log(f"▸ daemon restart failed: {e}")

        ok = (rc == 0)
        if ok:
            _cal_log("▸ ═══ CAL COMPLETE ═══")
        else:
            _cal_log(f"▸ ═══ CAL FAILED (rc={rc}) ═══")

    except subprocess.TimeoutExpired:
        _cal_log("▸ ERROR: cal_procedure timed out (>300s)")
    except Exception as e:
        _cal_log(f"▸ ERROR: {type(e).__name__}: {e}")
    finally:
        CAL_STATE["ended_at"] = time.time()
        with CAL_STATE["lock"]:
            CAL_STATE["running"] = False
            CAL_STATE["finished"] = True
            CAL_STATE["ok"] = ok


def _cal_snapshot() -> dict:
    """Thread-safe view of the cal state."""
    with CAL_STATE["lock"]:
        return {
            "running": CAL_STATE["running"],
            "finished": CAL_STATE["finished"],
            "ok": CAL_STATE["ok"],
            "log": list(CAL_STATE["log"]),
            "started_at": CAL_STATE["started_at"],
            "ended_at": CAL_STATE["ended_at"],
        }


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEBUI_DIR, **kwargs)

    # Suppress default request logging — too noisy at 4 Hz.
    def log_message(self, format, *args): pass

    # Browsers routinely disconnect mid-response (page reload, tab close,
    # request superseded) which throws BrokenPipeError / ConnectionResetError
    # in the response writer. The exception is harmless but the default
    # ThreadingHTTPServer dumps a 20-line traceback per occurrence, which
    # masks real errors and bloats the log. Override handle_one_request to
    # eat just those.
    def handle_one_request(self):
        try:
            super().handle_one_request()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_json(self, code: int, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, code: int, body: str):
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(self, code: int, content_type: str, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> str:
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0: return ""
        return self.rfile.read(n).decode("utf-8", errors="replace")

    def _client_for(self, cam: str) -> CamStatusClient | None:
        return CLIENTS.get(cam)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if path.startswith("/api/"):
            self._handle_api_get(path, parsed.query)
            return
        # Fall through to static-file serving.
        super().do_GET()

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        if path.startswith("/api/"):
            self._handle_api_post(path)
            return
        self._send_json(405, {"ok": False, "error": "method not allowed"})

    # --- /api routing ---

    def _handle_api_get(self, path: str, query: str = ""):
        # /api/cal/status (special — not per-cam)
        if path == "/api/cal/status":
            self._send_json(200, _cal_snapshot())
            return
        # /api/cal/monocal-status — merge cam2 monocal status + cam1 cal status.
        # cam1's view drops to {} once cam1 is offline-to-IP after deploy;
        # cam2's monocal-status carries the authoritative result either way.
        if path == "/api/cal/monocal-status":
            cam2 = self._client_for("cam2")
            cam1 = self._client_for("cam1")
            cam2_status = _cam_get_or_fetch(
                "cam2", "cam2:monocal-status", _CACHE_TTL["monocal-status"],
                cam2.get_monocal_status) if cam2 else None
            cam1_status = _cam_get_or_fetch(
                "cam1", "cam1:status", _CACHE_TTL["status"],
                cam1.get) if cam1 else None
            self._send_json(200, {
                "cam2": cam2_status or {"state": "transient"},
                "cam1": cam1_status or {},
            })
            return
        # /api/{cam}/{op}
        parts = path.strip("/").split("/")
        if len(parts) != 3:
            self._send_json(404, {"ok": False, "error": "bad path"})
            return
        _, cam, op = parts
        client = self._client_for(cam)
        if client is None:
            self._send_json(503, {"ok": False, "error": f"cam {cam!r} not initialised"})
            return

        if op == "grid":
            r = _cam_get_or_fetch(cam, f"{cam}:grid", _CACHE_TTL["grid"],
                                  client.get_brightness_grid)
            if r is None:
                self._send_json(200, {"ts": None, "blocks": None})
                return
            ts, blocks = r
            self._send_json(200, {"ts": ts, "blocks": blocks})
            return

        if op == "ae":
            v = _cam_get_or_fetch(cam, f"{cam}:ae", _CACHE_TTL["ae"],
                                  client.get_ae_freeze)
            self._send_json(200, {"value": v})
            return

        if op == "proc":
            r = _cam_get_or_fetch(cam, f"{cam}:proc", _CACHE_TTL["proc"],
                                  client.get_proc_status)
            if r is None:
                self._send_json(200, {"irlink": None, "daynightd": None, "prudynt": None})
                return
            self._send_json(200, r)
            return

        if op == "status":
            r = _cam_get_or_fetch(cam, f"{cam}:status", _CACHE_TTL["status"],
                                  client.get)
            if r is None:
                self._send_json(200, {})
                return
            self._send_json(200, r)
            return

        if op == "leds":
            r = _cam_get_or_fetch(cam, f"{cam}:leds", _CACHE_TTL["leds"],
                                  client.get_led_state)
            if r is None:
                self._send_json(200, {"ir850": None, "ir940": None, "ircut": None})
                return
            self._send_json(200, r)
            return

        if op == "snapshot":
            # JPEG passthrough from cam's /x/ch1.jpg (640x360 substream —
            # matches calibration.json frame_size). Optional ?ch=0 for the
            # full-res main stream. Returns image/jpeg bytes.
            ch = 1
            if query:
                params = dict(urllib.parse.parse_qsl(query))
                if params.get("ch") == "0":
                    ch = 0
            data = _cam_get_or_fetch(
                cam, f"{cam}:snapshot:{ch}", _CACHE_TTL["snapshot"],
                lambda: client.get_snapshot(channel=ch))
            if data is None:
                self._send_text(503, "(snapshot fetch failed)\n")
                return
            self._send_bytes(200, "image/jpeg", data)
            return

        if op == "cal-info":
            # Combined cal-info: irlink status JSON (live runtime state) +
            # saved-cal JSON (authoritative "where I see my peer", even if
            # this cam was only the holder in the last cal flow) + a
            # `snapshot_url` the client can hit to display the live frame.
            #
            # The webui's CAL RESULTS panel uses this to refresh both cams
            # without re-running cal. saved_cal.tx_pixel/peak_brightness/
            # grid_delta is the source of truth for the LIVE CAL crosshair
            # + readout — irlink-status's cal.peak is empty/stale on the
            # holder side of a protocol-path monocal.
            r = _cam_get_or_fetch(cam, f"{cam}:status", _CACHE_TTL["status"],
                                  client.get)
            if r is None:
                r = {}
            else:
                r = dict(r)  # defensive copy — we mutate below
            saved = _cam_get_or_fetch(cam, f"{cam}:saved-cal", 10.0,
                                      client.get_saved_cal)
            if saved is not None:
                r["saved_cal"] = saved
            r["snapshot_url"] = f"/api/{cam}/snapshot"
            self._send_json(200, r)
            return

        if op == "grid-deltas":
            # Per-cam fetch of last do_grid_calibration result. ~1.5KB JSON
            # with the full 240-block post-OSD-mask delta vector + best
            # block + delta. Used by the operator to debug "monocal landed
            # at the wrong block" without SSHing the cam.
            r = _cam_get_or_fetch(cam, f"{cam}:grid-deltas", 1.0,
                                  client.get_grid_deltas)
            if r is None:
                self._send_json(404, {"state": "never_run"})
                return
            self._send_json(200, r)
            return

        if op == "events":
            # Plain-text passthrough of /tmp/irlink-events.log.
            # ?tail=N forwarded to the CGI for live tailing without bloat.
            tail = None
            if query:
                params = dict(urllib.parse.parse_qsl(query))
                t = params.get("tail")
                if t and t.isdigit():
                    tail = int(t)
            if not _probe_alive(client.ip):
                self._send_text(503, "(cam unreachable)\n")
                return
            body = client.get_events_log(tail=tail)
            if body is None:
                self._send_text(503, "(could not reach events-log.cgi)\n")
                return
            self._send_text(200, body)
            return

        self._send_json(404, {"ok": False, "error": f"unknown op {op!r}"})

    def _handle_api_post(self, path: str):
        # /api/cal/start (special — kicks off the cal worker thread)
        if path == "/api/cal/start":
            with CAL_STATE["lock"]:
                if CAL_STATE["running"]:
                    self._send_json(409, {"ok": False, "error": "cal already running"})
                    return
                CAL_STATE["running"] = True
            t = threading.Thread(target=_cal_run, daemon=True)
            t.start()
            self._send_json(200, {"ok": True})
            return
        # /api/setup-all — POST setup + time-sync to every initialised cam in
        # parallel. Each cam-side CGI takes ~1-2s; running them concurrently
        # keeps webui.sh restart fast. Time sync is bundled here because the
        # cams have no RTC and outbound NTP is blocked — every preflight is
        # a chance to re-anchor their wall clocks to the laptop's time.
        if path == "/api/setup-all":
            results: dict[str, dict] = {}
            threads = []
            def _setup_one(name: str, cl: CamStatusClient):
                report = cl.run_setup() or {"ok": False,
                                            "error": "unreachable",
                                            "steps": []}
                tsync = cl.set_time()  # uses laptop's now()
                report["time_sync"] = tsync or {"ok": False,
                                                "error": "time-sync unreachable"}
                results[name] = report
            for name in CAM_HOSTS:
                cl = self._client_for(name)
                if cl is None:
                    results[name] = {"ok": False, "error": "not initialised",
                                     "steps": []}
                    continue
                t = threading.Thread(target=_setup_one, args=(name, cl),
                                     daemon=True)
                t.start(); threads.append(t)
            for t in threads:
                t.join(timeout=25)
            overall_ok = bool(results) and all(
                r.get("ok") for r in results.values())
            self._send_json(200 if overall_ok else 207,
                            {"ok": overall_ok, "cams": results})
            return
        # /api/cal/monocal — trigger over-light cal of cam1 (cam2 holds, cam1 scans).
        # All work happens on cam2 via /x/monocal-trigger.cgi; the laptop just
        # reads cam2's pixel from calibration.json and POSTs the CGI.
        if path == "/api/cal/monocal":
            cam2 = self._client_for("cam2")
            if cam2 is None:
                self._send_json(503, {"ok": False, "error": "cam2 not initialised"})
                return
            body = self._read_body()
            params = dict(urllib.parse.parse_qsl(body))
            coords_str = params.get("coords")
            if coords_str:
                try:
                    cx, cy = (int(p) for p in coords_str.split(",", 1))
                except (ValueError, TypeError):
                    self._send_json(400, {"ok": False,
                                          "error": "coords must be X,Y"})
                    return
                coords = (cx, cy)
            else:
                coords = _read_cam2_sees_cam1_pixel()
                if coords is None:
                    self._send_json(400, {"ok": False,
                        "error": "no coords; run cal_procedure first or pass coords=X,Y"})
                    return
            try:
                speed_ms = int(params.get("speed_ms", "160"))
            except (ValueError, TypeError):
                speed_ms = 160
            result = cam2.start_monocal(coords, speed_ms=speed_ms)
            if result is None:
                self._send_json(409, {"ok": False,
                    "error": "monocal trigger refused (irlink already running on cam2?)"})
                return
            self._send_json(200, {"ok": True, "coords": list(coords),
                                  "speed_ms": speed_ms, "trigger": result})
            return
        parts = path.strip("/").split("/")
        if len(parts) != 3:
            self._send_json(404, {"ok": False, "error": "bad path"})
            return
        _, cam, op = parts
        client = self._client_for(cam)
        if client is None:
            self._send_json(503, {"ok": False, "error": f"cam {cam!r} not initialised"})
            return
        body = self._read_body()
        params = dict(urllib.parse.parse_qsl(body))

        if op == "ae":
            value = params.get("value")
            if value not in ("0", "1"):
                self._send_json(400, {"ok": False, "error": "value must be 0 or 1"})
                return
            ok = client.set_ae_freeze(value == "1")
            # Cached GET would otherwise return the prior value for up to 2s.
            _cache_invalidate(f"{cam}:ae", f"{cam}:status", f"{cam}:cal-info")
            self._send_json(200 if ok else 502, {"ok": ok, "value": int(value) if ok else None})
            return

        if op == "time-sync":
            # Push laptop's current epoch to this cam. Manual trigger; the
            # /api/setup-all flow + the periodic background thread cover the
            # automatic case.
            report = client.set_time()
            if report is None:
                self._send_json(502, {"ok": False,
                                      "error": "time-sync CGI unreachable"})
                return
            self._send_json(200, report)
            return

        if op == "setup":
            # Single-cam: hit /x/setup.cgi on the named cam. Body is empty.
            # 200 if every step ok, 207 (multi-status) if some steps failed
            # but the report came back — caller inspects `steps[]`.
            report = client.run_setup()
            if report is None:
                self._send_json(502, {"ok": False, "error": "setup CGI unreachable"})
                return
            self._send_json(200 if report.get("ok") else 207, report)
            return

        if op == "led":
            lamp = params.get("lamp")
            value = params.get("value")
            if lamp not in ("ir850", "ir940") or value not in ("0", "1"):
                self._send_json(400, {"ok": False, "error": "lamp ∈ {ir850,ir940}; value ∈ {0,1}"})
                return
            # Force night mode on first LED-on so daynight loop doesn't fight us.
            if value == "1":
                client.imp_cmd("daynight", "night")
            ok = client.imp_cmd(lamp, int(value))
            # Live LED state is reflected in /leds, /status, /cal-info, /proc.
            _cache_invalidate(f"{cam}:leds", f"{cam}:status",
                              f"{cam}:cal-info", f"{cam}:proc")
            self._send_json(200 if ok else 502, {"ok": ok, "lamp": lamp, "value": int(value) if ok else None})
            return

        self._send_json(404, {"ok": False, "error": f"unknown op {op!r}"})


_TIME_SYNC_INTERVAL_S = 3600  # every hour

def _periodic_time_sync_loop():
    """Background daemon: re-push laptop time to every cam every hour.
    Cams have no RTC and outbound NTP is blocked, so without this they drift
    apart over long-running sessions. Best-effort: failures (cam offline,
    rebooting) are silently retried next tick. Started from main()."""
    while True:
        time.sleep(_TIME_SYNC_INTERVAL_S)
        for name, cl in list(CLIENTS.items()):
            try:
                r = cl.set_time()
                if r and r.get("drift_s") is not None:
                    drift = r["drift_s"]
                    if abs(drift) >= 2:
                        print(f"[time-sync] {name}: corrected drift {drift:+}s")
            except Exception as e:
                print(f"[time-sync] {name}: {type(e).__name__}: {e}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--bind", default="127.0.0.1")
    args = p.parse_args()

    print(f"liwifi 8-bit live-status server")
    print(f"  webui dir: {WEBUI_DIR}")
    print(f"  resolving cams:")
    init_clients()
    print(f"  listening: http://{args.bind}:{args.port}/")
    threading.Thread(target=_periodic_time_sync_loop, daemon=True).start()

    httpd = ThreadingHTTPServer((args.bind, args.port), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down…")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
