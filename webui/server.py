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
import subprocess
import sys
import threading
import time
import urllib.parse
from collections import deque
from http.server import HTTPServer, SimpleHTTPRequestHandler

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from host.cam_status import CamStatusClient, resolve_ip  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

WEBUI_DIR = os.path.dirname(os.path.abspath(__file__))

CALIBRATION_JSON_PATH = os.path.join(REPO_ROOT, "host", "calibration.json")


def _read_cam2_sees_cam1_pixel() -> tuple[int, int] | None:
    """Pull cam2's pixel for cam1's TX out of host/calibration.json.

    Monocal needs this as cam2's --pixel arg so cam2's RX can decode cam1's
    CAL_ACK + CAL_DONE over light. Returns None if the file is missing or
    the field isn't there (operator must run cal_procedure first)."""
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
        subprocess.run(
            ["ssh", "-o", "BatchMode=yes", "dacam1",
             "killall -9 irlink 2>/dev/null; "
             "echo 0 > /run/prudynt/ae_freeze 2>/dev/null"],
            timeout=15, capture_output=True,
        )
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

    def _send_json(self, code: int, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
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
        path = urllib.parse.urlparse(self.path).path
        if path.startswith("/api/"):
            self._handle_api_get(path)
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

    def _handle_api_get(self, path: str):
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
            cam2_status = cam2.get_monocal_status() if cam2 else None
            cam1_status = cam1.get() if cam1 else None
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
            r = client.get_brightness_grid()
            if r is None:
                self._send_json(200, {"ts": None, "blocks": None})
                return
            ts, blocks = r
            self._send_json(200, {"ts": ts, "blocks": blocks})
            return

        if op == "ae":
            v = client.get_ae_freeze()
            self._send_json(200, {"value": v})
            return

        if op == "proc":
            r = client.get_proc_status()
            if r is None:
                self._send_json(200, {"irlink": None, "daynightd": None, "prudynt": None})
                return
            self._send_json(200, r)
            return

        if op == "status":
            r = client.get()
            if r is None:
                self._send_json(200, {})
                return
            self._send_json(200, r)
            return

        if op == "leds":
            r = client.get_led_state()
            if r is None:
                self._send_json(200, {"ir850": None, "ir940": None, "ircut": None})
                return
            self._send_json(200, r)
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
            self._send_json(200 if ok else 502, {"ok": ok, "value": int(value) if ok else None})
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
            self._send_json(200 if ok else 502, {"ok": ok, "lamp": lamp, "value": int(value) if ok else None})
            return

        self._send_json(404, {"ok": False, "error": f"unknown op {op!r}"})


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

    httpd = HTTPServer((args.bind, args.port), Handler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down…")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
