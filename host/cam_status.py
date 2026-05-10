"""Poll a camera's state via Thingino /x/ web endpoints.

Replaces SSH-based state probes (which were timing out under load and
contributing to the cam2 overload during cal). Each poll is a single
authenticated HTTP GET — much cheaper than spawning sshd + shell, and
easier on the camera's CPU.

Endpoints consumed:
- /x/cal-status.cgi      → /run/irlink-status.json (CamStatusClient.get())
- /x/brightness-grid.cgi → /run/prudynt/brightness_grid (.get_brightness_grid())

Usage:
    from host.cam_status import CamStatusClient
    client = CamStatusClient("192.168.50.143")
    s = client.get()
    grid = client.get_brightness_grid()  # (ts_ms, [240 blocks]) or None
"""
from __future__ import annotations

import json
import os
import subprocess
from typing import Optional

CAM_PASSWORD = "password"


def resolve_ip(host: str) -> str:
    """Resolve an SSH alias (dacam1, etc) to a numeric IP for the HTTP client.
    Numeric input passes through. Raises RuntimeError on lookup failure."""
    if host and all(c.isdigit() or c == "." for c in host):
        return host
    r = subprocess.run(
        ["getent", "hosts", host],
        capture_output=True, text=True, timeout=2,
    )
    if r.returncode == 0 and r.stdout.strip():
        return r.stdout.split()[0]
    raise RuntimeError(f"could not resolve host {host!r} via getent")


class CamStatusClient:
    """Per-camera client. Keeps a cookie file so we don't re-login per poll."""

    def __init__(self, ip: str, password: str = CAM_PASSWORD):
        self.ip = ip
        self.password = password
        self.cookie_file = f"/tmp/cam_cookie_{ip}.txt"

    def _login(self, timeout: float = 3.0) -> bool:
        """Authenticate and persist the session cookie. Returns False on
        timeout, network failure, or auth rejection. Never raises — callers
        rely on the get*/fetch contract that all failure modes return None."""
        try:
            r = subprocess.run(
                ["curl", "-s", "--max-time", str(timeout), "-c", self.cookie_file,
                 f"http://{self.ip}/x/login.cgi",
                 "-H", "Content-Type: application/json",
                 "-d", json.dumps({"username": "root", "password": self.password},
                                  separators=(",", ":"))],
                capture_output=True, text=True, timeout=timeout + 1,
            )
        except (subprocess.TimeoutExpired, OSError):
            return False
        return '"success":true' in (r.stdout or "")

    def _ensure_cookie(self) -> bool:
        """True if a cookie is present (or freshly obtained); False if login
        was needed and failed."""
        if not os.path.exists(self.cookie_file) or os.path.getsize(self.cookie_file) < 10:
            return self._login()
        return True

    def _fetch(self, path: str, timeout: float = 3.0,
               retry_login: bool = True) -> Optional[str]:
        """Authenticated GET against `http://<ip><path>`, returning the
        response body as text. Returns None on timeout, empty body, HTML
        error page, or post-retry-login Forbidden. Re-logs in once on
        cookie-expired Forbidden, then retries the GET.

        Used by get() and get_brightness_grid(). Centralizing the cookie
        machinery keeps each new endpoint a 5-line wrapper around _fetch."""
        if not self._ensure_cookie():
            return None
        try:
            r = subprocess.run(
                ["curl", "-s", "--max-time", str(timeout), "-b", self.cookie_file,
                 f"http://{self.ip}{path}"],
                capture_output=True, text=True, timeout=timeout + 1,
            )
        except (subprocess.TimeoutExpired, OSError):
            return None
        body = (r.stdout or "").strip()
        if not body:
            return None
        # Auth-required indicators come in two flavors depending on Thingino
        # version: older builds return HTML `<title>Forbidden</title>`, newer
        # return JSON `{"error":"Authentication required..."}` with HTTP 401.
        # Both mean re-login (cookie is wiped on cam reboot).
        if (body.startswith("<") and "Forbidden" in body) or \
           (body.startswith("{") and "Authentication required" in body):
            if retry_login:
                self._login()
                return self._fetch(path, timeout=timeout, retry_login=False)
            return None
        if body.startswith("<"):
            return None  # HTML error page — not our endpoint
        return body

    def _fetch_binary(self, path: str, timeout: float = 3.0,
                       retry_login: bool = True) -> Optional[bytes]:
        """Authenticated GET returning raw bytes. Used for snapshot endpoints
        like /x/ch1.jpg that serve JPEG payloads. Same cookie-and-retry
        machinery as _fetch but no text-decoding."""
        if not self._ensure_cookie():
            return None
        try:
            r = subprocess.run(
                ["curl", "-s", "--max-time", str(timeout), "-b", self.cookie_file,
                 f"http://{self.ip}{path}"],
                capture_output=True, timeout=timeout + 1,
            )
        except (subprocess.TimeoutExpired, OSError):
            return None
        body = r.stdout or b""
        if not body:
            return None
        # Detect auth-required at the byte level — both old HTML and new JSON
        # 401 flavors. See `_fetch` for the same logic on text endpoints.
        head = body[:200]
        if (body[:1] == b"<" and b"Forbidden" in head) or \
           (body[:1] == b"{" and b"Authentication required" in head):
            if retry_login:
                self._login()
                return self._fetch_binary(path, timeout=timeout,
                                          retry_login=False)
            return None
        if body[:1] == b"<":
            return None
        return body

    def get_snapshot(self, channel: int = 1) -> Optional[bytes]:
        """Fetch a single JPEG snapshot from /x/ch{0,1}.jpg.

        channel=1 is the 640x360 substream that matches calibration.json's
        frame_size — use that for displaying cal results. channel=0 is the
        full-resolution main stream. Returns JPEG bytes or None on failure."""
        if channel not in (0, 1):
            return None
        return self._fetch_binary(f"/x/ch{channel}.jpg")

    def _post(self, path: str, data: str, content_type: Optional[str] = None,
              timeout: float = 8.0,
              retry_login: bool = True) -> Optional[tuple[int, str]]:
        """Authenticated POST. Returns (http_status, body) on transport
        success (regardless of HTTP code — caller decides what's a success
        code), or None on timeout / network failure. Re-logs in once on
        cookie-expired 403 then retries.

        `content_type` defaults to curl's `application/x-www-form-urlencoded`;
        pass `"application/json"` (or similar) for JSON-body endpoints."""
        if not self._ensure_cookie():
            return None
        cmd = ["curl", "-s", "-w", "\n%{http_code}", "-b", self.cookie_file,
               "-X", "POST", "-d", data]
        if content_type:
            cmd += ["-H", f"Content-Type: {content_type}"]
        cmd.append(f"http://{self.ip}{path}")
        try:
            r = subprocess.run(cmd, capture_output=True, text=True,
                               timeout=timeout)
        except (subprocess.TimeoutExpired, OSError):
            return None
        out = r.stdout or ""
        nl = out.rfind("\n")
        if nl < 0:
            return None
        body = out[:nl]
        try:
            code = int(out[nl + 1:].strip())
        except ValueError:
            return None
        if code in (401, 403) and retry_login:
            self._login()
            return self._post(path, data, content_type=content_type,
                              timeout=timeout, retry_login=False)
        return code, body

    def get(self) -> Optional[dict]:
        """Return parsed irlink status JSON or None on failure."""
        body = self._fetch("/x/cal-status.cgi")
        if body is None:
            return None
        try:
            return json.loads(body)
        except json.JSONDecodeError:
            return None

    def get_brightness_grid(self) -> Optional[tuple[int, list[int]]]:
        """Return (ts_ms, [240 block brightnesses]) or None on failure.

        Wire format from /x/brightness-grid.cgi: "<ts> <b0> <b1> ... <b239>\\n"
        (242 space-separated ints). Mirrors what aim_assist.read_grid()
        returns from the SSH path, so the consumer doesn't change."""
        body = self._fetch("/x/brightness-grid.cgi")
        if body is None:
            return None
        if body.startswith("error"):
            return None
        parts = body.split()
        if len(parts) < 241:
            return None
        try:
            ts = int(parts[0])
            blocks = [int(x) for x in parts[1:241]]
            return ts, blocks
        except ValueError:
            return None

    def get_ae_freeze(self) -> Optional[int]:
        """Return current AE-freeze state (0 or 1), or None on failure.

        Reads /run/prudynt/ae_freeze via /x/ae-freeze.cgi. Used by
        aim_assist.preflight() to verify IMP_ISP_Tuning_SetAeFreeze took
        effect before measuring a baseline. None means the file is missing
        or the CGI is unreachable — the caller decides how to handle."""
        body = self._fetch("/x/ae-freeze.cgi")
        if body is None:
            return None
        if body.startswith("error"):
            return None
        try:
            v = int(body.strip())
        except ValueError:
            return None
        if v not in (0, 1):
            return None
        return v

    def set_ae_freeze(self, value: bool) -> bool:
        """POST to /x/ae-freeze.cgi to write 0 or 1. Returns True iff the
        server confirmed the write (HTTP 200 + `"ok":true`). Phase 3 contract:
        no SSH fallback — writes fail loud. Callers must surface the False
        result rather than silently retrying."""
        val = 1 if value else 0
        res = self._post("/x/ae-freeze.cgi", f"value={val}")
        if res is None:
            return False
        code, body = res
        return code == 200 and '"ok":true' in body

    def get_led_state(self) -> Optional[dict]:
        """Return {ir850: 0|1, ir940: 0|1} from /x/gpio-state.cgi.

        Reads the actual GPIO pin levels (gpio read 47/49). Used by the
        live-status webui to reflect real IR LED state during cal — the
        stock /x/json-heartbeat-slow.cgi reports an internal daynightd
        model that doesn't sync with manual imp_cmd writes, so we have
        our own GPIO probe."""
        body = self._fetch("/x/gpio-state.cgi")
        if body is None:
            return None
        try:
            d = json.loads(body)
        except json.JSONDecodeError:
            return None
        if not isinstance(d, dict):
            return None
        try:
            return {
                "ir850": int(d.get("ir850", 0)),
                "ir940": int(d.get("ir940", 0)),
            }
        except (TypeError, ValueError):
            return None

    def imp_cmd(self, cmd: str, val) -> bool:
        """Send an ISP command to /x/json-imp.cgi.

        Body: `{"cmd":"<cmd>","val":<val>}`. `val` is encoded as JSON, so
        ints land as `1`/`0` and strings round-trip with their quotes
        (mirrors cam_setup.sh:imp_cmd / cal_procedure.imp_cmd).

        Known commands (Thingino IMP controller):
        - `daynight` "auto"|"day"|"night"
        - `color`    0|1   (0=color, 1=monochrome)
        - `ircut`    0|1   (0=open, 1=closed)
        - `ir850`    0|1
        - `ir940`    0|1

        Returns True iff the server's response contains `"success"` as a
        substring (Thingino's `/x/json-imp.cgi` returns
        `{"code":200,"result":"success","message":""}` — same substring check
        used in cam_setup.sh and cal_procedure.py). Phase 5 contract: writes
        fail loud, no SSH fallback."""
        body = json.dumps({"cmd": cmd, "val": val}, separators=(",", ":"))
        res = self._post("/x/json-imp.cgi", body, content_type="application/json")
        if res is None:
            return False
        code, response_body = res
        return code == 200 and '"success"' in response_body

    def get_saved_cal(self) -> Optional[dict]:
        """Return the parsed /opt/etc/calibration.json (saved cal, JFFS2-
        persistent) via /x/saved-cal.cgi. This is the authoritative
        "where this cam sees its peer" record — always current after any
        cal flow, regardless of which side scanned. Distinct from
        get(): /run/irlink-status.json's `cal.peak` only reflects this
        cam's most recent scan and stays empty if the cam was the holder.

        Returns None on transport failure or missing/malformed file."""
        body = self._fetch("/x/saved-cal.cgi")
        if body is None:
            return None
        try:
            d = json.loads(body)
        except json.JSONDecodeError:
            return None
        if not isinstance(d, dict) or "tx_pixel" not in d:
            return None
        return d

    def get_grid_deltas(self) -> Optional[dict]:
        """Return the last on-camera `do_grid_calibration` result as parsed
        JSON `{ts_ms, best_block, best_delta, deltas:[240 ints]}` via
        /x/grid-deltas.cgi. The deltas array is post-OSD-mask (top + bottom
        block rows already zeroed) — same vector irlink's argmax saw.

        Use this to debug a wrong-block monocal pick: inspect the runner-up
        block's delta vs the chosen one without SSHing in to read stderr.

        Returns None on transport failure or if the cam has not run a grid
        cal since boot (CGI returns 404 → `{"state":"never_run"}`)."""
        body = self._fetch("/x/grid-deltas.cgi")
        if body is None:
            return None
        try:
            d = json.loads(body)
        except json.JSONDecodeError:
            return None
        if not isinstance(d, dict) or "deltas" not in d:
            return None
        return d

    def get_events_log(self, tail: Optional[int] = None) -> Optional[str]:
        """Return /tmp/irlink-events.log contents (plain text) via /x/events-log.cgi.

        Optional tail=N returns only the last N lines (less data over the wire
        for live tailing). None on transport failure. Returns the text body as
        a single string with embedded newlines — caller splits if needed."""
        path = "/x/events-log.cgi"
        if tail is not None and tail > 0:
            path += f"?tail={int(tail)}"
        body = self._fetch(path)
        if body is None:
            return None
        return body

    def start_monocal(self, coords: tuple[int, int],
                      speed_ms: int = 160) -> Optional[dict]:
        """POST to /x/monocal-trigger.cgi to spawn `irlink monocal` on this cam.

        coords is (x, y) — the pixel cam2 sees cam1's TX at, taken from the
        host-side calibration.json. The CGI refuses with HTTP 409 if any
        irlink is already running.

        Returns the parsed response dict on HTTP 200 (`{ok, pid, coords,
        speed_ms}`), or None on transport failure / 409 / 400. Caller can
        treat None as "not started" and either resolve the conflict or
        surface the failure to the operator."""
        body = f"coords={coords[0]},{coords[1]}&speed={int(speed_ms)}"
        res = self._post("/x/monocal-trigger.cgi", body)
        if res is None:
            return None
        code, response_body = res
        if code != 200:
            return None
        try:
            d = json.loads(response_body)
        except json.JSONDecodeError:
            return None
        return d if isinstance(d, dict) and d.get("ok") else None

    def get_monocal_status(self) -> Optional[dict]:
        """Return cam2's monocal status JSON (the `state`, `peer_pixel`, and
        timing fields written by `irlink monocal` to /run/monocal-status.json).

        Returns None on transport failure OR on the brief mid-rename window
        (CGI returns 503 with `{"state":"transient"}`). The webui's intended
        behavior is to keep polling — the next 1-2s tick will see a fresh
        atomic file. Per the planning, no flock; tolerate transient."""
        body = self._fetch("/x/monocal-status.cgi")
        if body is None:
            return None
        try:
            d = json.loads(body)
        except json.JSONDecodeError:
            return None
        if not isinstance(d, dict):
            return None
        # Both the "transient" and "never_run" sentinel states are valid
        # responses — the caller distinguishes them. Don't filter here.
        return d

    def set_time(self, epoch: Optional[int] = None,
                 timeout: float = 6.0) -> Optional[dict]:
        """POST /x/time-sync.cgi to set the cam's wall clock from the laptop.

        Cameras have no battery-backed RTC and outbound NTP (UDP 123) is
        commonly blocked, so ntpd never syncs. This pushes the laptop's
        time over HTTP instead. Pass `epoch=None` to use the laptop's
        current time. Returns the cam's report:

            {ok, old_epoch, new_epoch, drift_s}

        drift_s = old - new (positive = cam was ahead). None on transport
        failure. Latency between this call's send and the cam's `date -s`
        is sub-second on LAN, so accuracy is bounded by SSH RTT (~100ms)."""
        import time as _time
        if epoch is None:
            epoch = int(_time.time())
        body = f"epoch={int(epoch)}"
        res = self._post("/x/time-sync.cgi", body, timeout=timeout)
        if res is None:
            return None
        code, response_body = res
        if code != 200:
            return None
        try:
            d = json.loads(response_body)
        except json.JSONDecodeError:
            return None
        return d if isinstance(d, dict) else None

    def run_setup(self, timeout: float = 20.0) -> Optional[dict]:
        """POST /x/setup.cgi to put this cam into a known IR-comm-ready state.

        Single round-trip equivalent of the old multi-step laptop preflight:
        kills daynightd, sets night/mono via prudyntctl, forces LEDs off
        (`light` + raw `gpio clear`), resets ae_freeze, opens ircut last,
        verifies grid + prudynt + irlink. Returns:

            {ok: bool, steps: [{name, ok, detail}, ...]}

        or None on transport failure. ~1-2s wall time on slow MIPS.
        Higher default timeout than the readonly endpoints — this CGI does
        ~10 cam-side ops with a 1s sleep after `killall daynightd`."""
        res = self._post("/x/setup.cgi", "", timeout=timeout)
        if res is None:
            return None
        code, body = res
        if code != 200:
            return None
        try:
            d = json.loads(body)
        except json.JSONDecodeError:
            return None
        return d if isinstance(d, dict) else None

    def get_proc_status(self) -> Optional[dict]:
        """Return process counts as `{irlink:N, daynightd:N, prudynt:N, ts_s:T}`
        or None on failure.

        Replaces the SSH `pgrep`/`pidof` probes in aim_assist.preflight() and
        restart_cam1_daemon. Note: busybox `pgrep -c` is not available, so the
        CGI uses `pgrep <name> | wc -l` for irlink/daynightd and `pidof` for
        prudynt-patched (basename match). All values are non-negative ints."""
        body = self._fetch("/x/proc-status.cgi")
        if body is None:
            return None
        try:
            d = json.loads(body)
        except json.JSONDecodeError:
            return None
        # Defensive: only accept dicts with the expected int fields.
        if not isinstance(d, dict):
            return None
        for key in ("irlink", "daynightd", "prudynt"):
            v = d.get(key)
            if not isinstance(v, int) or v < 0:
                return None
        return d


def format_status_line(status: dict, prefix: str = "") -> str:
    """One-line summary for live progress display."""
    cal = status.get("cal", {})
    counters = status.get("counters", {})
    parts = [
        f"{prefix}{status.get('mode','?')}/{status.get('state','?')}",
        f"rate={status.get('rate_ms','?')}ms",
    ]
    if cal.get("active"):
        parts.append(f"cal phase={cal.get('phase')} bidi={cal.get('bidi')} "
                     f"retries={cal.get('retries')}")
    px, py = cal.get("peak", [-1, -1])
    if px >= 0:
        parts.append(f"peak=({px},{py}) d={cal.get('peak_delta',0)}")
    parts.append(f"tx={counters.get('tx',0)} rx={counters.get('rx',0)} "
                 f"df={counters.get('decode_fail',0)}")
    parts.append(f"event=\"{status.get('last_event','')}\"")
    return " ".join(parts)


if __name__ == "__main__":
    import argparse
    import time
    p = argparse.ArgumentParser(description="Poll a camera's irlink status via web API")
    p.add_argument("--cam", required=True, help="Camera IP or alias (resolved via /etc/hosts)")
    p.add_argument("--interval", type=float, default=2.0)
    args = p.parse_args()

    ip = resolve_ip(args.cam)

    client = CamStatusClient(ip)
    print(f"polling http://{ip}/x/cal-status.cgi every {args.interval}s — Ctrl-C to stop")
    while True:
        s = client.get()
        if s is None:
            print(f"[{time.strftime('%H:%M:%S')}] (no status — endpoint failed)")
        else:
            print(f"[{time.strftime('%H:%M:%S')}] {format_status_line(s)}")
        time.sleep(args.interval)
