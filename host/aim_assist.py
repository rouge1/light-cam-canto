#!/usr/bin/env python3
"""aim_assist — laptop tool to point cam2 at cam1 with live feedback.

Workflow:
  0. Run host/cam_setup.sh on both cams (puts them in correct ISP state)
  1. SSH cam1 → kill daemon, force LEDs off
  2. Pre-flight: re-verify cam_setup state, freeze cam2 AE
  3. Measure 240-block baseline on cam2 (LEDs off, AE frozen)
  4. Turn cam1 IR LEDs ON; aim_loop refreshes them every 30s as keepalive
  5. Aim loop: 4 Hz live max-delta-block (★ GOOD / ◆ OK / · weak); when
     weak, dump top-5 candidates 1 Hz so user sees where to point
  6. User moves cam2 until aim is good, presses Enter
  7. Tool kills cam1 LEDs, restarts cam1's daemon-listen, runs bicall
     from cam2 → fresh /opt/etc/calibration.json on both sides.

Currently uses SSH to drive cam1 (dev mode — cam1 still on WiFi). Replace
the SSH LED-hold with an AIM_REQ message over light when cam1 goes truly
autonomous.

Usage:
  python -m host.aim_assist                          # setup → aim → bicall
  python -m host.aim_assist --no-cal                 # setup → aim only
  python -m host.aim_assist --skip-setup             # skip cam_setup.sh
  python -m host.aim_assist --cam2-pixel 271,91      # cam2's coord for bicall
  python -m host.aim_assist --probe setup-only       # just run cam_setup
  python -m host.aim_assist --probe cam2-state       # cam2 ISP + top-10 brightest
  python -m host.aim_assist --probe cam1-leds        # verify cam1 LEDs fire
  python -m host.aim_assist --probe baseline         # 20×12 ASCII heatmap
  python -m host.aim_assist --probe one-cycle        # one LED on/off, top-10 deltas
"""
import argparse
import os
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty

from host.cam_status import CamStatusClient, format_status_line, resolve_ip

CAM1 = "dacam1"
CAM2 = "dacam2"

RED = "\033[0;31m"
YEL = "\033[0;33m"
GRN = "\033[0;32m"
CYA = "\033[0;36m"
RST = "\033[0m"

THRESHOLD_GOOD = 80
THRESHOLD_OK = 40

# SSH timeouts — generous because cam WiFi can spike to 500ms+ RTT
SSH_CONNECT_TIMEOUT = 10
SSH_TIMEOUT = 15
SSH_RETRIES = 3

# Repo root (one level up from host/) for shelling out to host/cam_setup.sh
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def ssh(host, cmd, timeout=SSH_TIMEOUT, retries=SSH_RETRIES):
    """SSH with retry-on-transient-failure. Returns stdout on success, None on
    final failure. WiFi here can drop 1-in-3 SSH connects, so retry."""
    for attempt in range(retries):
        try:
            r = subprocess.run(
                ["ssh", "-o", f"ConnectTimeout={SSH_CONNECT_TIMEOUT}",
                 "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
                 "-o", "BatchMode=yes", host, cmd],
                capture_output=True, text=True, timeout=timeout,
            )
            if r.returncode == 0:
                return r.stdout
        except subprocess.TimeoutExpired:
            pass
        if attempt < retries - 1:
            time.sleep(1)
    return None


# Per-host CamStatusClient cache. aim_loop calls read_grid() at 4 Hz, and
# preflight calls get_ae_freeze() once per pre-aim check; both should reuse
# the same cookie-cached client for a host across an aim_assist run rather
# than logging in fresh per call.
_cam_clients: dict[str, CamStatusClient] = {}
_cam_http_disabled: dict[str, bool] = {}


def _get_cam_client(host: str) -> CamStatusClient | None:
    """Lazy-cached CamStatusClient per host. None if HTTP can't be reached
    (e.g. unresolvable hostname) — caller falls back to SSH where defined."""
    if _cam_http_disabled.get(host):
        return None
    if host not in _cam_clients:
        try:
            _cam_clients[host] = CamStatusClient(resolve_ip(host))
        except RuntimeError:
            _cam_http_disabled[host] = True
            return None
    return _cam_clients[host]


def read_grid(host, retries=3):
    """Read /run/prudynt/brightness_grid (timestamp + 240 blocks).

    HTTP-first via /x/brightness-grid.cgi (Phase 1 of the SSH→web migration);
    falls back to the legacy SSH `cat` path if the CGI is missing or the
    HTTP read fails. The fallback keeps this module working on cameras that
    don't have brightness-grid.cgi deployed yet."""
    client = _get_cam_client(host)
    if client is not None:
        result = client.get_brightness_grid()
        if result is not None:
            return result
        # HTTP returned None — fall through to SSH. Don't disable HTTP
        # globally on a single failure (the CGI may just be transiently
        # busy); the per-call fallback already covers that case.

    for attempt in range(retries):
        out = ssh(host, "cat /run/prudynt/brightness_grid", timeout=8)
        if out:
            parts = out.split()
            if len(parts) >= 241:
                try:
                    ts = int(parts[0])
                    blocks = [int(x) for x in parts[1:241]]
                    return ts, blocks
                except ValueError:
                    pass
        if attempt < retries - 1:
            time.sleep(0.5)
    return None


def parse_kv(s):
    """Parse 'k=v' lines into a dict, stripping whitespace."""
    out = {}
    for line in s.strip().split("\n"):
        if "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def find_peak_delta(baseline_blocks, current_blocks):
    """Return (idx, delta, current_brightness, col, row) for the block with
    the biggest positive delta vs baseline. This cancels static glare:
    a fixed bright spot has delta ~ 0, while the IR LED's block lights up."""
    best_idx, best_delta = 0, current_blocks[0] - baseline_blocks[0]
    for i in range(1, 240):
        d = current_blocks[i] - baseline_blocks[i]
        if d > best_delta:
            best_delta, best_idx = d, i
    return best_idx, best_delta, current_blocks[best_idx], best_idx % 20, best_idx // 20


def find_top_n_deltas(baseline_blocks, current_blocks, n=5):
    """Return top-n blocks by positive delta as list of
    (idx, delta, bright, base, col, row) sorted by delta desc."""
    rows = []
    for i in range(240):
        d = current_blocks[i] - baseline_blocks[i]
        rows.append((i, d, current_blocks[i], baseline_blocks[i], i % 20, i // 20))
    rows.sort(key=lambda r: -r[1])
    return rows[:n]


def dump_grid_ascii(blocks, label):
    """20×12 heatmap binned to 10 brightness chars (NV12 max ≈ 235)."""
    chars = " .:-=+*#%@"
    print(label)
    for row in range(12):
        line = ""
        for col in range(20):
            v = blocks[row * 20 + col]
            bin_idx = min(9, max(0, v // 24))
            line += chars[bin_idx]
        print(f"  {line}")


def set_ae_freeze(host, on):
    """Write 0 or 1 to /run/prudynt/ae_freeze via /x/ae-freeze.cgi (Phase 3).

    Returns True iff the server confirmed the write. No SSH fallback by
    design — writes fail loud so the caller surfaces a real error rather
    than silently diverging from the cam's actual ISP state."""
    client = _get_cam_client(host)
    if client is None:
        return False
    return client.set_ae_freeze(bool(on))


def measure_baseline_blocks(host, n=8):
    """Average all 240 grid blocks over N samples while the IR LEDs are OFF."""
    print(f"Measuring baseline (cam1 LEDs OFF) on {host} — {n} samples...")
    sums = [0] * 240
    count = 0
    for _ in range(n):
        g = read_grid(host)
        if g is not None:
            for i in range(240):
                sums[i] += g[1][i]
            count += 1
        time.sleep(0.25)
    if count == 0:
        raise RuntimeError(f"Could not read brightness_grid from {host}")
    blocks = [s // count for s in sums]
    print(f"  baseline captured ({count} samples; max block={max(blocks)}, "
          f"min={min(blocks)})")
    return blocks


def kill_cam1_daemon(host):
    """Stop any irlink daemon on cam1 so we can hold LEDs manually."""
    ssh(host, "killall irlink 2>/dev/null", timeout=SSH_TIMEOUT)
    time.sleep(1)
    ssh(host, "echo 0 > /run/prudynt/ae_freeze 2>/dev/null", timeout=SSH_TIMEOUT)


def turn_leds_on(host):
    """Turn on both IR LEDs via /x/json-imp.cgi (Phase 5).

    aim_loop calls this periodically (every 30s) as a keepalive so LEDs stay
    on for the duration of an aim session. We force night mode first so the
    daynight loop doesn't fight manual control, then enable both LEDs."""
    client = _get_cam_client(host)
    if client is None:
        return False
    client.imp_cmd("daynight", "night")
    ok1 = client.imp_cmd("ir850", 1)
    ok2 = client.imp_cmd("ir940", 1)
    return ok1 and ok2


def force_leds_off(host):
    """Turn off both IR LEDs via /x/json-imp.cgi (Phase 5)."""
    client = _get_cam_client(host)
    if client is None:
        return False
    ok1 = client.imp_cmd("ir850", 0)
    ok2 = client.imp_cmd("ir940", 0)
    return ok1 and ok2


def stdin_has_data():
    return select.select([sys.stdin], [], [], 0)[0]


# ---------------------------------------------------------------------------
# Setup + pre-flight
# ---------------------------------------------------------------------------

def run_cam_setup(cam1, cam2):
    """Invoke ./host/cam_setup.sh — canonical state-fixer for both cams.

    cam_setup.sh handles: prudynt running, daynightd killed, night/mono mode,
    LEDs off, AE freeze idle, brightness_grid present, irlink binary, ircut
    open (LAST, because other ISP commands re-close it). Idempotent — safe
    to call every aim_assist run.
    """
    print(f"{CYA}=== cam_setup.sh — verifying camera state ==={RST}")
    try:
        r = subprocess.run(
            ["./host/cam_setup.sh", cam1, cam2],
            cwd=REPO_ROOT,
            timeout=180,
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError("cam_setup.sh timed out after 180s")
    if r.returncode != 0:
        raise RuntimeError(
            f"cam_setup.sh reported errors (rc={r.returncode}) — fix before "
            "aiming.\nCommon causes: ircut closed, daynightd respawned, "
            "prudynt not running.\nTry: ./host/cam_setup.sh --reboot <camera>"
        )


# Phase 4 of SSH→web migration: pgrep probes pulled out of preflight; only
# the brightness_grid existence check stays on SSH (it's a 32-byte head test
# of the file's wc -c, distinct from the full grid read which uses HTTP).
_PREFLIGHT_GRID_CHECK_CAM2 = (
    'head -c 32 /run/prudynt/brightness_grid 2>/dev/null | wc -c'
)


def _read_ae_via_http(host):
    """HTTP-only read of /run/prudynt/ae_freeze via /x/ae-freeze.cgi.

    Returns the int (0 or 1), or None if the CGI is missing or unreachable.
    Phase 2 has no SSH fallback — fail-loud. Callers treat None as 'unknown'
    and surface that to the operator rather than silently retrying."""
    client = _get_cam_client(host)
    if client is None:
        return None
    return client.get_ae_freeze()


def _read_proc_via_http(host):
    """HTTP-only read of /x/proc-status.cgi.

    Returns the {irlink, daynightd, prudynt, ts_s} dict, or None on failure.
    Phase 4 has no SSH fallback — preflight surfaces the None as '?'."""
    client = _get_cam_client(host)
    if client is None:
        return None
    return client.get_proc_status()


def preflight(cam1, cam2):
    """Re-verify cam state after cam_setup; freeze cam2 AE. Returns True on all-OK.

    ircut state is NOT re-checked here — `gpio read 52` returns the GPIO
    pin's current output, not the filter's latched state, so it lies after
    cam_setup releases the open pulse. Trust cam_setup.sh's heartbeat-based
    ircut verification (its rc bubbles up via run_cam_setup).

    What we DO check: irlink/daynightd dead, AE freeze took on cam2,
    brightness_grid present."""
    print(f"\n{CYA}=== Pre-flight (post-setup re-verify) ==={RST}")
    ok = True

    # cam1 — all reads via HTTP (Phase 2 ae + Phase 4 proc).
    ae1 = _read_ae_via_http(cam1)
    proc1 = _read_proc_via_http(cam1)
    if proc1 is None:
        print(f"  {RED}cam1 ({cam1}): proc-status.cgi unreachable{RST}")
        return False
    cam1_idle = (proc1["irlink"] == 0
                 and ae1 in (0, None)
                 and proc1["daynightd"] == 0)
    ae1_str = "?" if ae1 is None else str(ae1)
    status = (GRN + "OK" + RST) if cam1_idle else (RED + "FAIL" + RST)
    print(f"  cam1 ({cam1}): irlink={proc1['irlink']} "
          f"ae={ae1_str} dnd={proc1['daynightd']}  {status}")
    if not cam1_idle:
        ok = False
        if proc1["daynightd"] != 0:
            print(f"    {RED}HINT: cam1 daynightd respawned — kill it{RST}")
        if proc1["irlink"] != 0:
            print(f"    {RED}HINT: cam1 irlink still running — kill_cam1_daemon failed?{RST}")

    # Set AE freeze on cam2 now, then verify it stuck.
    if not set_ae_freeze(cam2, True):
        print(f"  {RED}cam2 ({cam2}): could not set ae_freeze=1{RST}")
        return False
    time.sleep(0.5)

    ae2 = _read_ae_via_http(cam2)
    proc2 = _read_proc_via_http(cam2)
    grid_bytes = ssh(cam2, _PREFLIGHT_GRID_CHECK_CAM2)
    if proc2 is None:
        print(f"  {RED}cam2 ({cam2}): proc-status.cgi unreachable{RST}")
        return False
    cam2_ae_frozen = ae2 == 1
    cam2_dnd_dead = proc2["daynightd"] == 0
    cam2_grid_ok = (grid_bytes or "0").strip() not in ("0", "")
    cam2_ok = cam2_ae_frozen and cam2_dnd_dead and cam2_grid_ok
    ae2_str = "?" if ae2 is None else str(ae2)
    status = (GRN + "OK" + RST) if cam2_ok else (RED + "FAIL" + RST)
    print(f"  cam2 ({cam2}): ae={ae2_str} "
          f"dnd={proc2['daynightd']} "
          f"grid={(grid_bytes or '?').strip()}B  {status}")
    if not cam2_ok:
        ok = False
        if not cam2_ae_frozen:
            print(f"    {RED}HINT: AE freeze did not stick (ae_freeze "
                  f"missing, CGI down, or write didn't apply?){RST}")
        if not cam2_dnd_dead:
            print(f"    {RED}HINT: cam2 daynightd respawned — kill it{RST}")
        if not cam2_grid_ok:
            print(f"    {RED}HINT: brightness_grid empty — prudynt-patched "
                  f"may not be running{RST}")

    return ok


# ---------------------------------------------------------------------------
# Aim loop
# ---------------------------------------------------------------------------

def aim_loop(baseline_blocks, cam2, cam1, led_keepalive_s=30):
    """Live aim loop. Refreshes cam1 LEDs every led_keepalive_s seconds so
    they stay on for the duration. Returns 'cal' on Enter, 'quit' on q."""
    print()
    print(f"{CYA}Move cam2 until you see GOOD aim. [Enter]=cal  [q]=quit{RST}")
    print()
    last_len = 0
    iter_count = 0
    last_led_refresh = time.time()
    while True:
        # LED keepalive: re-issue light-on every N seconds so the LEDs don't
        # drop out mid-aim if a daynightd respawns or some prior 'sleep N; off'
        # pipeline expires. Cheap (~1 SSH every 30s).
        if time.time() - last_led_refresh > led_keepalive_s:
            turn_leds_on(cam1)
            last_led_refresh = time.time()
        if stdin_has_data():
            ch = sys.stdin.read(1)
            if ch in ("\r", "\n"):
                print()
                return "cal"
            if ch.lower() == "q":
                print()
                return "quit"
        g = read_grid(cam2)
        if g is None:
            line = "ERR: cam2 not responding"
            sys.stdout.write("\r" + " " * last_len + "\r" + RED + line + RST)
            sys.stdout.flush()
            last_len = len(line)
        else:
            _, delta, bright, bx, by = find_peak_delta(baseline_blocks, g[1])
            base = baseline_blocks[by * 20 + bx]
            if delta >= THRESHOLD_GOOD:
                tag, color = "★ GOOD AIM", GRN
            elif delta >= THRESHOLD_OK:
                tag, color = "◆ OK", YEL
            else:
                tag, color = "· weak", RED
            line = (f"IR-block ({bx:2d},{by:2d}) bright={bright:3d} "
                    f"baseline={base:3d} delta={delta:+4d}  {tag}")
            sys.stdout.write("\r" + " " * last_len + "\r" + color + line + RST)
            sys.stdout.flush()
            last_len = len(line)

            # Once per second, when weak, dump top-5 candidates as a scrolled
            # diagnostic block (live line resumes overwriting underneath).
            if iter_count > 0 and iter_count % 4 == 0 and delta < THRESHOLD_OK:
                sys.stdout.write("\n")
                top5 = find_top_n_deltas(baseline_blocks, g[1], n=5)
                print("  weak — top 5 by delta:")
                for _idx, d, b, base_, col, row in top5:
                    sat = " [SAT]" if base_ >= 230 else ""
                    print(f"    ({col:2d}, {row:2d}) b={b:3d} base={base_:3d} "
                          f"delta={d:+4d}{sat}")
                last_len = 0  # cursor is now at start of fresh line

        iter_count += 1
        time.sleep(0.25)


def reverse_check(cam1, cam2):
    """Verify cam1 sees cam2 (the bicall RX direction). aim_loop only shows
    cam2-sees-cam1; if the reverse direction is weak, bicall will fail at
    RX-side even though forward aim looked great. This catches the documented
    geometric-asymmetry pitfall before we waste 3 minutes on bicall.

    Returns peak delta on cam1, or None on read failure."""
    print(f"\n{CYA}=== Reverse check: does cam1 see cam2? ==={RST}")
    force_leds_off(cam1)
    force_leds_off(cam2)
    time.sleep(1)

    if not set_ae_freeze(cam1, True):
        print(f"  {YEL}Could not freeze cam1 AE — result will be noisier{RST}")
    time.sleep(0.5)

    try:
        cam1_baseline = measure_baseline_blocks(cam1, n=4)

        print(f"  Turning cam2 IR LEDs ON for 3 seconds...")
        ssh(cam2, "light ir850 on; light ir940 on")
        time.sleep(3)

        on_grid = read_grid(cam1, retries=5)
        ssh(cam2, "light ir850 off; light ir940 off")

        if on_grid is None:
            print(f"  {RED}Failed to read cam1's brightness_grid — cam1 SSH "
                  f"dropped. Reverse delta unknown.{RST}")
            return None
        _, on_blocks = on_grid

        idx, delta, bright, col, row = find_peak_delta(cam1_baseline, on_blocks)
        base = cam1_baseline[row * 20 + col]
        if delta >= THRESHOLD_GOOD:
            tag, color = "★ GOOD", GRN
        elif delta >= THRESHOLD_OK:
            tag, color = "◆ OK", YEL
        else:
            tag, color = "✗ WEAK", RED
        print(f"  cam1 sees cam2 at block ({col:2d},{row:2d}): "
              f"bright={bright} base={base} delta={delta:+d}  "
              f"{color}{tag}{RST}")

        # Show top-3 to help interpret weak results
        if delta < THRESHOLD_OK:
            print("  top 3 candidates:")
            top = find_top_n_deltas(cam1_baseline, on_blocks, n=3)
            for _i, d, b, base_, c, r in top:
                sat = " [SAT]" if base_ >= 230 else ""
                print(f"    ({c:2d},{r:2d}) b={b:3d} base={base_:3d} "
                      f"delta={d:+4d}{sat}")
        return delta
    finally:
        set_ae_freeze(cam1, False)
        ssh(cam2, "light ir850 off; light ir940 off")


def restart_cam1_daemon(host):
    """Read /opt/etc/calibration.json on cam1 and restart daemon-listen.

    Post-start verification prefers the CGI status endpoint (irlink writes
    /run/irlink-status.json on every protocol event, so once daemon-listen
    boots past 'starting', the JSON shows it). Falls back to the SSH pgrep
    probe if the CGI is unreachable — keeps this working on cams without
    cal-status.cgi deployed yet.
    """
    coords = ssh(host,
        'grep \'"tx_pixel"\' /opt/etc/calibration.json '
        '| grep -oE "[0-9]+" | head -2 | tr "\\n" "," | sed "s/,$//"',
        timeout=SSH_TIMEOUT)
    if not coords or not coords.strip():
        raise RuntimeError(f"{host}: /opt/etc/calibration.json missing or unparseable")
    coords = coords.strip()
    print(f"  starting cam1 daemon-listen --pixel {coords} --speed 160")
    subprocess.run(
        ["ssh", "-o", "BatchMode=yes", host,
         f"nohup /opt/bin/irlink daemon-listen --pixel {coords} "
         "--speed 160 >/var/log/irlink-boot.log 2>&1 &"],
        timeout=5,
    )

    # Two-tier verify (Phase 4): cal-status.cgi (irlink-internal state) is
    # the most informative; proc-status.cgi (process count) is a coarse but
    # cheap fallback if the daemon hasn't yet written its first status JSON.
    client = _get_cam_client(host)
    deadline = time.monotonic() + 6.0
    while client and time.monotonic() < deadline:
        s = client.get()
        if s and s.get("state") and s["state"] != "starting":
            print(f"  cam1 daemon-listen up (state={s['state']}, "
                  f"mode={s.get('mode','?')}, rate={s.get('rate_ms','?')}ms)")
            return
        time.sleep(0.5)

    # cal-status didn't move off "starting" — confirm via proc-status.cgi.
    if client is not None:
        proc = client.get_proc_status()
        if proc is not None and proc.get("irlink", 0) >= 1:
            print(f"  cam1 daemon-listen up (proc-status irlink={proc['irlink']}; "
                  f"cal-status still 'starting')")
            return

    raise RuntimeError(f"{host}: daemon-listen failed to start "
                       f"(neither cal-status.cgi nor proc-status.cgi confirmed)")


def _start_cam1_status_poller(host, stop_event, interval=2.0):
    """Background thread: polls cam1's /x/cal-status.cgi every `interval`s
    and prints a one-liner each tick. Gives the operator visibility into
    cam1's state during bicall (cam1 has no exposed stdout — only the JSON
    file). Returns the Thread object; caller sets stop_event to halt it.

    Designed to fail soft: a missing CGI, expired cookie, or transient HTTP
    error logs a brief note and keeps polling. Never raises into the main
    bicall stream loop."""
    client = _get_cam_client(host)
    if client is None:
        print(f"{YEL}[cam1-status] disabled — cannot resolve {host}{RST}")
        return None
    last_event_ms = 0  # suppress duplicate event spam

    def loop():
        nonlocal last_event_ms
        while not stop_event.is_set():
            s = client.get()
            if s is None:
                sys.stdout.write(f"{YEL}[cam1-status] (no status){RST}\n")
                sys.stdout.flush()
            else:
                # Only print when the last_event_ms advances (avoid 2s of
                # identical lines while cam1 sits in cal_holding).
                ev_ms = s.get("last_event_ms", 0)
                if ev_ms != last_event_ms:
                    last_event_ms = ev_ms
                    sys.stdout.write(f"{CYA}[cam1] {format_status_line(s)}{RST}\n")
                    sys.stdout.flush()
            stop_event.wait(interval)

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t


def run_bicall(cam2_pixel):
    """Spawn cam2 irlink connect, send bicall, stream output until done.
    Concurrently polls cam1's CGI so the operator sees both sides."""
    print(f"\n{CYA}=== bicall via cam2 (--pixel {cam2_pixel}) ==={RST}")
    print(f"{CYA}    (cam1 status polled every 2s via /x/cal-status.cgi){RST}")

    stop_poll = threading.Event()
    poll_thread = _start_cam1_status_poller(CAM1, stop_poll, interval=2.0)

    cmd = f"/opt/bin/irlink connect --pixel {cam2_pixel} --speed 160"
    proc = subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", CAM2, cmd],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    sent_bicall = False
    sent_quit = False
    try:
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            if not sent_bicall and "PROTO: connected!" in line:
                proc.stdin.write("bicall\n")
                proc.stdin.flush()
                sent_bicall = True
            if "BICAL: bidirectional cal complete" in line or "BICAL: phase 1" in line and "failed" in line:
                if not sent_quit:
                    proc.stdin.write("quit\n")
                    proc.stdin.flush()
                    sent_quit = True
        proc.wait(timeout=10)
    except (KeyboardInterrupt, subprocess.TimeoutExpired):
        proc.kill()
    finally:
        stop_poll.set()
        if poll_thread is not None:
            poll_thread.join(timeout=3.0)


# ---------------------------------------------------------------------------
# Probes — single-purpose diagnostic modes
# ---------------------------------------------------------------------------

def probe_cam1_leds(cam1):
    """Hold LEDs 5 s, verify GPIO47 (850) + PWM0 (940) are active."""
    print(f"\n{CYA}=== probe: cam1-leds ==={RST}")
    kill_cam1_daemon(cam1)
    print("Holding cam1 IR LEDs ON for 5 seconds...")
    ssh(cam1, "light ir850 on; light ir940 on")
    time.sleep(2)

    gpio47 = (ssh(cam1, "gpio read 47 2>/dev/null") or "").strip()
    pwm0 = (ssh(cam1, "cat /sys/class/pwm/pwmchip0/pwm0/enable 2>/dev/null") or "").strip()
    print(f"  GPIO 47 (850nm): {gpio47 or 'ERR':>3}  (expect 1)")
    print(f"  PWM0    (940nm): {pwm0 or 'ERR':>3}  (expect 1)")

    time.sleep(3)
    print("Turning LEDs off...")
    ssh(cam1, "light ir850 off; light ir940 off")

    if gpio47 == "1" and pwm0 == "1":
        print(f"{GRN}cam1 LEDs OK — GPIO47 high, PWM0 enabled{RST}")
        return 0
    print(f"{RED}cam1 LEDs FAILED — see values above. Common causes: "
          f"daynightd respawned (kill it), prudynt restarted (re-export PWM){RST}")
    return 1


def probe_cam2_state(cam2):
    """Dump cam2 ISP state + top-10 brightest blocks."""
    print(f"\n{CYA}=== probe: cam2-state ==={RST}")
    cmd = (
        'echo "AE freeze: $(cat /run/prudynt/ae_freeze 2>/dev/null)"; '
        'echo "daynightd: $(pgrep -af daynightd 2>/dev/null || echo none)"; '
        'echo "prudynt: $(pidof prudynt-patched > /dev/null && echo running || echo NOT-RUNNING)"; '
        'echo "ircut gpio52 (open): $(gpio read 52 2>/dev/null)"; '
        'echo "ircut gpio53 (close): $(gpio read 53 2>/dev/null)"; '
        'echo "ir850 gpio47: $(gpio read 47 2>/dev/null)"'
    )
    out = ssh(cam2, cmd)
    if out:
        for line in out.strip().split("\n"):
            print(f"  {line}")

    g = read_grid(cam2)
    if g is None:
        print(f"  {RED}brightness_grid read failed{RST}")
        return 1
    ts, blocks = g
    avg = sum(blocks) / 240
    print(f"\n  grid timestamp: {ts}")
    print(f"  grid brightness: avg={avg:.1f}, min={min(blocks)}, max={max(blocks)}")
    print(f"\n  Top 10 brightest blocks:")
    top = sorted(enumerate(blocks), key=lambda x: -x[1])[:10]
    for idx, b in top:
        sat = " [SAT]" if b >= 230 else ""
        print(f"    ({idx % 20:2d}, {idx // 20:2d}) bright={b:3d}{sat}")
    return 0


def probe_baseline(cam2):
    """Freeze AE, capture baseline, dump 20×12 ASCII heatmap."""
    print(f"\n{CYA}=== probe: baseline ==={RST}")
    if not set_ae_freeze(cam2, True):
        print(f"{RED}Could not set ae_freeze on {cam2}{RST}")
        return 1
    time.sleep(0.5)
    try:
        baseline = measure_baseline_blocks(cam2)
        print()
        dump_grid_ascii(baseline,
            "Baseline brightness map (cam2, LEDs off, AE frozen):")
        print(f"\n  legend: ' .:-=+*#%@' = bins of 24 brightness each (0..235)")
        sat_blocks = [(i, b) for i, b in enumerate(baseline) if b >= 230]
        if sat_blocks:
            print(f"\n  {YEL}{len(sat_blocks)} saturated block(s) (b>=230) — "
                  f"IR signal can't show positive delta there:{RST}")
            for idx, b in sat_blocks[:5]:
                print(f"    ({idx % 20:2d}, {idx // 20:2d}) b={b:3d}")
        return 0
    finally:
        set_ae_freeze(cam2, False)


def probe_one_cycle(cam1, cam2):
    """One LED on/off cycle. Definitive 'is there ANY signal?' test."""
    print(f"\n{CYA}=== probe: one-cycle ==={RST}")

    kill_cam1_daemon(cam1)
    force_leds_off(cam1)
    time.sleep(1)

    if not set_ae_freeze(cam2, True):
        print(f"{RED}Could not set ae_freeze on {cam2}{RST}")
        return 1
    time.sleep(0.5)

    try:
        baseline = measure_baseline_blocks(cam2)

        print(f"{CYA}Turning cam1 LEDs ON for 3 seconds...{RST}")
        ssh(cam1, "light ir850 on; light ir940 on")
        time.sleep(3)

        # read_grid retries internally; if it still returns None, kill LEDs
        # immediately so they don't keep blasting while we report the failure.
        on_grid = read_grid(cam2, retries=5)
        if on_grid is None:
            ssh(cam1, "light ir850 off; light ir940 off")
            print(f"{RED}Failed to read brightness_grid from {cam2} during ON "
                  f"phase. cam2 SSH dropped — re-run --probe one-cycle, or "
                  f"--probe cam2-state to investigate.{RST}")
            return 1
        ssh(cam1, "light ir850 off; light ir940 off")
        _, on_blocks = on_grid

        top = find_top_n_deltas(baseline, on_blocks, n=10)
        print("\nTop 10 blocks by delta (LEDs ON - baseline):")
        for _idx, d, bright, base, col, row in top:
            sat = " [SAT]" if base >= 230 else ""
            print(f"  ({col:2d}, {row:2d}) bright={bright:3d} base={base:3d} "
                  f"delta={d:+4d}{sat}")

        max_delta = top[0][1]
        print()
        if max_delta >= THRESHOLD_OK:
            print(f"{GRN}Strong signal — max delta = {max_delta}. "
                  f"Cameras well-aimed. Run full aim_assist for fine "
                  f"adjustment.{RST}")
        elif max_delta >= 10:
            print(f"{YEL}Weak signal — max delta = {max_delta}. "
                  f"Cameras misaimed; aim_assist live loop will guide you "
                  f"toward block ({top[0][4]}, {top[0][5]}).{RST}")
        else:
            print(f"{RED}No signal — max delta = {max_delta}. Re-aim cameras "
                  f"physically. cam1 LEDs may not be reaching cam2's field "
                  f"of view at all. Try --probe cam1-leds to confirm LEDs "
                  f"are actually firing.{RST}")
        return 0
    finally:
        set_ae_freeze(cam2, False)


def run_probe(probe, cam1, cam2):
    """Dispatch to the right probe. cam_setup ran already in main."""
    if probe == "setup-only":
        print(f"\n{GRN}cam_setup.sh completed.{RST}")
        return 0
    if probe == "cam1-leds":
        return probe_cam1_leds(cam1)
    if probe == "cam2-state":
        return probe_cam2_state(cam2)
    if probe == "baseline":
        return probe_baseline(cam2)
    if probe == "one-cycle":
        return probe_one_cycle(cam1, cam2)
    print(f"{RED}unknown probe: {probe}{RST}")
    return 1


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    global CAM1, CAM2
    p = argparse.ArgumentParser(
        description="Aim cam2 at cam1 with live feedback, then trigger bicall")
    p.add_argument("--cam1", default=CAM1)
    p.add_argument("--cam2", default=CAM2)
    p.add_argument("--cam2-pixel", default="258,92",
                   help="cam2's pixel coord for cam1's TX (passed to bicall)")
    p.add_argument("--no-cal", action="store_true",
                   help="skip bicall after aim — just print final aim quality")
    p.add_argument("--skip-setup", action="store_true",
                   help="skip cam_setup.sh — only when state is known good")
    p.add_argument("--probe", choices=[
        "setup-only", "cam1-leds", "cam2-state", "baseline", "one-cycle"],
        help="run a single diagnostic probe and exit")
    args = p.parse_args()
    CAM1, CAM2 = args.cam1, args.cam2

    # Step 0: cam_setup.sh — bulletproof state. Skippable for fast iterate.
    if not args.skip_setup:
        try:
            run_cam_setup(CAM1, CAM2)
        except RuntimeError as e:
            print(f"{RED}{e}{RST}", file=sys.stderr)
            return 1

    # Probe modes exit after their probe.
    if args.probe:
        try:
            return run_probe(args.probe, CAM1, CAM2)
        finally:
            set_ae_freeze(CAM2, False)

    # Full interactive aim flow needs a real TTY.
    if not sys.stdin.isatty():
        print("aim_assist needs a real TTY for keypress input", file=sys.stderr)
        return 1

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)

    def cleanup(*_):
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        except Exception:
            pass
        force_leds_off(CAM1)
        force_leds_off(CAM2)
        set_ae_freeze(CAM2, False)
        set_ae_freeze(CAM1, False)

    signal.signal(signal.SIGINT, lambda *_: (cleanup(), sys.exit(130)))

    try:
        print(f"\n{CYA}Stopping cam1 daemon (frees LED control)...{RST}")
        kill_cam1_daemon(CAM1)
        force_leds_off(CAM1)
        time.sleep(1)

        # Pre-flight re-verifies cam_setup state AND freezes cam2 AE.
        if not preflight(CAM1, CAM2):
            print(f"{RED}Pre-flight failed. Fix the issues above and retry.{RST}",
                  file=sys.stderr)
            return 1

        baseline_blocks = measure_baseline_blocks(CAM2)

        print(f"{CYA}Turning cam1 IR LEDs ON (keepalive every 30s){RST}")
        turn_leds_on(CAM1)
        time.sleep(2)

        tty.setcbreak(fd)
        action = aim_loop(baseline_blocks, CAM2, CAM1)
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)

        force_leds_off(CAM1)

        if action == "quit":
            print("Quit without cal.")
            return 0

        if args.no_cal:
            print("--no-cal: skipping bicall.")
            return 0

        # Before committing to a 3-minute bicall, verify cam1 actually sees
        # cam2 too. Catches geometric asymmetry: forward direction looks
        # great, reverse direction is too weak for bicall RX to succeed.
        rev = reverse_check(CAM1, CAM2)
        if rev is None or rev < THRESHOLD_OK:
            print(f"\n{RED}WARNING: reverse direction is "
                  f"{'unknown' if rev is None else f'weak (delta={rev:+d})'}. "
                  f"bicall will likely fail at RX-side. Common causes: cam1 "
                  f"angled away from cam2, or cam1's IR illumination range "
                  f"is exceeded.{RST}")
            try:
                resp = input("Proceed with bicall anyway? [y/N]: ").strip().lower()
            except EOFError:
                resp = ""
            if resp not in ("y", "yes"):
                print("Aborting bicall. Re-aim and try again.")
                return 0

        print(f"\n{CYA}=== restarting cam1 daemon-listen ==={RST}")
        restart_cam1_daemon(CAM1)
        run_bicall(args.cam2_pixel)
        return 0
    finally:
        cleanup()


if __name__ == "__main__":
    sys.exit(main())
