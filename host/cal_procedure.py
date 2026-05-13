#!/usr/bin/env python3
"""Calibration procedure: find the exact pixel location of the peer's IR transmitter.

Steps:
  1. Capture LED-off frame from receiver camera
  2. Turn on transmitter's IR LEDs (850nm + 940nm)
  3. Capture LED-on frame
  4. Diff frames, find peak pixel = transmitter location
  5. Save calibration result (pixel coords + ROI) for pixel_rx

Usage:
    python -m host.cal_procedure                     # calibrate both directions
    python -m host.cal_procedure --rx cam2 --tx cam1  # one direction only
    python -m host.cal_procedure --show               # show last calibration

Calibration results are saved to host/calibration.json
"""
import cv2
import numpy as np
import subprocess
import json
import time
import sys
import os
import argparse
import threading
from datetime import datetime

from host.config import pick_initial_rate_ms
from protocol import app
from protocol.app import (
    APP_CAL_VISUAL, APP_CHUNK,
    CAL_VISUAL_PANEL_BYTES,
    pack_chunk, reassemble, unpack_cal_visual_panel,
)

# Resolve cam IPs from /etc/hosts at import time. /etc/hosts is the single
# source of truth — every time DHCP shuffles the leases (memory:
# DHCP IP rotation on reboot), only that file needs updating and every
# downstream (this script, cam_setup, pixel_rx, tx_resync, config) picks
# it up automatically. Falls back to the last-known IPs if getent fails
# so a misconfigured /etc/hosts doesn't ImportError the whole module.
from host.cam_status import resolve_ip

def _cam_ips():
    out = {}
    for cam, alias, fallback in (("cam1", "dacam1", "192.168.50.110"),
                                 ("cam2", "dacam2", "192.168.50.143")):
        try:
            out[cam] = resolve_ip(alias)
        except RuntimeError:
            out[cam] = fallback
    return out

CAMERAS = _cam_ips()
CAM_PASSWORD = "password"

PHOTOS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "photos")
CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
HTML_FILE = os.path.join(PHOTOS_DIR, "calibration.html")

sys.stdout.reconfigure(line_buffering=True)


def imp_cmd(ip, cmd, val):
    """Send a command to the camera's IMP controller via curl.

    Always re-logins to avoid stale cookie issues.
    Uses shell=True to avoid JSON quoting issues with subprocess.
    """
    cookie_file = f"/tmp/cam_cookie_{ip}.txt"
    val_json = json.dumps(val)
    # Login fresh every time
    subprocess.run(
        f"curl -s -c {cookie_file} 'http://{ip}/x/login.cgi' "
        f"-H 'Content-Type: application/json' "
        f"-d '{{\"username\":\"root\",\"password\":\"{CAM_PASSWORD}\"}}'",
        shell=True, capture_output=True, timeout=5,
    )
    # Send command
    result = subprocess.run(
        f"curl -s -b {cookie_file} 'http://{ip}/x/json-imp.cgi' "
        f"-H 'Content-Type: application/json' "
        f"-d '{{\"cmd\":\"{cmd}\",\"val\":{val_json}}}'",
        shell=True, capture_output=True, text=True, timeout=5,
    )
    return result.stdout


def leds_on_hold(ip, hold_seconds=30):
    """Turn on cam1's IR LEDs via Thingino's web API (no SSH).

    Uses /x/json-imp.cgi commands to disable daynight and switch on both
    850nm + 940nm LEDs. No subprocess.Popen / sleep — caller invokes
    leds_off() when done. Returns a sentinel object for caller (so the
    existing led_proc.kill() pattern still works).

    Why web API: SSH on overloaded cams times out unpredictably. A
    timed-out off-SSH leaves LEDs stuck on, then the next direction's
    capture sees ON+ON frames → delta=0/garbage. Web API is HTTP POST
    via curl (subprocess) — much lighter, no sshd login.
    """
    imp_cmd(ip, "daynight", '"night"')
    on_result = imp_cmd(ip, "ir850", 1)
    on_result2 = imp_cmd(ip, "ir940", 1)
    if '"success"' in on_result and '"success"' in on_result2:
        print(f"  LEDs verified ON via web API (ir850, ir940)")
    else:
        print(f"  WARNING: LED on returned: 850={on_result} 940={on_result2}")

    # Verify via heartbeat (state should reflect commanded values within ~1s)
    time.sleep(1)
    cookie_file = f"/tmp/cam_cookie_{ip}.txt"
    hb = subprocess.run(
        f"curl -s -b {cookie_file} 'http://{ip}/x/json-heartbeat-slow.cgi' "
        f"-H 'Accept: application/json'",
        shell=True, capture_output=True, text=True, timeout=5,
    ).stdout
    ir850 = "1" if '"ir850_state":1' in hb else "0"
    ir940 = "1" if '"ir940_state":1' in hb else "0"
    if ir850 == "1" and ir940 == "1":
        print(f"  LEDs heartbeat-confirmed ON (ir850={ir850} ir940={ir940})")
    else:
        print(f"  WARNING: heartbeat shows ir850={ir850} ir940={ir940} (commands sent)")

    # Return a dummy object with .kill()/.wait() so callers (which expect a
    # subprocess.Popen) keep working unchanged.
    class _Sentinel:
        def kill(self): pass
        def wait(self, *a, **kw): pass
        def poll(self): return 0
    return _Sentinel()


def leds_off(ip):
    """Turn off both IR LEDs via Thingino's web API (no SSH)."""
    r1 = imp_cmd(ip, "ir850", 0)
    r2 = imp_cmd(ip, "ir940", 0)
    if not ('"success"' in r1 and '"success"' in r2):
        print(f"  WARNING: LED off returned: 850={r1} 940={r2}")


def grab_frames(cap, n=5):
    """Read n frames to flush buffer, return the last one."""
    frame = None
    for _ in range(n):
        ret, frame = cap.read()
    return frame if ret else None


def calibrate_pair(rx_name, tx_name, interactive=True):
    """Calibrate one direction: find TX pixel location as seen by RX camera.

    Returns dict with calibration data or None on failure.
    """
    rx_ip = CAMERAS[rx_name]
    tx_ip = CAMERAS[tx_name]
    rtsp_url = f"rtsp://thingino:thingino@{rx_ip}:554/ch1"

    print(f"\n{'='*50}")
    print(f"  Calibrating: {rx_name} sees {tx_name}")
    print(f"{'='*50}\n")

    # Step 1: Turn on LEDs FIRST, then open stream (avoids stale RTSP buffer)
    print("Step 1: Turning on transmitter LEDs (850nm + 940nm)...")
    led_proc = leds_on_hold(tx_ip, hold_seconds=30)

    if interactive:
        input("  LEDs should be ON now. Press Enter to continue...")
    else:
        time.sleep(3)

    # Step 2: Open fresh RTSP stream and capture LED-on frame
    print("Step 2: Capturing LED-on frame (fresh stream, no buffer)...")
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print(f"  FAILED: Cannot open RTSP stream {rtsp_url}")
        led_proc.kill()
        leds_off(tx_ip)
        return None
    time.sleep(1)
    frame_on = grab_frames(cap, n=10)
    cap.release()
    if frame_on is None:
        print("  FAILED: Cannot grab frame")
        led_proc.kill()
        leds_off(tx_ip)
        return None
    print("  OK")

    # Step 3: Turn off LEDs, wait, then capture LED-off frame
    led_proc.kill()
    led_proc.wait()
    leds_off(tx_ip)
    print("Step 3: LEDs off, capturing LED-off frame...")
    time.sleep(2)

    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print(f"  FAILED: Cannot open RTSP stream {rtsp_url}")
        return None
    time.sleep(1)
    frame_off = grab_frames(cap, n=10)
    cap.release()
    if frame_off is None:
        print("  FAILED: Cannot grab frame")
        return None
    print("  OK")

    # Step 4: Diff and find peak
    print("Step 4: Finding transmitter...")
    gray_off = cv2.cvtColor(frame_off, cv2.COLOR_BGR2GRAY).astype(np.int16)
    gray_on = cv2.cvtColor(frame_on, cv2.COLOR_BGR2GRAY).astype(np.int16)
    # Only look for positive changes (LED ON = brighter, not OSD/scene changes)
    diff = (gray_on - gray_off)
    diff_pos = np.clip(diff, 0, 255).astype(np.uint8)
    diff_abs = np.abs(diff).astype(np.uint8)

    # Mask out OSD text regions (timestamps in corners)
    # Top-left and top-right OSD: mask top 30 pixels and bottom 30 pixels
    h, w = diff_pos.shape
    diff_pos[:30, :] = 0          # top strip (timestamps)
    diff_pos[h-30:, :] = 0        # bottom strip
    diff_pos[:, :10] = 0          # left edge
    diff_pos[:, w-10:] = 0        # right edge

    diff_blur = cv2.GaussianBlur(diff_pos, (11, 11), 0)
    peak_blurred = int(diff_blur.max())
    max_pos = np.unravel_index(np.argmax(diff_blur), diff_blur.shape)
    tx_y, tx_x = int(max_pos[0]), int(max_pos[1])

    off_val = int(gray_off[max_pos[0], max_pos[1]])
    on_val = int(gray_on[max_pos[0], max_pos[1]])
    delta = on_val - off_val

    print(f"  Peak pixel: ({tx_x}, {tx_y})")
    print(f"  OFF={off_val}, ON={on_val}, delta={delta}")
    print(f"  Peak diff (blurred): {peak_blurred}")

    if peak_blurred < 5:
        print("  FAILED: No transmitter detected (peak too low)")
        return None

    # Step 5: Generate calibration images
    print("Step 5: Saving calibration images...")
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = f"{rx_name}_sees_{tx_name}"

    # Box image
    thresh_val = int(peak_blurred * 0.5)
    _, thresh = cv2.threshold(diff_blur, thresh_val, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    output = frame_on.copy()
    cv2.drawMarker(output, (tx_x, tx_y), (0, 0, 255), cv2.MARKER_CROSS, 15, 2)
    if contours:
        sig = [c for c in contours if cv2.contourArea(c) > 10]
        if sig:
            merged = np.vstack(sig)
            bx, by, bw, bh = cv2.boundingRect(merged)
            pad = 15
            bx, by = max(0, bx - pad), max(0, by - pad)
            bw = min(output.shape[1] - bx, bw + 2 * pad)
            bh = min(output.shape[0] - by, bh + 2 * pad)
            cv2.rectangle(output, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2)
    cv2.putText(output, f"TX: {tx_name} ({tx_x},{tx_y}) d={delta}",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    box_path = os.path.join(PHOTOS_DIR, f"cal_{prefix}_box_{ts}.png")
    cv2.imwrite(box_path, output)

    # Diff heatmap
    diff_color = cv2.applyColorMap((diff_abs * 4).clip(0, 255).astype(np.uint8), cv2.COLORMAP_JET)
    cv2.drawMarker(diff_color, (tx_x, tx_y), (255, 255, 255), cv2.MARKER_CROSS, 15, 2)
    diff_path = os.path.join(PHOTOS_DIR, f"cal_{prefix}_diff_{ts}.png")
    cv2.imwrite(diff_path, diff_color)

    # Zoomed view
    scale = 8
    roi_sz = 25
    ry1, ry2 = max(0, tx_y - roi_sz), min(frame_on.shape[0], tx_y + roi_sz)
    rx1, rx2 = max(0, tx_x - roi_sz), min(frame_on.shape[1], tx_x + roi_sz)
    crop_off = cv2.resize(frame_off[ry1:ry2, rx1:rx2], None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    crop_on = cv2.resize(frame_on[ry1:ry2, rx1:rx2], None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    crop_diff = cv2.resize(diff_color[ry1:ry2, rx1:rx2], None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    for img, lbl in [(crop_off, "OFF"), (crop_on, "ON"), (crop_diff, "DIFF")]:
        cv2.putText(img, lbl, (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    zoom_path = os.path.join(PHOTOS_DIR, f"cal_{prefix}_zoom_{ts}.png")
    cv2.imwrite(zoom_path, np.hstack([crop_off, crop_on, crop_diff]))

    print(f"  Saved: {box_path}")
    print(f"  Saved: {diff_path}")
    print(f"  Saved: {zoom_path}")

    h, w = frame_on.shape[:2]

    return {
        "rx_cam": rx_name,
        "tx_cam": tx_name,
        "tx_pixel": [tx_x, tx_y],
        "frame_size": [w, h],
        "off_value": off_val,
        "on_value": on_val,
        "delta": delta,
        "peak_blurred": peak_blurred,
        "recommended_rate_ms": pick_initial_rate_ms(delta),
        "timestamp": ts,
        "images": {
            "box": box_path,
            "diff": diff_path,
            "zoom": zoom_path,
        },
    }


def _panel_to_image(panel_bytes: bytes, label: str, scale: int = 32) -> np.ndarray:
    """Render one 8×8 4-bit panel as a labeled grayscale image (256×256 default)."""
    grid = np.array(unpack_cal_visual_panel(panel_bytes), dtype=np.uint8) * 17
    upscaled = cv2.resize(grid, (8 * scale, 8 * scale),
                          interpolation=cv2.INTER_NEAREST)
    bgr = cv2.cvtColor(upscaled, cv2.COLOR_GRAY2BGR)
    cv2.putText(bgr, label, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 255), 2, cv2.LINE_AA)
    return bgr


def render_cal_visual_zoom(zoom_4bit: bytes, peak_x: int, peak_y: int,
                            delta: int, brightness: int, output_path: str) -> None:
    """Render the 8-byte 4×4 grid-delta zoom as a 256×256 PNG with colormap.

    cam1 ships a 4×4 sub-area of its 20×12 brightness-grid deltas around the
    peak block (peak at cell (1,1)). Each cell is a per-block delta clamped
    to 0-15 (4-bit). The render upscales 64× nearest-neighbour and JET-
    colormaps. A center crosshair marks the (peak_x, peak_y) source-pixel.
    """
    if len(zoom_4bit) != CAL_VISUAL_PANEL_BYTES:
        raise ValueError(
            f"zoom_4bit must be {CAL_VISUAL_PANEL_BYTES} bytes "
            f"(4×4 4-bit), got {len(zoom_4bit)}"
        )
    grid = np.array(unpack_cal_visual_panel(zoom_4bit), dtype=np.uint8) * 17
    upscaled = cv2.resize(grid, (256, 256), interpolation=cv2.INTER_NEAREST)
    colored = cv2.applyColorMap(upscaled, cv2.COLORMAP_JET)
    cv2.drawMarker(colored, (128, 128), (255, 255, 255),
                   cv2.MARKER_CROSS, 24, 2)
    cv2.putText(colored, f"({peak_x},{peak_y}) d={delta} b={brightness}",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                cv2.LINE_AA)
    cv2.putText(colored, "OVER-LINK 4x4 (8B)",
                (8, 248), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA)
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    cv2.imwrite(output_path, colored)


def cal_over_light(cam2_host: str = "dacam2", symbol_ms: int = 160,
                   bicall_timeout: float = 600.0) -> dict | None:
    """Run cam1-sees-cam2 calibration entirely over the IR link.

    Spawns `irlink connect` on cam2, sends `bicall`, captures stdout for:
      - PEER-CAL line (cam1's coords + delta, from CAL_DONE)
      - APP_CHUNK frames carrying a fragmented APP_CAL_VISUAL
      - BICAL: bidirectional cal complete marker

    Reassembles the 4-chunk CAL_VISUAL, renders the 8×8 zoom to PNG, and
    returns a calibration entry compatible with the existing schema (less
    the box/diff images, which aren't available without RTSP-to-cam1).
    """
    cam2_pixel = get_tx_position("cam2", "cam1")
    if cam2_pixel is None:
        print("  ERROR: no cam2_sees_cam1 calibration. Run direction 1 first.")
        return None

    cmd = (f"/opt/bin/irlink connect --pixel {cam2_pixel[0]},{cam2_pixel[1]} "
           f"--speed {symbol_ms}")
    print(f"  driving cam2: {cmd}")
    proc = subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", cam2_host, cmd],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )

    sent_bicall = False
    chunks_by_seq: dict[int, bytes] = {}
    expected_total: int | None = None
    expected_msg_id: int | None = None
    cal_visual: dict | None = None
    bicall_complete = False
    sent_quit = False
    deadline = time.monotonic() + bicall_timeout

    try:
        for line in proc.stdout:
            if time.monotonic() > deadline:
                print("  TIMEOUT during bicall + CAL_VISUAL collection")
                break
            line = line.rstrip()
            sys.stdout.write(line + "\n")
            sys.stdout.flush()

            if not sent_bicall and "PROTO: connected!" in line:
                proc.stdin.write("bicall\n")
                proc.stdin.flush()
                sent_bicall = True

            if line.startswith("MSG-HEX:"):
                hex_part = line[len("MSG-HEX:"):].strip()
                try:
                    payload = bytes.fromhex(hex_part)
                    msg = app.unpack(payload)
                except Exception as e:
                    print(f"  MSG-HEX parse error: {e}")
                    continue
                if msg.type == APP_CAL_VISUAL:
                    # Unfragmented (would only happen if irlink supports
                    # >16-byte single-frame DATA at this rate). Capture and stop.
                    cal_visual = dict(msg.fields)
                elif msg.type == APP_CHUNK:
                    seq = msg.fields["seq"]
                    msg_id = msg.fields["msg_id"]
                    total = msg.fields["total"]
                    if expected_msg_id is None:
                        expected_msg_id = msg_id
                        expected_total = total
                    if msg_id == expected_msg_id:
                        chunks_by_seq[seq] = msg.fields["data"]
                        if expected_total and len(chunks_by_seq) == expected_total:
                            try:
                                raw = [pack_chunk(expected_msg_id, s,
                                                   expected_total,
                                                   chunks_by_seq[s])
                                       for s in range(expected_total)]
                                app_type, body = reassemble(raw)
                                if app_type == APP_CAL_VISUAL:
                                    full = app.unpack(bytes([app_type]) + body)
                                    cal_visual = dict(full.fields)
                                else:
                                    print(f"  reassembled to 0x{app_type:02x}, "
                                          f"not CAL_VISUAL")
                            except Exception as e:
                                print(f"  reassemble failed: {e}")

            if "BICAL: bidirectional cal complete" in line:
                bicall_complete = True

            if (bicall_complete and cal_visual is not None and not sent_quit):
                proc.stdin.write("quit\n")
                proc.stdin.flush()
                sent_quit = True

        if not sent_quit:
            try:
                proc.stdin.write("quit\n")
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
        proc.wait(timeout=10)
    except (KeyboardInterrupt, subprocess.TimeoutExpired):
        proc.kill()

    if cal_visual is None:
        print("  ERROR: never received complete CAL_VISUAL")
        return None

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    zoom_path = os.path.join(PHOTOS_DIR,
                             f"cal_cam1_sees_cam2_zoom_{ts}.png")
    render_cal_visual_zoom(
        cal_visual["zoom_4bit"],
        cal_visual["x"], cal_visual["y"],
        cal_visual["delta"], cal_visual["brightness"],
        zoom_path,
    )
    print(f"  Saved over-link zoom: {zoom_path}")

    return {
        "rx_cam": "cam1",
        "tx_cam": "cam2",
        "tx_pixel": [cal_visual["x"], cal_visual["y"]],
        "frame_size": [640, 360],
        "off_value": None,
        "on_value": cal_visual["brightness"],
        "delta": cal_visual["delta"],
        "peak_blurred": cal_visual["delta"],
        "recommended_rate_ms": pick_initial_rate_ms(cal_visual["delta"]),
        "timestamp": ts,
        "source": "over_light",
        "images": {"zoom": zoom_path},
    }


def load_calibration():
    """Load saved calibration from file."""
    if os.path.exists(CAL_FILE):
        with open(CAL_FILE) as f:
            return json.load(f)
    return {}


def save_calibration(cal_data):
    """Save calibration to file."""
    with open(CAL_FILE, "w") as f:
        json.dump(cal_data, f, indent=2)
    print(f"\nCalibration saved to {CAL_FILE}")
    write_html(cal_data)
    print(f"HTML viewer updated: {HTML_FILE}")


def _fmt_ts(ts):
    """Format '20260421_165801' → '2026-04-21 16:58:01'."""
    try:
        return datetime.strptime(ts, "%Y%m%d_%H%M%S").strftime("%Y-%m-%d %H:%M:%S")
    except (ValueError, TypeError):
        return ts or "unknown"


def write_html(cal_data):
    """Regenerate photos/calibration.html from calibration data."""
    generated = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    rows = []
    sections = []
    for key, entry in cal_data.items():
        rx, tx = entry["rx_cam"], entry["tx_cam"]
        px, py = entry["tx_pixel"]
        delta = entry["delta"]
        off_v = entry.get("off_value")
        on_v = entry.get("on_value")
        ts_human = _fmt_ts(entry.get("timestamp"))
        images = entry.get("images", {})
        zoom = os.path.basename(images["zoom"]) if images.get("zoom") else None
        box = os.path.basename(images["box"]) if images.get("box") else None
        diff = os.path.basename(images["diff"]) if images.get("diff") else None
        is_over_link = entry.get("source") == "over_light"

        rows.append(
            f'<tr><td>{rx.upper()} sees {tx.upper()}'
            f'{" <span style=\"color:#fa0;font-size:12px\">[over-link]</span>" if is_over_link else ""}</td>'
            f'<td style="color:#0ff">({px}, {py})</td>'
            f'<td style="color:#0f0">+{delta}</td>'
            f'<td>{"-" if off_v is None else off_v}</td>'
            f'<td>{"-" if on_v is None else on_v}</td>'
            f'<td style="color:#aaa">{ts_human}</td></tr>'
        )

        # Section header
        title_suffix = (
            ' <span style="color:#fa0;font-size:14px">[over-link, low-res]</span>'
            if is_over_link else ""
        )
        section_parts = [f'<div class="pair">',
                         f'<h2>{rx.upper()} sees {tx.upper()}{title_suffix}</h2>',
                         f'<p class="summary">TX at <span>({px}, {py})</span> | '
                         f'Delta: <span>+{delta}</span> | '
                         f'<span style="color:#aaa;font-size:14px">{ts_human}</span></p>']

        if box or diff:
            section_parts.append('<h3>Analysis</h3><div class="row">')
            if box:
                section_parts.append(
                    f'<figure><img src="{box}" width="320">'
                    f'<figcaption>Peak pixel marked</figcaption></figure>'
                )
            if diff:
                section_parts.append(
                    f'<figure><img src="{diff}" width="320">'
                    f'<figcaption>Diff heatmap</figcaption></figure>'
                )
            section_parts.append('</div>')

        if zoom:
            zoom_caption = (
                "OFF | ON | DIFF — 8×8 4-bit per panel, ~6 min over IR link"
                if is_over_link
                else "OFF | ON | DIFF"
            )
            section_parts.append(
                f'<h3>{"Over-link Triptych (8×8)" if is_over_link else "Zoomed Peak (8x)"}</h3>'
                f'<div class="row"><figure>'
                f'<img src="{zoom}" width="100%">'
                f'<figcaption>{zoom_caption}</figcaption></figure></div>'
            )

        if is_over_link:
            section_parts.append(
                '<p style="color:#aaa;font-size:13px">'
                'Full-frame box/diff not available — cam1 has no laptop link, '
                'only the IR channel. Visual confirmation is the 8×8 zoom above.'
                '</p>'
            )

        section_parts.append('</div>')
        sections.append('\n'.join(section_parts))

    if not rows:
        rows.append('<tr><td colspan="6" style="color:#aaa">No calibration data</td></tr>')

    html = f"""<!DOCTYPE html>
<html>
<head><title>IR Calibration</title>
<style>
body{{background:#1a1a2e;color:#eee;font-family:sans-serif;margin:20px}}
h1{{color:#e94560;margin-bottom:5px}} h2{{color:#fff;background:#e94560;display:inline-block;padding:5px 15px;border-radius:4px}}
h3{{color:#aaa;margin-top:20px}}
.pair{{margin:30px 0;padding:20px;background:#16213e;border-radius:8px}}
.summary{{font-size:18px;margin:10px 0}} .summary span{{color:#0ff;font-weight:bold}}
.row{{display:flex;gap:10px;flex-wrap:wrap;margin:15px 0}}
.row figure{{margin:0;text-align:center}}
.row figcaption{{color:#aaa;font-size:13px;margin-top:5px}}
.row img{{border:2px solid #333;border-radius:4px}}
table{{border-collapse:collapse;margin:15px 0}}
td,th{{padding:8px 16px;border:1px solid #333;text-align:left}}
th{{background:#0f3460}}
.generated{{color:#888;font-size:13px;margin:0 0 20px 0}}
</style></head><body>
<h1>IR Calibration Results</h1>
<p class="generated">Page generated: <span style="color:#0ff">{generated}</span></p>
<table>
<tr><th>Direction</th><th>TX Pixel</th><th>Delta</th><th>OFF</th><th>ON</th><th>Calibrated</th></tr>
{chr(10).join(rows)}
</table>

{chr(10).join(sections)}

</body></html>
"""
    os.makedirs(PHOTOS_DIR, exist_ok=True)
    with open(HTML_FILE, "w") as f:
        f.write(html)


def show_calibration():
    """Display current calibration."""
    cal = load_calibration()
    if not cal:
        print("No calibration data found. Run calibration first.")
        return

    for key, entry in cal.items():
        print(f"\n{key}:")
        print(f"  TX pixel: ({entry['tx_pixel'][0]}, {entry['tx_pixel'][1]})")
        print(f"  Delta: {entry['delta']} (OFF={entry['off_value']}, ON={entry['on_value']})")
        print(f"  Peak (blurred): {entry['peak_blurred']}")
        if "recommended_rate_ms" in entry:
            print(f"  Recommended rate: {entry['recommended_rate_ms']}ms/sym")
        print(f"  Calibrated: {entry['timestamp']}")


def get_tx_position(rx_name, tx_name):
    """Get calibrated TX pixel position for a given RX/TX pair."""
    cal = load_calibration()
    key = f"{rx_name}_sees_{tx_name}"
    if key not in cal:
        return None
    entry = cal[key]
    return entry["tx_pixel"][0], entry["tx_pixel"][1]


def get_calibration_entry(rx_name, tx_name):
    """Get full calibration entry for a given RX/TX pair."""
    cal = load_calibration()
    key = f"{rx_name}_sees_{tx_name}"
    return cal.get(key)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IR transmitter calibration")
    parser.add_argument("--rx", help="Receiver camera (cam1 or cam2)")
    parser.add_argument("--tx", help="Transmitter camera (cam1 or cam2)")
    parser.add_argument("--show", action="store_true", help="Show current calibration")
    parser.add_argument("--no-interactive", action="store_true", help="Skip interactive prompts")
    parser.add_argument("--over-light", action="store_true",
                        help="Run cam1-sees-cam2 over the IR link (no WiFi to cam1). "
                             "Direction 1 (cam2-sees-cam1) still uses RTSP. cam1's "
                             "visual is a low-res 8×8 zoom shipped over the link.")
    parser.add_argument("--symbol-ms", type=int, default=160,
                        help="Symbol rate for the over-light cal (default: 160ms)")
    parser.add_argument("--skip-setup", action="store_true",
                        help="Skip cam_setup.sh — use only when cam state is "
                             "known good (saves ~60s on slow WiFi).")
    args = parser.parse_args()

    if args.show:
        show_calibration()
        sys.exit(0)

    cal = load_calibration()

    # Pre-flight: put BOTH cams into a known IR-comm-ready state. Hits
    # /x/setup.cgi in parallel — ~3s wall vs ~30-60s for the old
    # ./host/cam_setup.sh SSH path. Same operations, same definitions of
    # "ok" (kills daynightd, night+mono, LEDs off, ae_freeze=0, ircut open,
    # verifies grid + prudynt + irlink binary). Runs in BOTH legacy
    # paths (--over-light and the default frame-diff path); the old code
    # only ran setup in --over-light, leaving the default path vulnerable
    # to stuck LEDs / stuck ae_freeze from a prior `kill -9 irlink`.
    if args.skip_setup:
        print("\n=== Skipping setup.cgi pre-flight (--skip-setup) ===")
    else:
        print("\n=== setup.cgi — verifying camera state (parallel, both cams) ===")
        from host.cam_status import CamStatusClient
        clients = {name: CamStatusClient(ip) for name, ip in CAMERAS.items()}
        results = {}
        threads = []
        def _run(name, client):
            results[name] = client.run_setup(timeout=20.0)
        for name, client in clients.items():
            t = threading.Thread(target=_run, args=(name, client), daemon=True)
            t.start()
            threads.append(t)
        for t in threads:
            t.join(timeout=25.0)
        failed = []
        for name, r in results.items():
            if r is None:
                print(f"  {name}: NO RESPONSE (uhttpd or setup.cgi unreachable)")
                failed.append(name)
                continue
            steps = r.get("steps", [])
            ok_count = sum(1 for s in steps if s.get("ok"))
            print(f"  {name}: ok={r.get('ok')} ({ok_count}/{len(steps)} steps)")
            for s in steps:
                if not s.get("ok"):
                    print(f"      ✗ {s.get('name','?')}: {s.get('detail','')}")
            if not r.get("ok"):
                failed.append(name)
        if failed:
            print(f"\n  ERROR: setup failed on {','.join(failed)} — "
                  "fix and retry, or run ./host/cam_setup.sh manually, "
                  "or pass --skip-setup if state is known good.")
            sys.exit(1)

    if args.over_light:
        cam1_ip = CAMERAS["cam1"]

        # Direction 1 needs cam1's autostarted daemon-listen out of the way
        # (it owns LED control and would race with `light ir850 off` SSH calls).
        print("\n=== Stopping cam1 daemon-listen for direction 1 ===")
        try:
            subprocess.run(
                ["ssh", "-o", "ConnectTimeout=30", "-o", "BatchMode=yes",
                 f"root@{cam1_ip}",
                 "killall -9 irlink 2>/dev/null; sleep 1; "
                 "echo 0 > /run/prudynt/ae_freeze 2>/dev/null"],
                timeout=45,
            )
        except subprocess.TimeoutExpired:
            print("  WARNING: cam1 daemon-stop SSH timed out — cam1 may "
                  "still be running irlink. Direction 1 may fight with it.")

        print("\n=== Direction 1: cam2 sees cam1 (RTSP frame-diff) ===")
        result1 = calibrate_pair("cam2", "cam1",
                                 interactive=not args.no_interactive)
        if result1:
            cal["cam2_sees_cam1"] = result1
            save_calibration(cal)  # save direction-1 even if direction-2 fails

        # Direction 2 needs cam1's daemon-listen RUNNING so cam2's `irlink
        # connect` can reach it. The --pixel arg here is cam1's view of cam2's
        # TX (where cam1 watches for incoming DATA frames) — pull from the
        # last-known cam1_sees_cam2 entry, or fall back to cam1's local
        # /opt/etc/calibration.json. Direction 2 then re-discovers cam2's
        # exact TX pixel via on-camera grid scan, regardless of this hint.
        print("\n=== Restarting cam1 daemon-listen for direction 2 ===")
        prev_cam1_sees_cam2 = cal.get("cam1_sees_cam2", {}).get("tx_pixel")
        if prev_cam1_sees_cam2:
            coords = f"{prev_cam1_sees_cam2[0]},{prev_cam1_sees_cam2[1]}"
        else:
            # Read cam1's persisted coords. This is what rc.local autostart uses.
            from host.cam_status import CamStatusClient  # lazy: only here
            saved = CamStatusClient(cam1_ip).get_saved_cal()
            tx = (saved or {}).get("tx_pixel")
            if isinstance(tx, list) and len(tx) == 2:
                coords = f"{int(tx[0])},{int(tx[1])}"
            else:
                coords = "320,180"  # last-resort frame center
        print(f"  using cam1 daemon-listen --pixel {coords}")
        try:
            subprocess.run(
                ["ssh", "-o", "ConnectTimeout=30", "-o", "BatchMode=yes",
                 f"root@{cam1_ip}",
                 f"nohup /opt/bin/irlink daemon-listen --pixel {coords} "
                 f"--speed {args.symbol_ms} >/var/log/irlink-boot.log 2>&1 &"],
                timeout=45,
            )
        except subprocess.TimeoutExpired:
            print(f"  WARNING: cam1 daemon-listen restart SSH timed out — "
                  f"cam1 may be busy. Continuing; if direction 2 fails, "
                  f"verify cam1 is reachable.")
        time.sleep(4)  # let it bind

        print("\n=== Direction 2: cam1 sees cam2 (over IR link) ===")
        result2 = cal_over_light(symbol_ms=args.symbol_ms)
        if result2:
            cal["cam1_sees_cam2"] = result2
            save_calibration(cal)
        else:
            print("\n  Direction 2 failed — direction 1 result still saved.")
    elif args.rx and args.tx:
        # Calibrate one direction (legacy single-direction mode)
        result = calibrate_pair(args.rx, args.tx, interactive=not args.no_interactive)
        if result:
            key = f"{args.rx}_sees_{args.tx}"
            cal[key] = result
            save_calibration(cal)
            print(f"\n  TX position: ({result['tx_pixel'][0]}, {result['tx_pixel'][1]}), delta: {result['delta']}")
    else:
        # Calibrate both directions
        for rx, tx in [("cam2", "cam1"), ("cam1", "cam2")]:
            result = calibrate_pair(rx, tx, interactive=not args.no_interactive)
            if result:
                key = f"{rx}_sees_{tx}"
                cal[key] = result
                print(f"\n  TX position: ({result['tx_pixel'][0]}, {result['tx_pixel'][1]}), delta: {result['delta']}")

        save_calibration(cal)

    print("\n" + "=" * 50)
    print("  Calibration Summary")
    print("=" * 50)
    show_calibration()
