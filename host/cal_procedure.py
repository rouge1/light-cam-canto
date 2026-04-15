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
from datetime import datetime

CAMERAS = {
    "cam1": "192.168.50.110",
    "cam2": "192.168.50.141",
}
CAM_PASSWORD = "password"

PHOTOS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "photos")
CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")

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


def ssh_cmd(ip, cmd):
    """Run a command on the camera via SSH."""
    result = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
         f"root@{ip}", cmd],
        capture_output=True, text=True, timeout=10,
    )
    return result.stdout.strip()


def leds_on_hold(ip, hold_seconds=30):
    """Turn on LEDs and hold them on in a background SSH session.

    Disables daynight, turns on LEDs, verifies, then sleeps to prevent
    prudynt from re-enabling daynight and overriding the GPIO state.
    Returns the Popen process — caller must kill it when done.
    """
    cmd = (
        'echo \'{"daynight":{"enabled":false}}\' | prudyntctl json - >/dev/null 2>&1; '
        'light ir850 on; light ir940 on; '
        'ir850=$(light ir850 read); ir940=$(light ir940 read); '
        'echo "LEDS_ON ir850=$ir850 ir940=$ir940"; '
        f'sleep {hold_seconds}'
    )
    proc = subprocess.Popen(
        ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
         f"root@{ip}", cmd],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    # Read the verification line
    line = proc.stdout.readline().strip()
    if "ir850=1" in line and "ir940=1" in line:
        print(f"  LEDs verified ON ({line})")
    else:
        print(f"  WARNING: LED verify unexpected: {line}")
    return proc


def leds_off(ip):
    """Turn off both IR LEDs in a single SSH session."""
    ssh_cmd(ip, (
        'echo \'{"daynight":{"enabled":false}}\' | prudyntctl json - 2>/dev/null; '
        'light ir850 off; light ir940 off'
    ))


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
        "timestamp": ts,
        "images": {
            "box": box_path,
            "diff": diff_path,
            "zoom": zoom_path,
        },
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
    args = parser.parse_args()

    if args.show:
        show_calibration()
        sys.exit(0)

    cal = load_calibration()

    if args.rx and args.tx:
        # Calibrate one direction
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
