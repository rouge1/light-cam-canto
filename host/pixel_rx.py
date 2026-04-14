#!/usr/bin/env python3
"""Host-side pixel-level IR receiver using RTSP stream.

Instead of using the coarse 20x12 brightness grid, this reads the RTSP stream
directly and tracks a small ROI around the transmitter's exact pixel location.
Peak pixel delta of 250 vs grid delta of 6-15 at 10-20 feet.

Usage:
    python -m host.pixel_rx --cam cam2 --tx-pos 424,163 --roi 15
    python -m host.pixel_rx --cam cam2 --auto-detect cam1

The --auto-detect option runs find_transmitter to locate the TX automatically.
"""
import cv2
import numpy as np
import subprocess
import sys
import time
import argparse
from protocol.manchester import manchester_decode
from protocol.frame import parse_frame_bits, SYNC_WORD

CAMERAS = {
    "cam1": "192.168.50.110",
    "cam2": "192.168.50.141",
}


def ssh_cmd(ip, cmd):
    subprocess.run(["ssh", f"root@{ip}", cmd], capture_output=True, timeout=10)


def auto_detect_tx(rx_name, tx_name, roi_size=15):
    """Toggle the TX LEDs and find the peak pixel location."""
    rx_ip = CAMERAS[rx_name]
    tx_ip = CAMERAS[tx_name]
    rtsp_url = f"rtsp://thingino:thingino@{rx_ip}:554/ch1"

    # Ensure LEDs off
    ssh_cmd(tx_ip, "gpio clear 47; gpio clear 49")
    time.sleep(1)

    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print(f"Failed to open RTSP: {rtsp_url}")
        sys.exit(1)

    # Grab LED-off frame
    for _ in range(5):
        ret, frame = cap.read()
    gray_off = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int16)

    # Turn on LEDs
    ssh_cmd(tx_ip, "gpio set 47; gpio set 49")
    time.sleep(2)

    # Grab LED-on frame
    for _ in range(5):
        ret, frame = cap.read()
    gray_on = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int16)

    # Turn off LEDs
    ssh_cmd(tx_ip, "gpio clear 47; gpio clear 49")
    cap.release()

    # Find peak pixel
    diff = (gray_on - gray_off).astype(np.float64)
    diff_blur = cv2.GaussianBlur(np.abs(diff).astype(np.uint8), (5, 5), 0)
    max_pos = np.unravel_index(np.argmax(diff_blur), diff_blur.shape)
    peak_val = diff_blur[max_pos]

    tx_x, tx_y = int(max_pos[1]), int(max_pos[0])
    print(f"TX detected at ({tx_x}, {tx_y}), peak delta: {peak_val}")
    return tx_x, tx_y


def run_rx(rx_name, tx_x, tx_y, roi_size=15, symbol_ms=160, settle_ms=400, min_delta=10):
    """Monitor RTSP stream and decode Manchester frames from pixel brightness."""
    rx_ip = CAMERAS[rx_name]
    rtsp_url = f"rtsp://thingino:thingino@{rx_ip}:554/ch1"

    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print(f"Failed to open RTSP: {rtsp_url}")
        sys.exit(1)

    # Get frame dimensions
    ret, frame = cap.read()
    h, w = frame.shape[:2]

    # ROI bounds
    half = roi_size // 2
    y1 = max(0, tx_y - half)
    y2 = min(h, tx_y + half + 1)
    x1 = max(0, tx_x - half)
    x2 = min(w, tx_x + half + 1)

    print(f"Receiver: {rx_name} ({rx_ip})")
    print(f"TX pixel: ({tx_x}, {tx_y}), ROI: ({x1},{y1})-({x2},{y2}) = {x2-x1}x{y2-y1} pixels")
    print(f"Symbol: {symbol_ms}ms, Settle: {settle_ms}ms, Min delta: {min_delta}")
    print(f"Listening...\n")

    # State machine
    IDLE = 0
    ACTIVE = 1
    state = IDLE

    # Collect samples as (timestamp_ms, brightness)
    samples = []
    baseline = None
    active_count = 0
    last_active_time = 0

    frame_count = 0
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        now_ms = int((time.time() - start_time) * 1000)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Sample the ROI — mean brightness of the small pixel region
        roi = gray[y1:y2, x1:x2]
        brightness = int(np.mean(roi))

        frame_count += 1

        # Set initial baseline
        if baseline is None:
            baseline = brightness
            print(f"Baseline: {baseline}")
            continue

        diff = abs(brightness - baseline)

        if state == IDLE:
            if diff >= min_delta:
                active_count += 1
                if active_count >= 2:
                    print(f"RX: activity detected (baseline={baseline}, now={brightness}, delta={diff})")
                    state = ACTIVE
                    samples = []
                    # Add synthetic baseline sample
                    samples.append((now_ms - 10, baseline))
                    samples.append((now_ms, brightness))
                    last_active_time = now_ms
            else:
                active_count = 0
                # Slowly adapt baseline
                baseline = int(baseline * 0.99 + brightness * 0.01)

        elif state == ACTIVE:
            samples.append((now_ms, brightness))

            if diff >= min_delta // 2:
                last_active_time = now_ms

            # Check for end of TX (silence for settle_ms)
            if now_ms - last_active_time > settle_ms:
                print(f"RX: end of TX ({len(samples)} samples over {samples[-1][0] - samples[0][0]}ms)")

                # Try to decode
                result = decode_samples(samples, symbol_ms, min_delta)
                if result:
                    print(f"\n*** DECODED: {result} ***\n")

                # Reset
                state = IDLE
                active_count = 0
                baseline = brightness
                samples = []


def decode_samples(samples, symbol_ms, min_delta):
    """Decode collected (timestamp, brightness) samples into a message."""
    if len(samples) < 10:
        print(f"  Too few samples ({len(samples)})")
        return None

    # Extract brightness values
    times = [s[0] for s in samples]
    values = [s[1] for s in samples]

    bmin = min(values[1:])  # skip synthetic baseline
    bmax = max(values[1:])
    baseline = values[0]
    delta = bmax - bmin

    if delta < min_delta:
        print(f"  Delta too small ({delta})")
        return None

    # Threshold: midpoint between baseline and peak
    threshold = (baseline + bmax) / 2
    if threshold <= baseline:
        threshold = baseline + delta / 4

    print(f"  {len(samples)} samples, brightness {bmin}-{bmax}, baseline={baseline}, threshold={threshold:.0f}")

    # Find TX start
    tx_start = None
    for i in range(1, len(samples)):
        if abs(values[i] - baseline) >= delta / 3:
            tx_start = max(1, i - 1)
            break

    if tx_start is None:
        print("  No TX start found")
        return None

    t0 = times[tx_start] - symbol_ms // 2

    # Extract symbols by sampling the middle of each symbol window
    symbols = []
    sym = 0
    while True:
        win_start = t0 + symbol_ms * sym + symbol_ms // 4
        win_end = t0 + symbol_ms * sym + symbol_ms * 3 // 4

        vals = [values[i] for i in range(tx_start, len(samples))
                if win_start <= times[i] <= win_end]

        if not vals:
            break

        avg = sum(vals) / len(vals)
        symbols.append(1 if avg >= threshold else 0)
        sym += 1

    print(f"  {len(symbols)} symbols: {''.join(str(s) for s in symbols[:120])}")

    if len(symbols) < 20:
        print("  Too few symbols")
        return None

    # Try decoding with trim offsets (same as irlink.c)
    for trim_s in range(12):
        for trim_e in range(12):
            sc = len(symbols) - trim_s - trim_e
            if sc < 20 or sc % 2 != 0:
                continue

            sub = symbols[trim_s:trim_s + sc]
            bits = manchester_decode(sub)
            if bits is None:
                continue

            msg = parse_frame_bits(bits)
            if msg is not None:
                return msg

    # Try inverted
    inv_symbols = [1 - s for s in symbols]
    for trim_s in range(12):
        for trim_e in range(12):
            sc = len(inv_symbols) - trim_s - trim_e
            if sc < 20 or sc % 2 != 0:
                continue

            sub = inv_symbols[trim_s:trim_s + sc]
            bits = manchester_decode(sub)
            if bits is None:
                continue

            msg = parse_frame_bits(bits)
            if msg is not None:
                return msg

    print("  Decode failed")
    return None


import sys as _sys
# Force unbuffered output
_sys.stdout.reconfigure(line_buffering=True)
_sys.stderr.reconfigure(line_buffering=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pixel-level IR receiver")
    parser.add_argument("--cam", default="cam2", help="Receiver camera (cam1 or cam2)")
    parser.add_argument("--tx-pos", help="TX pixel position as x,y")
    parser.add_argument("--auto-detect", help="Auto-detect TX from this camera name")
    parser.add_argument("--roi", type=int, default=15, help="ROI size in pixels")
    parser.add_argument("--symbol-ms", type=int, default=160, help="Symbol duration in ms")
    parser.add_argument("--settle-ms", type=int, default=400, help="Settle time in ms")
    parser.add_argument("--min-delta", type=int, default=10, help="Min brightness delta")
    args = parser.parse_args()

    if args.auto_detect:
        tx_x, tx_y = auto_detect_tx(args.cam, args.auto_detect, args.roi)
    elif args.tx_pos:
        tx_x, tx_y = [int(v) for v in args.tx_pos.split(",")]
    else:
        # Try loading from calibration file
        from host.cal_procedure import get_tx_position
        # Figure out which cam is TX (the other one)
        tx_name = "cam1" if args.cam == "cam2" else "cam2"
        pos = get_tx_position(args.cam, tx_name)
        if pos:
            tx_x, tx_y = pos
            print(f"Loaded calibration: TX at ({tx_x}, {tx_y})")
        else:
            print("No calibration found. Run: python -m host.cal_procedure")
            print("Or specify --tx-pos x,y or --auto-detect <cam>")
            sys.exit(1)

    run_rx(args.cam, tx_x, tx_y,
           roi_size=args.roi,
           symbol_ms=args.symbol_ms,
           settle_ms=args.settle_ms,
           min_delta=args.min_delta)
