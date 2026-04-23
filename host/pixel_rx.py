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
import json
import os
from datetime import datetime
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


def run_rx(rx_name, tx_x, tx_y, roi_size=15, symbol_ms=160, settle_ms=400, min_delta=10,
           cal_frame_size=None, dump_path=None, decoder="baseline",
           chunk_syms=None, resync_syms=None, rs_n=None, rs_k=None):
    """Monitor RTSP stream and decode Manchester frames from pixel brightness."""
    rx_ip = CAMERAS[rx_name]
    rtsp_url = f"rtsp://thingino:thingino@{rx_ip}:554/ch1"

    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print(f"Failed to open RTSP: {rtsp_url}")
        sys.exit(1)

    # Flush initial frames — first frames are often black (codec init / stale buffer)
    print("Flushing RTSP buffer...")
    for _ in range(15):
        ret, frame = cap.read()
        if not ret:
            continue

    # Get frame dimensions from a warm frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame after flush")
        sys.exit(1)
    h, w = frame.shape[:2]

    # Validate frame size matches calibration
    if cal_frame_size:
        cal_w, cal_h = cal_frame_size
        if w != cal_w or h != cal_h:
            print(f"ERROR: stream is {w}x{h} but calibration was {cal_w}x{cal_h}")
            print("Re-run calibration: python -m host.cal_procedure --no-interactive")
            cap.release()
            sys.exit(1)
        print(f"Frame size {w}x{h} matches calibration")

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
                from protocol.frame import (
                    DEFAULT_CHUNK_SYMS, DEFAULT_RESYNC_SYMS,
                    DEFAULT_RS_N, DEFAULT_RS_K,
                )
                if decoder == "resync":
                    from experiments.resync_decoder import decode_samples_resync
                    result = decode_samples_resync(
                        samples, symbol_ms, min_delta,
                        chunk_syms=chunk_syms or DEFAULT_CHUNK_SYMS,
                        resync_syms=resync_syms or DEFAULT_RESYNC_SYMS,
                    )
                elif decoder == "resync-fec":
                    from experiments.resync_decoder import decode_samples_resync_fec
                    result = decode_samples_resync_fec(
                        samples, symbol_ms, min_delta,
                        chunk_syms=chunk_syms or DEFAULT_CHUNK_SYMS,
                        resync_syms=resync_syms or DEFAULT_RESYNC_SYMS,
                        rs_n=rs_n or DEFAULT_RS_N,
                        rs_k=rs_k or DEFAULT_RS_K,
                    )
                else:
                    result = decode_samples(samples, symbol_ms, min_delta)
                if result:
                    print(f"\n*** DECODED: {result} ***\n")

                # Dump raw samples for offline replay
                if dump_path is not None:
                    record = {
                        "ts": datetime.now().isoformat(timespec="seconds"),
                        "rx_cam": rx_name,
                        "tx_pixel": [tx_x, tx_y],
                        "roi_size": roi_size,
                        "symbol_ms": symbol_ms,
                        "min_delta": min_delta,
                        "baseline_at_start": samples[0][1] if samples else None,
                        "samples": samples,
                        "decoded": result,
                    }
                    with open(dump_path, "a") as f:
                        f.write(json.dumps(record) + "\n")
                    print(f"RX: dumped {len(samples)} samples to {dump_path}")

                # Reset
                state = IDLE
                active_count = 0
                baseline = brightness
                samples = []


def decode_samples(samples, symbol_ms, min_delta):
    """Decode collected (timestamp, brightness) samples into a message.

    Two-pass approach:
    1. DPLL pass: resample to 1ms, find edges, track phase with early-late
       gate using the raw (non-interpolated) sample times to avoid gap artifacts.
    2. Fallback: brute-force t0 search with majority-vote (the method that
       already works at 160ms).
    """
    if len(samples) < 10:
        print(f"  Too few samples ({len(samples)})")
        return None

    times = np.array([s[0] for s in samples], dtype=np.float64)
    values = np.array([s[1] for s in samples], dtype=np.float64)

    bmin = float(values[1:].min())
    bmax = float(values[1:].max())
    delta = bmax - bmin

    if delta < min_delta:
        print(f"  Delta too small ({delta})")
        return None

    threshold = (bmin + bmax) / 2.0
    print(f"  {len(samples)} samples, brightness {bmin:.0f}-{bmax:.0f}, threshold={threshold:.0f}")

    # --- Pass 1: DPLL on raw samples (no interpolation artifacts) ---
    # Work directly with the irregular sample times.
    # For each symbol: find all raw samples in the center window, majority vote.
    # For phase correction: find transitions in raw samples near boundary.

    T = float(symbol_ms)

    # Digitize raw samples
    dig_raw = (values >= threshold).astype(np.int8)

    # Find first rising edge in raw samples
    first_edge_t = None
    for i in range(1, len(dig_raw)):
        if dig_raw[i] == 1 and dig_raw[i-1] == 0:
            first_edge_t = (times[i-1] + times[i]) / 2.0
            break

    if first_edge_t is not None:
        for phase_init_off in [0.5, 0.3, 0.7]:
            for gain in [0.15, 0.25, 0.35]:
                phase = first_edge_t + T * phase_init_off  # center of first symbol
                symbols = []

                while phase < times[-1] - T * 0.3:
                    # Collect raw samples in center 60% of symbol window
                    w_lo = phase - T * 0.3
                    w_hi = phase + T * 0.3

                    mask = (times >= w_lo) & (times <= w_hi)
                    window_vals = values[mask]

                    if len(window_vals) == 0:
                        # No samples in window — use nearest sample
                        idx = np.argmin(np.abs(times - phase))
                        sym_val = 1 if values[idx] >= threshold else 0
                    else:
                        sym_val = 1 if float(window_vals.mean()) >= threshold else 0
                    symbols.append(sym_val)

                    # Phase correction: find transitions near expected boundary
                    boundary = phase + T * 0.5
                    search_lo = boundary - T * 0.4
                    search_hi = boundary + T * 0.4

                    best_edge_t = None
                    best_dist = T * 0.4 + 1

                    for j in range(1, len(times)):
                        if times[j] < search_lo or times[j-1] > search_hi:
                            continue
                        if dig_raw[j] != dig_raw[j-1]:
                            edge_t = (times[j-1] + times[j]) / 2.0
                            dist = abs(edge_t - boundary)
                            if dist < best_dist:
                                best_dist = dist
                                best_edge_t = edge_t

                    if best_edge_t is not None:
                        error = best_edge_t - boundary
                        phase += T + gain * error
                    else:
                        phase += T

                if len(symbols) < 20:
                    continue

                # Try decode
                for syms in [symbols, [1 - s for s in symbols]]:
                    for trim_s in range(16):
                        for trim_e in range(16):
                            sc = len(syms) - trim_s - trim_e
                            if sc < 20 or sc % 2 != 0:
                                continue
                            sub = syms[trim_s:trim_s + sc]
                            bits = manchester_decode(sub)
                            if bits is None:
                                continue
                            msg = parse_frame_bits(bits)
                            if msg is not None:
                                sym_str = ''.join(str(s) for s in symbols[:250])
                                print(f"  DPLL: {len(symbols)} syms, gain={gain}: {sym_str}")
                                return msg

    # --- Pass 2: Brute-force t0 search on interpolated signal ---
    # This is the method that works reliably at 160ms.
    t_start = int(times[0])
    t_end = int(times[-1])
    t_uniform = np.arange(t_start, t_end + 1, dtype=np.float64)
    v_uniform = np.interp(t_uniform, times, values)
    digital = (v_uniform >= threshold).astype(np.int8)
    sig_len = len(digital)

    first_edge = None
    for i in range(1, sig_len):
        if digital[i] == 1 and digital[i-1] == 0:
            first_edge = i
            break

    if first_edge is not None:
        for t0_off in range(-symbol_ms // 2, symbol_ms // 2, symbol_ms // 8):
            t0 = first_edge + t0_off
            symbols = []
            sym = 0
            while True:
                win_start = t0 + int(T * sym + T * 0.15)
                win_end = t0 + int(T * sym + T * 0.85)
                if win_start < 0:
                    sym += 1
                    continue
                if win_end >= sig_len:
                    break
                window = digital[win_start:win_end]
                ones = int(window.sum())
                symbols.append(1 if ones > len(window) // 2 else 0)
                sym += 1

            if len(symbols) < 20:
                continue

            for syms in [symbols, [1 - s for s in symbols]]:
                for trim_s in range(16):
                    for trim_e in range(16):
                        sc = len(syms) - trim_s - trim_e
                        if sc < 20 or sc % 2 != 0:
                            continue
                        sub = syms[trim_s:trim_s + sc]
                        bits = manchester_decode(sub)
                        if bits is None:
                            continue
                        msg = parse_frame_bits(bits)
                        if msg is not None:
                            sym_str = ''.join(str(s) for s in symbols[:200])
                            print(f"  Brute: {len(symbols)} syms (t0_off={t0_off}): {sym_str}")
                            return msg

    if symbols:
        print(f"  {len(symbols)} symbols: {''.join(str(s) for s in symbols[:200])}")
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
    parser.add_argument("--dump-samples", help="Append raw samples per capture as JSONL to this file")
    parser.add_argument("--decoder", default="baseline",
                        choices=["baseline", "resync", "resync-fec"],
                        help="Decoder variant (resync: mid-frame resync markers; "
                             "resync-fec: resync + Reed-Solomon error correction)")
    parser.add_argument("--chunk-syms", type=int, help="resync: data symbols per chunk (default 48)")
    parser.add_argument("--resync-syms", type=int, help="resync: symbols per resync block (default 16)")
    parser.add_argument("--rs-n", type=int, help="resync-fec: RS codeword length (default 15)")
    parser.add_argument("--rs-k", type=int, help="resync-fec: RS message length per block (default 11)")
    args = parser.parse_args()

    if args.auto_detect:
        tx_x, tx_y = auto_detect_tx(args.cam, args.auto_detect, args.roi)
    elif args.tx_pos:
        tx_x, tx_y = [int(v) for v in args.tx_pos.split(",")]
    else:
        # Try loading from calibration file
        from host.cal_procedure import get_calibration_entry
        # Figure out which cam is TX (the other one)
        tx_name = "cam1" if args.cam == "cam2" else "cam2"
        entry = get_calibration_entry(args.cam, tx_name)
        if entry:
            tx_x, tx_y = entry["tx_pixel"]
            cal_frame_size = tuple(entry["frame_size"])
            print(f"Loaded calibration: TX at ({tx_x}, {tx_y}), frame {cal_frame_size[0]}x{cal_frame_size[1]}")
        else:
            print("No calibration found. Run: python -m host.cal_procedure")
            print("Or specify --tx-pos x,y or --auto-detect <cam>")
            sys.exit(1)

    run_rx(args.cam, tx_x, tx_y,
           roi_size=args.roi,
           symbol_ms=args.symbol_ms,
           settle_ms=args.settle_ms,
           min_delta=args.min_delta,
           cal_frame_size=cal_frame_size if 'cal_frame_size' in dir() else None,
           dump_path=args.dump_samples,
           decoder=args.decoder,
           chunk_syms=args.chunk_syms,
           resync_syms=args.resync_syms,
           rs_n=args.rs_n,
           rs_k=args.rs_k)
