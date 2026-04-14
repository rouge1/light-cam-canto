"""
Capture RTSP frame from cam2 and plot vertical column brightness to detect
rolling shutter stripe patterns from a 500Hz LED toggle on cam1.

Usage:
    # First: ssh da-camera1 "rs_toggle_test 5"
    # Then:  python host/stripe_test.py

Saves plot to photos/stripe_test.png
"""

import sys
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Camera config
RTSP_URL = "rtsp://thingino:thingino@192.168.50.141:554/ch0"
COL_X = 320  # Center column of 640-wide frame
NUM_FRAMES = 10  # Grab this many, plot the best few
OUTPUT_PATH = "photos/stripe_test.png"


def grab_frames(url, n):
    """Grab n frames from RTSP, return list of grayscale frames."""
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"ERROR: Cannot open {url}", file=sys.stderr)
        sys.exit(1)

    # Set small buffer to reduce latency
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    frames = []
    for i in range(n):
        ret, frame = cap.read()
        if not ret:
            print(f"WARNING: Failed to read frame {i}", file=sys.stderr)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frames.append(gray)

    cap.release()
    print(f"Captured {len(frames)} frames", file=sys.stderr)
    return frames


def analyze_column(col_data, frame_idx):
    """Analyze a vertical column for stripe patterns. Returns (period_estimate, amplitude)."""
    # FFT to find dominant spatial frequency
    col_centered = col_data - np.mean(col_data)
    fft = np.fft.rfft(col_centered)
    magnitudes = np.abs(fft)

    # Skip DC (index 0) and very low frequencies (index 1-2)
    magnitudes[:3] = 0

    peak_idx = np.argmax(magnitudes)
    peak_freq = peak_idx / len(col_data)  # cycles per row
    period = len(col_data) / peak_idx if peak_idx > 0 else 0
    amplitude = magnitudes[peak_idx]

    return period, amplitude, magnitudes


def find_brightest_column(frame):
    """Find the column with the highest peak brightness (likely the IR spot)."""
    col_maxes = frame.max(axis=0)
    return int(np.argmax(col_maxes))


def main():
    os.makedirs("photos", exist_ok=True)

    print("Grabbing frames from RTSP...", file=sys.stderr)
    frames = grab_frames(RTSP_URL, NUM_FRAMES)
    if not frames:
        print("ERROR: No frames captured", file=sys.stderr)
        sys.exit(1)

    h, w = frames[0].shape
    print(f"Frame size: {w}x{h}", file=sys.stderr)

    # Find the brightest column (IR spot) from the first usable frame
    bright_col = find_brightest_column(frames[-1])
    print(f"Brightest column: {bright_col}", file=sys.stderr)

    # Test multiple columns around the IR spot and the center
    test_cols = sorted(set([
        bright_col,
        max(0, bright_col - 10), min(w - 1, bright_col + 10),
        COL_X,
        w // 2,
    ]))

    # For each column, analyze all frames and find best
    best_result = None
    all_col_results = []

    for col_x in test_cols:
        for i, frame in enumerate(frames):
            col = frame[:, col_x].astype(np.float64)
            period, amplitude, mags = analyze_column(col, i)
            entry = (col_x, i, col, period, amplitude, mags)
            all_col_results.append(entry)
            if best_result is None or amplitude > best_result[4]:
                best_result = entry

        # Report best for this column
        col_best = max(
            [r for r in all_col_results if r[0] == col_x],
            key=lambda r: r[4]
        )
        print(f"  Col {col_x}: best period={col_best[3]:.1f} rows, "
              f"FFT amp={col_best[4]:.0f}", file=sys.stderr)

    # Sort all results by amplitude, take top 4
    all_col_results.sort(key=lambda r: r[4], reverse=True)
    top = all_col_results[:4]

    # Plot top results: column trace + FFT
    n_plot = len(top)
    fig, axes = plt.subplots(n_plot, 2, figsize=(14, 4 * n_plot))
    if n_plot == 1:
        axes = axes.reshape(1, 2)

    # Compute expected stripe period based on frame height
    # At 500Hz, half-period = 1ms. Row readout = frame_period / height
    frame_period_ms = 40.0  # 25fps
    row_time_us = frame_period_ms * 1000 / h
    expected_period = 2000.0 / row_time_us  # 2ms full period / row_time
    expected_freq = 1.0 / expected_period if expected_period > 0 else 0

    print(f"Expected stripe period: {expected_period:.1f} rows "
          f"({row_time_us:.1f} µs/row)", file=sys.stderr)

    for row_idx, (col_x, frame_idx, col, period, amplitude, mags) in enumerate(top):
        # Left: column brightness vs row
        ax_col = axes[row_idx, 0]
        ax_col.plot(col, linewidth=0.5)
        ax_col.set_xlabel("Row")
        ax_col.set_ylabel("Brightness")
        ax_col.set_title(f"Col {col_x}, Frame {frame_idx} "
                         f"(period≈{period:.1f} rows)")
        ax_col.grid(True, alpha=0.3)

        # Right: FFT magnitude spectrum
        ax_fft = axes[row_idx, 1]
        freqs = np.arange(len(mags)) / len(col)
        ax_fft.plot(freqs, mags, linewidth=0.5)
        ax_fft.set_xlabel("Spatial frequency (cycles/row)")
        ax_fft.set_ylabel("Magnitude")
        ax_fft.set_title(f"FFT — peak at {1/period:.4f} cyc/row"
                         if period > 0 else "FFT")
        ax_fft.set_xlim(0, 0.25)
        ax_fft.grid(True, alpha=0.3)

        # Mark expected stripe frequency
        if expected_freq > 0:
            ax_fft.axvline(expected_freq, color='r', linestyle='--', alpha=0.5,
                            label=f"Expected ({expected_freq:.3f})")
            ax_fft.legend()

    fig.suptitle(f"Rolling Shutter Stripe Test — 500Hz LED Toggle ({w}x{h})",
                 fontsize=14)
    plt.tight_layout()
    plt.savefig(OUTPUT_PATH, dpi=150)
    print(f"\nSaved to {OUTPUT_PATH}", file=sys.stderr)

    # Summary
    b_col, b_fi, b_data, b_per, b_amp, _ = best_result
    print(f"\nBest: col={b_col}, frame={b_fi}, period={b_per:.1f} rows, "
          f"amplitude={b_amp:.0f}")
    tol = expected_period * 0.4  # 40% tolerance
    if abs(b_per - expected_period) < tol and b_amp > 100:
        print("STRIPES DETECTED — rolling shutter communication looks feasible!")
    elif b_amp > 50:
        print("Weak stripes or wrong frequency — may need exposure tuning")
    else:
        print("No clear stripes — ISP may be removing rolling shutter artifacts")


if __name__ == "__main__":
    main()
