"""RTSP-based IR receiver with Manchester decoding.

Captures frames from the camera's RTSP stream, records brightness
over time, detects edges, and decodes Manchester-encoded frames.

Manchester encoding guarantees transitions every half-bit, so
inter-edge intervals are always ~T/2 or ~T (where T = data bit period).
"""
import cv2
import numpy as np
import time
import threading

from protocol.frame import parse_frame_bits
from protocol.manchester import manchester_decode

# Samples cap: at 20fps, 10 minutes = 12000 samples. 20000 is generous.
MAX_SAMPLES = 20_000


class Receiver:
    """Captures brightness samples from an RTSP stream."""

    def __init__(self, rtsp_url: str):
        self.rtsp_url = rtsp_url
        self.samples: list[tuple[float, float]] = []
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=10)
        return self.samples

    def _capture_loop(self):
        cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            print("RX: FAILED to open RTSP stream")
            return

        print("RX: capturing...")
        while self._running:
            ret = cap.grab()
            if not ret:
                continue
            ret, frame = cap.retrieve()
            if not ret:
                continue

            # Use single channel — skip full BGR-to-gray conversion
            brightness = float(frame[:, :, 0].mean())
            self.samples.append((time.time(), brightness))

            if len(self.samples) >= MAX_SAMPLES:
                print("RX: sample buffer full")
                break

        cap.release()
        print(f"RX: captured {len(self.samples)} samples")


def _detect_edges(timestamps, brightness, half_bit_duration):
    """Detect rising and falling edges in brightness data."""
    db = np.diff(brightness)
    # Use a percentage of the brightness range as threshold.
    # This is more robust than MAD for long captures where auto-exposure
    # creates a bimodal brightness distribution.
    b_range = brightness.max() - brightness.min()
    edge_threshold = max(b_range * 0.08, 5.0)

    dt = np.diff(timestamps)
    dt[dt == 0] = 0.001
    median_dt = np.median(dt)
    kernel_size = max(3, int(0.05 / median_dt))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones(kernel_size) / kernel_size
    smooth_db = np.convolve(db, kernel, mode="same")

    events = []
    min_edge_gap = half_bit_duration * 0.7
    last_edge_time = -1
    i = 0

    while i < len(smooth_db):
        if timestamps[i] - last_edge_time < min_edge_gap:
            i += 1
            continue

        if smooth_db[i] > edge_threshold:
            peak_i = i
            while i < len(smooth_db) and smooth_db[i] > edge_threshold * 0.5:
                if smooth_db[i] > smooth_db[peak_i]:
                    peak_i = i
                i += 1
            events.append((timestamps[peak_i], 1))
            last_edge_time = timestamps[peak_i]

        elif smooth_db[i] < -edge_threshold:
            peak_i = i
            while i < len(smooth_db) and smooth_db[i] < -edge_threshold * 0.5:
                if smooth_db[i] < smooth_db[peak_i]:
                    peak_i = i
                i += 1
            events.append((timestamps[peak_i], 0))
            last_edge_time = timestamps[peak_i]

        else:
            i += 1

    return events


def _edges_to_symbols(events, half_bit_duration):
    """Convert edge events to Manchester symbols using interval quantization.

    In Manchester, inter-edge intervals are ~T/2 (one symbol slot)
    or ~T (two symbol slots).
    """
    if len(events) < 2:
        return []

    symbols = []
    for i in range(len(events)):
        state = events[i][1]

        if i + 1 < len(events):
            gap = events[i + 1][0] - events[i][0]
            n_slots = 1 if gap < half_bit_duration * 1.5 else round(gap / half_bit_duration)
            n_slots = max(1, min(n_slots, 4))
        else:
            n_slots = 1

        symbols.extend([state] * n_slots)

    return symbols


def _try_decode(symbols: list[int]) -> str | None:
    """Try to Manchester-decode and parse a symbol stream.

    Tries multiple small alignments and both polarities to handle
    edge-detection offsets and inverted brightness mapping.
    """
    for invert in [False, True]:
        syms = [1 - s for s in symbols] if invert else symbols
        for trim_start in range(6):
            for trim_end in range(6):
                end = len(syms) - trim_end if trim_end > 0 else len(syms)
                candidate = syms[trim_start:end]
                if len(candidate) < 4 or len(candidate) % 2 != 0:
                    continue
                bits = manchester_decode(candidate)
                if bits is None:
                    continue
                msg = parse_frame_bits(bits)
                if msg is not None:
                    inv_str = " (inverted)" if invert else ""
                    if trim_start or trim_end:
                        print(f"RX: decoded with trim start={trim_start} end={trim_end}{inv_str}")
                    elif invert:
                        print(f"RX: decoded with inverted polarity")
                    print(f"RX: {len(bits)} data bits")
                    return msg
    return None


def _find_tx_start(timestamps, brightness, half_bit):
    """Find when transmission starts using a variance detector.

    Before TX, brightness is stable. During TX, it oscillates rapidly.
    Finds the transition from low to high variance.
    """
    fps = len(timestamps) / (timestamps[-1] - timestamps[0])
    window = max(5, int(fps * half_bit * 4))  # ~2 data bits of samples

    # Compute rolling standard deviation
    rolling_std = np.array([
        np.std(brightness[max(0, i - window):i + 1])
        for i in range(len(brightness))
    ])

    # TX starts when rolling_std exceeds a threshold
    std_threshold = rolling_std.max() * 0.3

    # Skip first 1.5 seconds
    start_idx = np.searchsorted(timestamps, 1.5)

    for i in range(start_idx, len(rolling_std)):
        if rolling_std[i] > std_threshold:
            # Back up half a data bit to catch the preamble start
            return max(0, timestamps[i] - half_bit)

    return None


def _level_decode(timestamps, brightness, tx_start, half_bit, max_symbols=600):
    """Decode by sampling brightness at symbol midpoints.

    Uses a local adaptive threshold — for each sample point, compute
    the median brightness in a window around it and use that as the
    threshold. This handles auto-exposure drift over long transmissions.
    """
    # Window size: ~10 data bits worth of samples for stable local median
    window_seconds = half_bit * 20
    fps = len(timestamps) / (timestamps[-1] - timestamps[0])
    window_samples = max(10, int(window_seconds * fps))

    symbols = []
    t = tx_start + half_bit * 0.5

    while t < timestamps[-1] and len(symbols) < max_symbols:
        idx = np.argmin(np.abs(timestamps - t))

        # Local window for adaptive threshold
        w_start = max(0, idx - window_samples // 2)
        w_end = min(len(brightness), idx + window_samples // 2)
        local_median = np.median(brightness[w_start:w_end])

        symbols.append(1 if brightness[idx] > local_median else 0)
        t += half_bit

    return symbols


def decode_samples(
    samples: list[tuple[float, float]],
    bit_duration: float = 1.0,
    save_plot: str | None = None,
) -> str | None:
    """Decode brightness samples into a message.

    Uses two strategies:
    1. Level-based sampling at fixed intervals (robust for precise TX timing)
    2. Edge-based decoding as fallback
    """
    if len(samples) < 10:
        print("RX: not enough samples")
        return None

    timestamps = np.array([s[0] for s in samples])
    brightness = np.array([s[1] for s in samples])
    timestamps -= timestamps[0]

    half_bit = bit_duration / 2.0
    print(f"RX: {len(samples)} samples over {timestamps[-1]:.1f}s")
    print(f"RX: brightness range [{brightness.min():.1f}, {brightness.max():.1f}]")

    if brightness.max() - brightness.min() < 10:
        print("RX: brightness range too narrow — no signal")
        return None

    # Find transmission start
    tx_start = _find_tx_start(timestamps, brightness, half_bit)
    if tx_start is None:
        print("RX: could not find transmission start")
        return None
    print(f"RX: TX start at {tx_start:.2f}s")

    # Strategy 1: Level-based sampling
    symbols = _level_decode(timestamps, brightness, tx_start, half_bit)
    print(f"RX: level decode: {len(symbols)} symbols")

    if save_plot:
        _save_debug_plot(timestamps, brightness, [], symbols, half_bit, save_plot,
                         tx_start=tx_start)

    msg = _try_decode(symbols)
    if msg is not None:
        print("RX: decoded via level sampling")
        return msg

    # Strategy 2: Edge-based decoding as fallback
    print("RX: level decode failed, trying edge-based...")
    events = _detect_edges(timestamps, brightness, half_bit)
    rising = sum(1 for _, d in events if d == 1)
    falling = sum(1 for _, d in events if d == 0)
    print(f"RX: found {rising} rising, {falling} falling edges")

    if len(events) >= 4:
        edge_symbols = _edges_to_symbols(events, half_bit)
        print(f"RX: edge decode: {len(edge_symbols)} symbols")
        msg = _try_decode(edge_symbols)
        if msg is not None:
            print("RX: decoded via edge detection")
            return msg

    print("RX: decode failed")
    return None


def _save_debug_plot(timestamps, brightness, events, symbols, half_bit, path,
                     tx_start=None):
    """Save a debug plot."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    axes[0].plot(timestamps, brightness, "b-", linewidth=0.5)
    threshold = (brightness.min() + brightness.max()) / 2
    axes[0].axhline(threshold, color="orange", alpha=0.5, linestyle="--", label="threshold")
    if tx_start is not None:
        axes[0].axvline(tx_start, color="green", linewidth=2, label="TX start")
    axes[0].set_ylabel("Brightness")
    axes[0].set_title("Brightness Signal")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    if symbols and tx_start is not None:
        sym_times = [tx_start + i * half_bit for i in range(len(symbols))]
        axes[1].step(sym_times, symbols, "purple", where="post", linewidth=1)
    axes[1].set_ylabel("Symbol")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Decoded Symbols")
    axes[1].set_ylim(-0.2, 1.2)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(path, dpi=150)
    print(f"RX: saved debug plot to {path}")
