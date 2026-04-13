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
    median_db = np.median(db)
    mad = np.median(np.abs(db - median_db))
    edge_threshold = max(mad * 10, 5.0)

    dt = np.diff(timestamps)
    dt[dt == 0] = 0.001
    median_dt = np.median(dt)
    kernel_size = max(3, int(0.05 / median_dt))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones(kernel_size) / kernel_size
    smooth_db = np.convolve(db, kernel, mode="same")

    events = []
    min_edge_gap = half_bit_duration * 0.5
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

    Tries multiple small alignments to handle edge-detection offsets.
    """
    for trim_start in range(4):
        for trim_end in range(3):
            end = len(symbols) - trim_end if trim_end > 0 else len(symbols)
            candidate = symbols[trim_start:end]
            if len(candidate) < 4 or len(candidate) % 2 != 0:
                continue
            bits = manchester_decode(candidate)
            if bits is None:
                continue
            msg = parse_frame_bits(bits)
            if msg is not None:
                if trim_start or trim_end:
                    print(f"RX: decoded with trim start={trim_start} end={trim_end}")
                print(f"RX: {len(bits)} data bits")
                return msg
    return None


def decode_samples(
    samples: list[tuple[float, float]],
    bit_duration: float = 1.0,
    save_plot: str | None = None,
) -> str | None:
    """Decode brightness samples into a message using Manchester decoding."""
    if len(samples) < 10:
        print("RX: not enough samples")
        return None

    timestamps = np.array([s[0] for s in samples])
    brightness = np.array([s[1] for s in samples])
    timestamps -= timestamps[0]

    half_bit = bit_duration / 2.0
    print(f"RX: {len(samples)} samples over {timestamps[-1]:.1f}s")
    print(f"RX: brightness range [{brightness.min():.1f}, {brightness.max():.1f}]")

    events = _detect_edges(timestamps, brightness, half_bit)
    rising = sum(1 for _, d in events if d == 1)
    falling = sum(1 for _, d in events if d == 0)
    print(f"RX: found {rising} rising, {falling} falling edges")

    if len(events) < 4:
        print("RX: not enough edges")
        return None

    # Estimate half-bit duration from short inter-edge intervals
    edge_times = np.array([e[0] for e in events])
    intervals = np.diff(edge_times)
    short_intervals = intervals[intervals < half_bit * 1.5]
    if len(short_intervals) > 2:
        estimated_half_bit = float(np.median(short_intervals))
    else:
        estimated_half_bit = half_bit
    print(f"RX: estimated half-bit={estimated_half_bit:.3f}s (expected={half_bit:.3f}s)")

    symbols = _edges_to_symbols(events, estimated_half_bit)
    print(f"RX: {len(symbols)} symbols")

    if save_plot:
        _save_debug_plot(timestamps, brightness, events, symbols, estimated_half_bit, save_plot)

    msg = _try_decode(symbols)
    if msg is None:
        print("RX: decode failed")
        print(f"RX: symbols: {''.join(map(str, symbols[:80]))}")
    return msg


def _save_debug_plot(timestamps, brightness, events, symbols, half_bit, path):
    """Save a debug plot."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    axes[0].plot(timestamps, brightness, "b-", linewidth=0.5)
    for t, d in events:
        color = "green" if d == 1 else "red"
        axes[0].axvline(t, color=color, alpha=0.5, linewidth=1)
    axes[0].set_ylabel("Brightness")
    axes[0].set_title("Brightness + Edges (green=ON, red=OFF)")
    axes[0].grid(True, alpha=0.3)

    if events and symbols:
        sym_times = []
        t = events[0][0]
        for _ in symbols:
            sym_times.append(t)
            t += half_bit
        if len(sym_times) == len(symbols):
            axes[1].step(sym_times, symbols, "purple", where="post", linewidth=1)
    axes[1].set_ylabel("Symbol")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Reconstructed Symbols")
    axes[1].set_ylim(-0.2, 1.2)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(path, dpi=150)
    print(f"RX: saved debug plot to {path}")
