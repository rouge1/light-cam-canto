"""Diagnostic: replay a dumped capture, print the extracted symbol stream,
compare against the expected symbols for a known message.

Usage:
    python -m experiments.debug_capture runs/resync_110.jsonl \
        --message "The quick brown fox jumps over the lazy dog" \
        --symbol-ms 110
"""
import argparse
import json
import numpy as np

from protocol.manchester import manchester_encode
from protocol.frame import (
    SYNC_WORD, PREAMBLE_BITS, encode_frame_symbols_with_resync,
    DEFAULT_CHUNK_SYMS, DEFAULT_RESYNC_SYMS,
)
from experiments.resync_decoder import _extract_symbols_dpll


def load_first_capture(path):
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                return json.loads(line)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("capture")
    parser.add_argument("--message", required=True)
    parser.add_argument("--symbol-ms", type=int)
    parser.add_argument("--chunk-syms", type=int, default=DEFAULT_CHUNK_SYMS)
    parser.add_argument("--resync-syms", type=int, default=DEFAULT_RESYNC_SYMS)
    args = parser.parse_args()

    rec = load_first_capture(args.capture)
    if rec is None:
        print("no captures")
        return 1

    sym_ms = args.symbol_ms or rec["symbol_ms"]
    samples = [tuple(s) for s in rec["samples"]]
    times = np.array([s[0] for s in samples], dtype=np.float64)
    values = np.array([s[1] for s in samples], dtype=np.float64)
    bmin, bmax = values[1:].min(), values[1:].max()
    thr = (bmin + bmax) / 2.0
    print(f"capture: {len(samples)} samples, dur={times[-1]-times[0]:.0f}ms, "
          f"brightness {bmin:.0f}-{bmax:.0f}, thr={thr:.0f}")

    expected = encode_frame_symbols_with_resync(
        args.message, chunk_syms=args.chunk_syms, resync_syms=args.resync_syms,
    )
    print(f"expected: {len(expected)} symbols on air = {len(expected)*sym_ms}ms")
    print(f"expected first 64: {''.join(str(s) for s in expected[:64])}")
    print(f"expected last 32:  {''.join(str(s) for s in expected[-32:])}")

    # Try multiple DPLL configs, show extracted symbols
    print("\n--- DPLL extraction attempts ---")
    for phase_off in (0.5, 0.3, 0.7):
        for gain in (0.15, 0.25, 0.35):
            for thr_scale in (1.0, 0.9, 1.1):
                t = thr * thr_scale
                syms = _extract_symbols_dpll(times, values, t, sym_ms, phase_off, gain)
                if syms is None:
                    continue
                # Show similarity to expected
                matched = 0
                for i in range(min(len(syms), len(expected))):
                    if syms[i] == expected[i]:
                        matched += 1
                # Also try inverted
                syms_inv = [1-s for s in syms]
                matched_inv = 0
                for i in range(min(len(syms_inv), len(expected))):
                    if syms_inv[i] == expected[i]:
                        matched_inv += 1
                best = max(matched, matched_inv)
                inv_str = " (inverted)" if matched_inv > matched else ""
                if best < min(len(syms), len(expected)) * 0.7:
                    continue
                cmp_n = min(len(syms), len(expected))
                best_syms = syms_inv if matched_inv > matched else syms
                print(f"phase_off={phase_off} gain={gain} thr={t:.0f}{inv_str}: "
                      f"{len(syms)} syms, match {best}/{cmp_n} ({100*best/cmp_n:.1f}%)")
                # Show first divergence
                for i in range(cmp_n):
                    if best_syms[i] != expected[i]:
                        print(f"    first divergence at sym {i}: got {best_syms[i]}, want {expected[i]}")
                        print(f"    expected[{i}:{i+32}] = {''.join(str(s) for s in expected[i:i+32])}")
                        print(f"    got     [{i}:{i+32}] = {''.join(str(s) for s in best_syms[i:i+32])}")
                        break
                print(f"    got first 64:  {''.join(str(s) for s in best_syms[:64])}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
