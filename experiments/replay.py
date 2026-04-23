"""Replay dumped RTSP sample captures against decoder variants.

Enables offline A/B comparison of decoder changes (e.g. resync-aware DPLL)
against real-world captures, without repeating the on-air TX.

Usage:
    # dump a few captures first:
    #   python -m host.pixel_rx --cam cam2 --dump-samples runs/capture.jsonl
    #   (trigger TX from cam1 a few times)

    python -m experiments.replay runs/capture.jsonl
    python -m experiments.replay runs/capture.jsonl --decoder baseline
    python -m experiments.replay runs/capture.jsonl --symbol-ms 110

Each capture record is one JSON object per line, with fields:
    ts, rx_cam, tx_pixel, roi_size, symbol_ms, min_delta,
    baseline_at_start, samples (list of [t_ms, brightness]), decoded
"""
import argparse
import io
import json
import contextlib

from host.pixel_rx import decode_samples as decode_baseline
from experiments.resync_decoder import decode_samples_resync, decode_samples_resync_fec


def _decode_resync_default(samples, symbol_ms, min_delta):
    return decode_samples_resync(samples, symbol_ms, min_delta)


def _decode_resync_fec_default(samples, symbol_ms, min_delta):
    return decode_samples_resync_fec(samples, symbol_ms, min_delta)


DECODERS = {
    "baseline": decode_baseline,
    "resync": _decode_resync_default,
    "resync-fec": _decode_resync_fec_default,
}


def load_captures(path):
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            yield json.loads(line)


def replay(path, decoder_name, symbol_ms_override=None, verbose=False):
    decoder = DECODERS[decoder_name]

    total = 0
    decoded = 0
    for i, rec in enumerate(load_captures(path)):
        total += 1
        sym_ms = symbol_ms_override or rec["symbol_ms"]
        samples = [tuple(s) for s in rec["samples"]]

        if verbose:
            result = decoder(samples, sym_ms, rec["min_delta"])
        else:
            with contextlib.redirect_stdout(io.StringIO()):
                result = decoder(samples, sym_ms, rec["min_delta"])

        ok = result is not None
        if ok:
            decoded += 1
        truth = rec.get("decoded")
        dur = samples[-1][0] - samples[0][0] if len(samples) >= 2 else 0
        match = "="
        if truth is not None and result is not None:
            match = "=" if truth == result else "!="
        print(
            f"[{i:3d}] T={sym_ms}ms n={len(samples):3d} dur={dur:5d}ms  "
            f"truth={truth!r}  {decoder_name}={result!r}  {match}"
        )

    print(f"\n{decoder_name}: {decoded}/{total} decoded")


def main():
    parser = argparse.ArgumentParser(description="Replay dumped captures against decoders")
    parser.add_argument("captures", help="JSONL file from pixel_rx --dump-samples")
    parser.add_argument("--decoder", default="baseline", choices=sorted(DECODERS.keys()))
    parser.add_argument("--symbol-ms", type=int, help="Override symbol_ms from capture metadata")
    parser.add_argument("-v", "--verbose", action="store_true", help="Show per-capture decoder debug output")
    args = parser.parse_args()
    replay(args.captures, args.decoder, args.symbol_ms, args.verbose)


if __name__ == "__main__":
    main()
