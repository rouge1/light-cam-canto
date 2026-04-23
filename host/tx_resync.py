"""Send a message using the resync-aware wire format.

Encodes via `protocol.frame.encode_frame_symbols_with_resync` and ships the raw
symbol stream as hex to `irlink tx-symbols` over SSH. Use with
`python -m host.pixel_rx --decoder resync ...` on the receiving side.

Usage:
    python -m host.tx_resync --cam cam1 --speed 110 "HELLO"
    python -m host.tx_resync --cam cam1 --speed 100 --chunk-syms 48 "..."
    python -m host.tx_resync --print-only "HELLO"          # just show the hex
"""
import argparse
import subprocess
import sys

from protocol.frame import (
    encode_frame_symbols_with_resync,
    encode_frame_symbols_with_resync_fec,
    symbols_to_hex,
    DEFAULT_CHUNK_SYMS,
    DEFAULT_RESYNC_SYMS,
    DEFAULT_RS_N,
    DEFAULT_RS_K,
)

CAMERAS = {
    "cam1": "192.168.50.110",
    "cam2": "192.168.50.141",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("message")
    parser.add_argument("--cam", default="cam1", choices=sorted(CAMERAS.keys()))
    parser.add_argument("--speed", type=int, default=110, help="symbol_ms")
    parser.add_argument("--chunk-syms", type=int, default=DEFAULT_CHUNK_SYMS)
    parser.add_argument("--resync-syms", type=int, default=DEFAULT_RESYNC_SYMS)
    parser.add_argument("--fec", action="store_true",
                        help="Wrap payload in Reed-Solomon FEC for error correction")
    parser.add_argument("--rs-n", type=int, default=DEFAULT_RS_N,
                        help="RS codeword length (default 15)")
    parser.add_argument("--rs-k", type=int, default=DEFAULT_RS_K,
                        help="RS message length per block (default 11)")
    parser.add_argument("--print-only", action="store_true",
                        help="Print hex + stats; skip SSH")
    args = parser.parse_args()

    if args.fec:
        symbols = encode_frame_symbols_with_resync_fec(
            args.message,
            chunk_syms=args.chunk_syms, resync_syms=args.resync_syms,
            rs_n=args.rs_n, rs_k=args.rs_k,
        )
        fec_label = f"RS({args.rs_n},{args.rs_k})"
    else:
        symbols = encode_frame_symbols_with_resync(
            args.message, chunk_syms=args.chunk_syms, resync_syms=args.resync_syms,
        )
        fec_label = "no FEC"
    hex_str = symbols_to_hex(symbols)
    on_air_sec = (len(symbols) * args.speed) / 1000.0
    data_bits = len(args.message) * 8
    effective_bps = data_bits / on_air_sec if on_air_sec > 0 else 0

    print(f"TX via {args.cam}: msg={args.message!r} ({len(args.message)} chars)")
    print(f"Symbols: {len(symbols)}, speed={args.speed}ms/sym, "
          f"~{on_air_sec:.1f}s on air, effective {effective_bps:.2f} bps")
    print(f"Resync layout: chunk={args.chunk_syms} syms, resync={args.resync_syms} syms, {fec_label}")
    print(f"Hex ({len(hex_str)} chars): {hex_str[:80]}{'...' if len(hex_str) > 80 else ''}")

    if args.print_only:
        return 0

    cam_ip = CAMERAS[args.cam]
    remote_cmd = f"/opt/bin/irlink tx-symbols {hex_str} --speed {args.speed}"
    cmd = ["ssh", f"root@{cam_ip}", remote_cmd]
    print(f"$ ssh root@{cam_ip} /opt/bin/irlink tx-symbols <{len(hex_str)}ch> --speed {args.speed}")
    result = subprocess.run(cmd)
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
