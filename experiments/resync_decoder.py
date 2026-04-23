"""Resync-aware DPLL decoder.

Pairs with protocol.frame.encode_frame_symbols_with_resync on the TX side.

Strategy (v1):
  1. Run the existing DPLL (same one pixel_rx.py uses) to extract a symbol stream.
     Resync blocks are '1010...' which already have an edge every symbol time —
     the baseline DPLL's phase-correction finds these edges and naturally
     re-locks on them without code changes.
  2. Locate the Manchester-encoded sync word at the symbol level.
  3. Strip resync blocks from the symbols following sync.
  4. Manchester-decode the remaining data symbols, parse length + payload + CRC.

If v1 is not aggressive enough, v2 can add an explicit phase-reset hook in the
DPLL at each chunk boundary (see `decode_samples_resync_aggressive`, TODO).
"""
from __future__ import annotations

import numpy as np

from protocol.manchester import manchester_encode, manchester_decode
from protocol.frame import (
    PREAMBLE_BITS,
    SYNC_WORD,
    POSTAMBLE_BITS,
    DEFAULT_CHUNK_SYMS,
    DEFAULT_RESYNC_SYMS,
    DEFAULT_RS_N,
    DEFAULT_RS_K,
    strip_resync_symbols,
    _bits_to_byte,
    _rs_encoded_size,
)
from protocol.crc import crc8


def _manchester_decode_tolerant(symbols, max_errors=4):
    """Manchester-decode, tolerating up to `max_errors` invalid pairs.

    Returns a list of candidate bit streams, each paired with its error count,
    sorted by error count (clean first). For invalid pairs, both bit=0 and
    bit=1 are enumerated. Empty list if too many errors.
    """
    if len(symbols) % 2 != 0:
        return []

    bits = []
    error_positions = []  # bit-index positions that were invalid
    for i in range(0, len(symbols), 2):
        pair = (symbols[i], symbols[i + 1])
        if pair == (1, 0):
            bits.append(0)
        elif pair == (0, 1):
            bits.append(1)
        else:
            bits.append(0)  # placeholder, will be overridden
            error_positions.append(len(bits) - 1)

    if len(error_positions) > max_errors:
        return []

    # Enumerate all 2^N combinations of the unknown bits.
    candidates = []
    n_err = len(error_positions)
    for mask in range(1 << n_err):
        trial = list(bits)
        for k, pos in enumerate(error_positions):
            trial[pos] = (mask >> k) & 1
        candidates.append((trial, n_err))
    return candidates


def _extract_symbols_dpll(times, values, threshold, symbol_ms, phase_init_off, gain):
    """One pass of the DPLL from pixel_rx.py.  Returns symbol list or None."""
    T = float(symbol_ms)
    dig_raw = (values >= threshold).astype(np.int8)

    first_edge_t = None
    for i in range(1, len(dig_raw)):
        if dig_raw[i] == 1 and dig_raw[i - 1] == 0:
            first_edge_t = (times[i - 1] + times[i]) / 2.0
            break
    if first_edge_t is None:
        return None

    phase = first_edge_t + T * phase_init_off
    symbols = []

    while phase < times[-1] - T * 0.3:
        w_lo = phase - T * 0.3
        w_hi = phase + T * 0.3
        mask = (times >= w_lo) & (times <= w_hi)
        window_vals = values[mask]
        if len(window_vals) == 0:
            idx = int(np.argmin(np.abs(times - phase)))
            sym_val = 1 if values[idx] >= threshold else 0
        else:
            sym_val = 1 if float(window_vals.mean()) >= threshold else 0
        symbols.append(sym_val)

        boundary = phase + T * 0.5
        search_lo = boundary - T * 0.4
        search_hi = boundary + T * 0.4

        best_edge_t = None
        best_dist = T * 0.4 + 1

        for j in range(1, len(times)):
            if times[j] < search_lo or times[j - 1] > search_hi:
                continue
            if dig_raw[j] != dig_raw[j - 1]:
                edge_t = (times[j - 1] + times[j]) / 2.0
                dist = abs(edge_t - boundary)
                if dist < best_dist:
                    best_dist = dist
                    best_edge_t = edge_t

        if best_edge_t is not None:
            error = best_edge_t - boundary
            phase += T + gain * error
        else:
            phase += T

    return symbols


def _try_parse_resync_framed(
    symbols: list[int],
    chunk_syms: int,
    resync_syms: int,
    sync_pattern: list[int],
) -> str | None:
    """Given extracted symbols, find sync word, strip resync, decode payload.

    Peeks at the length byte (first 16 symbols of the first chunk — always
    before the first resync block since chunk_syms >= 16) to determine the
    exact number of data symbols to keep. Avoids trailing TX-trailer garbage
    polluting the Manchester decode.
    """
    SL = len(sync_pattern)
    n = len(symbols)

    # Search for sync pattern anywhere in the symbol stream.
    for start in range(n - SL):
        if symbols[start:start + SL] != sync_pattern:
            continue

        after_sync = symbols[start + SL:]

        # Length byte is the first 16 symbols of the first chunk (chunk_syms>=16).
        if len(after_sync) < 16:
            continue
        length_bits = manchester_decode(after_sync[:16])
        if length_bits is None:
            continue
        length = _bits_to_byte(length_bits[:8])
        if length < 1 or length > 255:
            continue

        # Total DATA symbols we need (length byte + payload + CRC + postamble, all Manchester).
        needed_data_syms = 16 + length * 16 + 16 + 8

        # Walk the after_sync stream, collecting chunks while stripping resync.
        data_syms: list[int] = []
        i = 0
        m = len(after_sync)
        while i < m and len(data_syms) < needed_data_syms:
            take = min(chunk_syms, needed_data_syms - len(data_syms), m - i)
            data_syms.extend(after_sync[i:i + take])
            i += take
            if len(data_syms) >= needed_data_syms:
                break
            # Skip the resync block between chunks.
            i += resync_syms
        if len(data_syms) < needed_data_syms:
            # Not enough signal captured; try the next sync candidate.
            continue

        # Tolerant Manchester decode: enumerate candidate bit streams when a
        # few pairs are invalid (typical for isolated noise hits on otherwise
        # clean resync-locked signals). CRC picks the right candidate.
        # max_errors=6 gives 2^6=64 combos per sync candidate — still trivial.
        candidates = _manchester_decode_tolerant(data_syms, max_errors=6)
        for bits, _nerr in candidates:
            if len(bits) < 8 + length * 8 + 8:
                continue
            payload_bytes = []
            offset = 8
            for _ in range(length):
                payload_bytes.append(_bits_to_byte(bits[offset:offset + 8]))
                offset += 8
            received_crc = _bits_to_byte(bits[offset:offset + 8])

            payload = bytes(payload_bytes)
            crc_data = bytes([length]) + payload
            if crc8(crc_data) == received_crc:
                try:
                    return payload.decode("ascii", errors="replace")
                except Exception:
                    return None
    return None


def decode_samples_resync(
    samples,
    symbol_ms,
    min_delta,
    chunk_syms: int = DEFAULT_CHUNK_SYMS,
    resync_syms: int = DEFAULT_RESYNC_SYMS,
) -> str | None:
    """Resync-aware decoder for captures with mid-frame resync markers."""
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
    print(f"  resync: {len(samples)} samples, brightness {bmin:.0f}-{bmax:.0f}, "
          f"threshold={threshold:.0f}, chunk={chunk_syms} resync={resync_syms}")

    sync_pattern = manchester_encode(SYNC_WORD)  # 16 symbols

    for phase_init_off in (0.5, 0.3, 0.7):
        for gain in (0.15, 0.25, 0.35):
            symbols = _extract_symbols_dpll(
                times, values, threshold, symbol_ms, phase_init_off, gain
            )
            if symbols is None or len(symbols) < 32:
                continue

            # Try direct + inverted polarity.
            for syms in (symbols, [1 - s for s in symbols]):
                result = _try_parse_resync_framed(
                    syms, chunk_syms, resync_syms, sync_pattern
                )
                if result is not None:
                    print(f"  resync decode OK: phase_off={phase_init_off} gain={gain}")
                    return result

    print("  resync decode failed")
    return None


# ---------------------------------------------------------------------------
# Resync + Reed-Solomon FEC decoder
# ---------------------------------------------------------------------------

def _decode_manchester_with_erasures(symbols: list[int]) -> tuple[list[int], list[int]]:
    """Manchester-decode, marking invalid pairs.

    Returns (bits, erasure_bit_positions). Invalid pairs produce bit=0 at that
    position and the position is added to the erasure list. RS can leverage
    erasure positions (known-bad locations) to correct 2× as many errors as
    without positional info.
    """
    bits = []
    erasures = []
    for i in range(0, len(symbols) - 1, 2):
        pair = (symbols[i], symbols[i + 1])
        if pair == (1, 0):
            bits.append(0)
        elif pair == (0, 1):
            bits.append(1)
        else:
            bits.append(0)
            erasures.append(len(bits) - 1)
    return bits, erasures


def _try_parse_resync_fec_framed(
    symbols: list[int],
    chunk_syms: int,
    resync_syms: int,
    sync_pattern: list[int],
    rs_n: int,
    rs_k: int,
) -> str | None:
    """Sync-search + length-aware truncation + RS decode with erasures.

    Unlike the plain resync decoder, the LENGTH byte here is the ORIGINAL
    message length and the on-wire payload is
    `_rs_encoded_size(orig_len, rs_n, rs_k)` bytes.

    Invalid Manchester pairs are passed to RS as byte-level erasures, letting
    RS correct up to 2T erasures per block (vs T random errors) — doubles the
    effective correction capacity when the receiver can locate bad positions.
    """
    from reedsolo import RSCodec, ReedSolomonError

    SL = len(sync_pattern)
    n = len(symbols)
    rsc = RSCodec(rs_n - rs_k, nsize=rs_n)

    for start in range(n - SL):
        if symbols[start:start + SL] != sync_pattern:
            continue

        after_sync = symbols[start + SL:]
        if len(after_sync) < 16:
            continue
        length_bits = manchester_decode(after_sync[:16])
        if length_bits is None:
            continue
        orig_len = _bits_to_byte(length_bits[:8])
        if orig_len < 1:
            continue
        wire_len = _rs_encoded_size(orig_len, rs_n, rs_k)
        if wire_len > 255:
            continue

        needed_data_syms = 16 + wire_len * 16 + 16 + 8
        data_syms: list[int] = []
        i = 0
        m = len(after_sync)
        while i < m and len(data_syms) < needed_data_syms:
            take = min(chunk_syms, needed_data_syms - len(data_syms), m - i)
            data_syms.extend(after_sync[i:i + take])
            i += take
            if len(data_syms) >= needed_data_syms:
                break
            i += resync_syms
        if len(data_syms) < needed_data_syms:
            continue

        bits, bit_erasures = _decode_manchester_with_erasures(data_syms)
        if len(bits) < 8 + wire_len * 8 + 8:
            continue

        wire_payload = bytearray()
        offset = 8
        for _ in range(wire_len):
            wire_payload.append(_bits_to_byte(bits[offset:offset + 8]))
            offset += 8
        received_crc = _bits_to_byte(bits[offset:offset + 8])

        # Convert bit erasures in the payload region into byte-positions.
        payload_bit_start = 8
        payload_bit_end = 8 + wire_len * 8
        byte_erasures = set()
        for bp in bit_erasures:
            if payload_bit_start <= bp < payload_bit_end:
                byte_pos = (bp - payload_bit_start) // 8
                byte_erasures.add(byte_pos)
        erase_list = sorted(byte_erasures)

        # Clean-frame CRC check (fast path).
        if crc8(bytes([orig_len]) + bytes(wire_payload)) == received_crc:
            try:
                decoded, _, _ = rsc.decode(bytes(wire_payload))
            except ReedSolomonError:
                continue
            if len(decoded) != orig_len:
                continue
            try:
                return bytes(decoded).decode("ascii", errors="replace")
            except Exception:
                return None

        # RS correction with erasure hints, then CRC-verify against corrected bytes.
        try:
            decoded, decoded_msgecc, _ = rsc.decode(bytes(wire_payload), erase_pos=erase_list)
        except ReedSolomonError:
            continue
        if len(decoded) != orig_len:
            continue
        if crc8(bytes([orig_len]) + bytes(decoded_msgecc)) == received_crc:
            try:
                return bytes(decoded).decode("ascii", errors="replace")
            except Exception:
                return None
    return None


def decode_samples_resync_fec(
    samples,
    symbol_ms,
    min_delta,
    chunk_syms: int = DEFAULT_CHUNK_SYMS,
    resync_syms: int = DEFAULT_RESYNC_SYMS,
    rs_n: int = DEFAULT_RS_N,
    rs_k: int = DEFAULT_RS_K,
) -> str | None:
    """Resync + Reed-Solomon decoder for FEC-wrapped captures."""
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
    print(f"  resync+fec: {len(samples)} samples, brightness {bmin:.0f}-{bmax:.0f}, "
          f"threshold={threshold:.0f}, chunk={chunk_syms} resync={resync_syms} "
          f"RS({rs_n},{rs_k})")

    sync_pattern = manchester_encode(SYNC_WORD)

    for phase_init_off in (0.5, 0.3, 0.7):
        for gain in (0.15, 0.25, 0.35):
            symbols = _extract_symbols_dpll(
                times, values, threshold, symbol_ms, phase_init_off, gain
            )
            if symbols is None or len(symbols) < 32:
                continue

            for syms in (symbols, [1 - s for s in symbols]):
                result = _try_parse_resync_fec_framed(
                    syms, chunk_syms, resync_syms, sync_pattern, rs_n, rs_k,
                )
                if result is not None:
                    print(f"  resync+fec decode OK: phase_off={phase_init_off} gain={gain}")
                    return result

    print("  resync+fec decode failed")
    return None
