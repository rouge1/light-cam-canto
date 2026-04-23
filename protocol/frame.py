"""IR communication frame encoding and decoding with Manchester + CRC-8.

Frame format (data bits, before Manchester encoding):
  [PREAMBLE: 10101010] [SYNC: 11001011] [LENGTH: 8 bits] [PAYLOAD: N bytes] [CRC-8] [POSTAMBLE: 1010]

All data bits are Manchester-encoded for transmission (2 symbols per bit).
"""

from protocol.manchester import manchester_encode, manchester_decode
from protocol.crc import crc8

PREAMBLE_BITS = [1, 0, 1, 0, 1, 0, 1, 0]
SYNC_WORD = [1, 1, 0, 0, 1, 0, 1, 1]
POSTAMBLE_BITS = [1, 0, 1, 0]


def _byte_to_bits(byte: int) -> list[int]:
    """Convert a byte to 8 bits, MSB first."""
    return [(byte >> (7 - i)) & 1 for i in range(8)]


def _bits_to_byte(bits: list[int]) -> int:
    """Convert 8 bits (MSB first) to a byte."""
    value = 0
    for b in bits:
        value = (value << 1) | b
    return value


def encode_frame_bits(message: str) -> list[int]:
    """Encode message into raw data bits (pre-Manchester).

    Returns the full frame as data bits: preamble + sync + length + payload + CRC + postamble.
    """
    payload = message.encode("ascii")
    if len(payload) > 255:
        raise ValueError(f"Message too long ({len(payload)} bytes, max 255)")
    length = len(payload)
    crc_data = bytes([length]) + payload
    cs = crc8(crc_data)

    bits = []
    bits.extend(PREAMBLE_BITS)
    bits.extend(SYNC_WORD)
    bits.extend(_byte_to_bits(length))
    for byte in payload:
        bits.extend(_byte_to_bits(byte))
    bits.extend(_byte_to_bits(cs))
    bits.extend(POSTAMBLE_BITS)
    return bits


def encode_frame(message: str) -> list[int]:
    """Encode message into Manchester symbols ready for transmission."""
    data_bits = encode_frame_bits(message)
    return manchester_encode(data_bits)


def parse_frame_bits(bits: list[int]) -> str | None:
    """Decode raw data bits (post-Manchester) into a message.

    Finds sync word, extracts length, payload, CRC. Verifies checksum.
    Returns None on failure.
    """
    # Find sync word
    sync_idx = None
    for i in range(len(bits) - len(SYNC_WORD)):
        if bits[i:i + len(SYNC_WORD)] == SYNC_WORD:
            sync_idx = i + len(SYNC_WORD)
            break

    if sync_idx is None:
        return None

    remaining = bits[sync_idx:]

    # Need at least length (8) + 1 payload byte (8) + CRC (8) = 24 bits
    if len(remaining) < 24:
        return None

    # Extract length
    length = _bits_to_byte(remaining[0:8])

    # Extract payload + CRC
    needed = 8 + length * 8 + 8  # length + payload + CRC
    if len(remaining) < needed:
        return None

    payload_bytes = []
    offset = 8
    for _ in range(length):
        payload_bytes.append(_bits_to_byte(remaining[offset:offset + 8]))
        offset += 8

    received_crc = _bits_to_byte(remaining[offset:offset + 8])

    # Verify CRC
    payload = bytes(payload_bytes)
    crc_data = bytes([length]) + payload
    expected_crc = crc8(crc_data)

    if received_crc != expected_crc:
        return None

    return payload.decode("ascii", errors="replace")


def decode_frame(symbols: list[int]) -> str | None:
    """Decode Manchester symbols into a message.

    Manchester-decodes the symbols, then parses the frame.
    Returns None on failure.
    """
    bits = manchester_decode(symbols)
    if bits is None:
        return None
    return parse_frame_bits(bits)


# ---------------------------------------------------------------------------
# Resync-aware framing
#
# Wire layout (all in symbol space after TX-side Manchester encoding):
#   [PREAMBLE 16 syms]   # data bits '10101010' → Manchester symbols '0110...'
#   [SYNC 16 syms]       # data bits '11001011' → Manchester symbols '0101...'
#   [CHUNK chunk_syms]  [RESYNC resync_syms]
#   [CHUNK chunk_syms]  [RESYNC resync_syms]
#   ... up to the last chunk, which is <= chunk_syms and has no trailing resync.
#
# The RESYNC block is plain alternating '1010...' at the symbol level, giving
# the receiver DPLL an edge every symbol time to snap phase back to.  This is
# identical in texture to the initial preamble but embedded mid-frame.
#
# Both sides must agree on (chunk_syms, resync_syms). Defaults chosen to stay
# well inside the observed ~80-symbol drift wall at 120 ms/sym.
# ---------------------------------------------------------------------------

DEFAULT_CHUNK_SYMS = 48
DEFAULT_RESYNC_SYMS = 16


def _resync_block(n: int) -> list[int]:
    """Alternating 1010... of length n."""
    return [1 if i % 2 == 0 else 0 for i in range(n)]


def encode_frame_symbols_with_resync(
    message: str,
    chunk_syms: int = DEFAULT_CHUNK_SYMS,
    resync_syms: int = DEFAULT_RESYNC_SYMS,
) -> list[int]:
    """Encode a message into a raw symbol stream with mid-frame resync markers.

    Returns the full on-wire symbol list (0s and 1s). Caller transmits as-is
    via `irlink tx-symbols <hex>`.
    """
    if chunk_syms <= 0 or resync_syms <= 0:
        raise ValueError("chunk_syms and resync_syms must be positive")

    payload = message.encode("ascii")
    if len(payload) > 255:
        raise ValueError(f"Message too long ({len(payload)} bytes, max 255)")
    length = len(payload)
    crc_data = bytes([length]) + payload
    cs = crc8(crc_data)

    # Data bits (length + payload + CRC + postamble); NOT preamble/sync.
    data_bits: list[int] = []
    data_bits.extend(_byte_to_bits(length))
    for byte in payload:
        data_bits.extend(_byte_to_bits(byte))
    data_bits.extend(_byte_to_bits(cs))
    data_bits.extend(POSTAMBLE_BITS)

    data_symbols = manchester_encode(data_bits)

    # Header symbols (preamble + sync, Manchester-encoded together).
    header_symbols = manchester_encode(PREAMBLE_BITS + SYNC_WORD)

    # Interleave resync blocks between data chunks.
    out: list[int] = list(header_symbols)
    i = 0
    n = len(data_symbols)
    while i < n:
        out.extend(data_symbols[i:i + chunk_syms])
        i += chunk_syms
        if i < n:
            out.extend(_resync_block(resync_syms))
    return out


def symbols_to_hex(symbols: list[int]) -> str:
    """Pack symbols (0/1) MSB-first into hex bytes. Pads trailing bits with 0."""
    out = []
    for i in range(0, len(symbols), 8):
        chunk = symbols[i:i + 8]
        if len(chunk) < 8:
            chunk = chunk + [0] * (8 - len(chunk))
        byte = 0
        for s in chunk:
            byte = (byte << 1) | (s & 1)
        out.append(f"{byte:02x}")
    return "".join(out)


def strip_resync_symbols(
    symbols_after_sync: list[int],
    chunk_syms: int = DEFAULT_CHUNK_SYMS,
    resync_syms: int = DEFAULT_RESYNC_SYMS,
) -> list[int]:
    """Remove resync blocks from a symbol stream that starts right after SYNC.

    Assumes TX inserted `resync_syms` symbols after every `chunk_syms` data
    symbols. The last chunk may be shorter and has no trailing resync.
    """
    out: list[int] = []
    i = 0
    n = len(symbols_after_sync)
    while i < n:
        out.extend(symbols_after_sync[i:i + chunk_syms])
        i += chunk_syms
        if i < n:
            i += resync_syms
    return out


# ---------------------------------------------------------------------------
# Reed-Solomon FEC wrapping
#
# RS(n,k) over bytes. Default RS(15,11) corrects up to 2 byte errors per 15-byte
# block (~13% error-correct capacity). `reedsolo` handles block splitting.
#
# Wire layout (inside the frame, replacing the plain payload bytes):
#   [LENGTH 1B = original msg length]
#   [RS_ENCODED_PAYLOAD (len + ceil(len/k)*nsym) bytes]
#   [CRC-8 over: LENGTH || RS_ENCODED_PAYLOAD]
#   [POSTAMBLE bits]
#
# CRC is over the on-wire (RS-encoded) bytes so clean frames verify fast; if
# CRC fails the receiver still runs RS which may correct errors, then verifies
# the CRC matches the RS-corrected bytes.
# ---------------------------------------------------------------------------

DEFAULT_RS_N = 15
DEFAULT_RS_K = 11


def _rs_encoded_size(original_len: int, rs_n: int, rs_k: int) -> int:
    """Bytes produced by RSCodec.encode for an input of this length."""
    if original_len == 0:
        return 0
    n_blocks = (original_len + rs_k - 1) // rs_k
    return original_len + n_blocks * (rs_n - rs_k)


def _encode_frame_symbols_with_resync_bytes(
    payload: bytes,
    orig_length: int,
    chunk_syms: int,
    resync_syms: int,
) -> list[int]:
    """Build symbol stream from raw frame bytes. `orig_length` goes in the length
    field (may differ from len(payload) when payload is RS-encoded)."""
    if orig_length > 255:
        raise ValueError(f"length field overflow ({orig_length} > 255)")
    if len(payload) > 255:
        raise ValueError(f"on-wire payload too large ({len(payload)} > 255)")

    crc_data = bytes([orig_length]) + payload
    cs = crc8(crc_data)

    data_bits: list[int] = []
    data_bits.extend(_byte_to_bits(orig_length))
    for byte in payload:
        data_bits.extend(_byte_to_bits(byte))
    data_bits.extend(_byte_to_bits(cs))
    data_bits.extend(POSTAMBLE_BITS)

    data_symbols = manchester_encode(data_bits)
    header_symbols = manchester_encode(PREAMBLE_BITS + SYNC_WORD)

    out: list[int] = list(header_symbols)
    i = 0
    n = len(data_symbols)
    while i < n:
        out.extend(data_symbols[i:i + chunk_syms])
        i += chunk_syms
        if i < n:
            out.extend(_resync_block(resync_syms))
    return out


def encode_frame_symbols_with_resync_fec(
    message: str,
    chunk_syms: int = DEFAULT_CHUNK_SYMS,
    resync_syms: int = DEFAULT_RESYNC_SYMS,
    rs_n: int = DEFAULT_RS_N,
    rs_k: int = DEFAULT_RS_K,
) -> list[int]:
    """Resync framing + RS(n,k) FEC. Length byte records the ORIGINAL message length;
    on-wire payload is RS-encoded (original + parity). Receiver needs (rs_n, rs_k)
    to know how to split/decode."""
    from reedsolo import RSCodec

    if rs_k >= rs_n:
        raise ValueError("rs_k must be < rs_n")
    payload = message.encode("ascii")
    orig_len = len(payload)
    max_orig = 0
    # Find largest original length that fits in the 255-byte wire payload cap.
    for candidate_len in (orig_len,):
        wire_len = _rs_encoded_size(candidate_len, rs_n, rs_k)
        if wire_len > 255:
            raise ValueError(
                f"RS-encoded payload would be {wire_len}B (>255). "
                f"Reduce message to ≤{(255 * rs_k) // rs_n - (rs_n - rs_k)}B at RS({rs_n},{rs_k})."
            )

    rsc = RSCodec(rs_n - rs_k, nsize=rs_n)
    encoded = bytes(rsc.encode(payload))
    return _encode_frame_symbols_with_resync_bytes(
        encoded, orig_len, chunk_syms, resync_syms,
    )
