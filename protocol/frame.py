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
