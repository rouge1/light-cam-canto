"""Manchester encoding/decoding (IEEE 802.3 convention).

Each data bit maps to two symbols:
  0 -> [1, 0]  (falling edge at mid-bit)
  1 -> [0, 1]  (rising edge at mid-bit)

Every bit has a guaranteed mid-bit transition, enabling self-clocking.
"""

ENCODE_MAP = {0: [1, 0], 1: [0, 1]}
DECODE_MAP = {(1, 0): 0, (0, 1): 1}


def manchester_encode(bits: list[int]) -> list[int]:
    """Encode data bits into Manchester symbols (2 symbols per bit)."""
    return [s for b in bits for s in ENCODE_MAP[b]]


def manchester_decode(symbols: list[int]) -> list[int] | None:
    """Decode Manchester symbols back to data bits.

    Returns None if any symbol pair is invalid or length is odd.
    """
    if len(symbols) % 2 != 0:
        return None
    bits = []
    for i in range(0, len(symbols), 2):
        pair = (symbols[i], symbols[i + 1])
        if pair not in DECODE_MAP:
            return None
        bits.append(DECODE_MAP[pair])
    return bits
