"""CRC-8/CCITT checksum.

Polynomial: x^8 + x^2 + x + 1 (0x07)
"""

CRC8_POLY = 0x07


def crc8(data: bytes, init: int = 0x00) -> int:
    """Compute CRC-8/CCITT over data bytes."""
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc
