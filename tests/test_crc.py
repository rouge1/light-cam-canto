"""Tests for CRC-8/CCITT."""
from protocol.crc import crc8


def test_empty():
    assert crc8(b"") == 0x00


def test_single_byte():
    result = crc8(b"\x01")
    assert isinstance(result, int)
    assert 0 <= result <= 255


def test_deterministic():
    assert crc8(b"HELLO") == crc8(b"HELLO")


def test_different_data_different_crc():
    assert crc8(b"HELLO") != crc8(b"HELLP")


def test_single_bit_flip_detected():
    original = crc8(b"\x00\x01\x02")
    flipped = crc8(b"\x00\x01\x03")  # flip bit 0 of last byte
    assert original != flipped
