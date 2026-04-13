"""Tests for Manchester encoding/decoding."""
from protocol.manchester import manchester_encode, manchester_decode


def test_encode_zero():
    assert manchester_encode([0]) == [1, 0]


def test_encode_one():
    assert manchester_encode([1]) == [0, 1]


def test_encode_sequence():
    assert manchester_encode([0, 1, 0]) == [1, 0, 0, 1, 1, 0]


def test_roundtrip():
    bits = [1, 0, 1, 1, 0, 0, 1, 0]
    symbols = manchester_encode(bits)
    assert manchester_decode(symbols) == bits


def test_invalid_pair_00():
    assert manchester_decode([0, 0]) is None


def test_invalid_pair_11():
    assert manchester_decode([1, 1]) is None


def test_odd_length():
    assert manchester_decode([0, 1, 0]) is None


def test_empty():
    assert manchester_decode([]) == []
    assert manchester_encode([]) == []
