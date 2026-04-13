"""Tests for protocol frame encoding/decoding."""
from protocol.frame import (
    encode_frame, decode_frame,
    encode_frame_bits, parse_frame_bits,
    PREAMBLE_BITS, SYNC_WORD, POSTAMBLE_BITS,
)
from protocol.manchester import manchester_encode


def test_roundtrip_hello():
    symbols = encode_frame("HELLO")
    result = decode_frame(symbols)
    assert result == "HELLO"


def test_roundtrip_single_char():
    symbols = encode_frame("A")
    result = decode_frame(symbols)
    assert result == "A"


def test_roundtrip_numbers():
    symbols = encode_frame("12345")
    result = decode_frame(symbols)
    assert result == "12345"


def test_roundtrip_raw_bits():
    bits = encode_frame_bits("HI")
    result = parse_frame_bits(bits)
    assert result == "HI"


def test_frame_bits_structure():
    bits = encode_frame_bits("A")
    # Should start with preamble
    assert bits[:len(PREAMBLE_BITS)] == PREAMBLE_BITS
    # Then sync word
    p = len(PREAMBLE_BITS)
    assert bits[p:p + len(SYNC_WORD)] == SYNC_WORD
    # Should end with postamble
    assert bits[-len(POSTAMBLE_BITS):] == POSTAMBLE_BITS


def test_corrupted_bit_detected():
    bits = encode_frame_bits("HELLO")
    # Flip a payload bit
    data_start = len(PREAMBLE_BITS) + len(SYNC_WORD) + 8  # after preamble+sync+length
    bits[data_start + 3] ^= 1
    result = parse_frame_bits(bits)
    assert result is None


def test_decode_with_leading_noise():
    """Decoder finds sync word even with noise before frame."""
    noise = [0, 1, 1, 0, 1, 0, 0, 1, 0, 1]
    bits = noise + encode_frame_bits("HI")
    result = parse_frame_bits(bits)
    assert result == "HI"


def test_manchester_symbols_with_noise():
    """Full Manchester roundtrip with leading noise symbols."""
    noise_symbols = [0, 1, 1, 0, 0, 1, 0, 1]
    symbols = noise_symbols + encode_frame("OK")
    # decode_frame expects clean Manchester — this tests that
    # parse_frame_bits handles noise before the sync word
    # We need to Manchester-decode valid symbols, so prepend valid encoded noise
    from protocol.manchester import manchester_decode
    # Actually, noise symbols might fail Manchester decode.
    # Test with valid Manchester-encoded noise instead.
    noise_bits = [0, 1, 1, 0]
    noise_symbols = manchester_encode(noise_bits)
    symbols = noise_symbols + encode_frame("OK")
    result = decode_frame(symbols)
    assert result == "OK"


def test_all_printable_ascii():
    msg = "Hello, World! 123"
    symbols = encode_frame(msg)
    result = decode_frame(symbols)
    assert result == msg


def test_length_field_correct():
    bits = encode_frame_bits("ABC")
    p = len(PREAMBLE_BITS) + len(SYNC_WORD)
    length_bits = bits[p:p + 8]
    length = 0
    for b in length_bits:
        length = (length << 1) | b
    assert length == 3
