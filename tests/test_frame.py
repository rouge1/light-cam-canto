"""Tests for protocol frame encoding/decoding."""
from protocol.frame import (
    encode_frame, decode_frame,
    encode_frame_bits, parse_frame_bits,
    PREAMBLE_BITS, SYNC_WORD, POSTAMBLE_BITS,
    encode_frame_symbols_with_resync, strip_resync_symbols,
    encode_frame_symbols_with_resync_fec, _rs_encoded_size,
    symbols_to_hex,
    DEFAULT_CHUNK_SYMS, DEFAULT_RESYNC_SYMS,
    DEFAULT_RS_N, DEFAULT_RS_K,
)
from protocol.manchester import manchester_encode, manchester_decode


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


def _decode_resync_stream(symbols, chunk_syms, resync_syms):
    """Strip header + resync blocks and Manchester-decode back to message."""
    header_len = 32  # preamble (16 syms) + sync (16 syms)
    after_sync = symbols[header_len:]
    data_syms = strip_resync_symbols(after_sync, chunk_syms, resync_syms)
    bits = manchester_decode(data_syms)
    assert bits is not None
    # parse_frame_bits looks for the sync word; re-prepend it.
    return parse_frame_bits(list(SYNC_WORD) + bits)


def test_resync_roundtrip_short():
    msg = "HELLO"
    syms = encode_frame_symbols_with_resync(msg)
    assert _decode_resync_stream(syms, DEFAULT_CHUNK_SYMS, DEFAULT_RESYNC_SYMS) == msg


def test_resync_roundtrip_long_pangram():
    msg = "The quick brown fox jumps over the lazy dog"
    syms = encode_frame_symbols_with_resync(msg)
    # Long frames exercise the resync-block interleaving multiple times.
    assert _decode_resync_stream(syms, DEFAULT_CHUNK_SYMS, DEFAULT_RESYNC_SYMS) == msg


def test_resync_custom_chunk_sizes():
    msg = "hello world"
    for chunk, resync in [(32, 8), (64, 12), (96, 24)]:
        syms = encode_frame_symbols_with_resync(msg, chunk_syms=chunk, resync_syms=resync)
        assert _decode_resync_stream(syms, chunk, resync) == msg, f"failed for {chunk}/{resync}"


def test_resync_includes_preamble_and_sync():
    msg = "X"
    syms = encode_frame_symbols_with_resync(msg)
    # First 32 symbols = Manchester(preamble) + Manchester(sync).
    expected_header = manchester_encode(PREAMBLE_BITS + SYNC_WORD)
    assert syms[:32] == expected_header


def test_symbols_to_hex_roundtrip():
    syms = [1, 0, 1, 0, 1, 0, 1, 0,  # 0xAA
            0, 1, 0, 1, 0, 1, 0, 1]  # 0x55
    assert symbols_to_hex(syms) == "aa55"


def test_symbols_to_hex_pads_trailing():
    # 10 symbols -> 2 bytes, second byte right-padded with 0s
    syms = [1, 1, 1, 1, 0, 0, 0, 0, 1, 1]
    # First byte: 11110000 = 0xf0; second byte: 11000000 = 0xc0
    assert symbols_to_hex(syms) == "f0c0"


def test_rs_encoded_size():
    # RS(15,11): input bytes + ceil(n/11)*4 parity bytes
    assert _rs_encoded_size(0, 15, 11) == 0
    assert _rs_encoded_size(11, 15, 11) == 15   # 11 data + 4 parity = 1 block
    assert _rs_encoded_size(12, 15, 11) == 20   # 2 blocks: 12 + 8 parity
    assert _rs_encoded_size(22, 15, 11) == 30   # 2 blocks: 22 + 8 parity
    assert _rs_encoded_size(82, 15, 11) == 114  # 8 blocks: 82 + 32 parity


def test_fec_encode_decode_roundtrip():
    """Encoder output can be RS-decoded back to the original message."""
    from reedsolo import RSCodec
    msg = "HELLO FEC"
    syms = encode_frame_symbols_with_resync_fec(msg)
    # Strip header (32 syms), strip resync blocks, Manchester-decode, then RS-decode.
    after_sync = syms[32:]
    data_syms = strip_resync_symbols(after_sync)
    bits = manchester_decode(data_syms)
    assert bits is not None
    from protocol.frame import _bits_to_byte
    orig_len = _bits_to_byte(bits[:8])
    assert orig_len == len(msg)
    wire_len = _rs_encoded_size(orig_len, DEFAULT_RS_N, DEFAULT_RS_K)
    payload_bytes = bytearray()
    offset = 8
    for _ in range(wire_len):
        payload_bytes.append(_bits_to_byte(bits[offset:offset + 8]))
        offset += 8
    rsc = RSCodec(DEFAULT_RS_N - DEFAULT_RS_K, nsize=DEFAULT_RS_N)
    decoded, _, _ = rsc.decode(bytes(payload_bytes))
    assert bytes(decoded).decode("ascii") == msg


def test_fec_size_overflow_raises():
    """Messages that produce >255B on-wire payload should raise."""
    import pytest
    # At RS(15,11), 187 bytes encodes to 255. 188 encodes to 260.
    with pytest.raises(ValueError):
        encode_frame_symbols_with_resync_fec("x" * 188)
