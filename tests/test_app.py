"""Tests for protocol.app — application-layer message encoding."""
import pytest

from protocol.app import (
    APP_HELLO, APP_META, APP_META_ACK, APP_CAL_RESULT, APP_STATS,
    APP_TEXT, APP_BYE, APP_CHUNK, APP_CHUNK_ACK, APP_NACK,
    PROTOCOL_VERSION, MAX_SINGLE_FRAME, MAX_CHUNK_DATA,
    STATUS_IDLE, STATUS_READY, STATUS_BUSY, STATUS_ERROR,
    pack_hello, pack_meta, pack_meta_ack, pack_cal_result, pack_stats,
    pack_text, pack_bye, pack_chunk, pack_chunk_ack, pack_nack,
    unpack, fragment, reassemble, missing_chunks,
)


def test_hello_roundtrip():
    b = pack_hello("cam1-window")
    m = unpack(b)
    assert m.type == APP_HELLO
    assert m.fields["name"] == "cam1-window"


def test_meta_roundtrip():
    b = pack_meta(1745280000, STATUS_READY, 1)
    m = unpack(b)
    assert m.type == APP_META
    assert m.fields["ts"] == 1745280000
    assert m.fields["status"] == STATUS_READY
    assert m.fields["ver"] == 1


def test_meta_fits_drift_budget():
    b = pack_meta(0xFFFFFFFF, STATUS_ERROR, 0xFFFF)
    assert len(b) <= MAX_SINGLE_FRAME


def test_meta_ack_roundtrip():
    b = pack_meta_ack()
    m = unpack(b)
    assert m.type == APP_META_ACK
    assert m.fields["ver"] == PROTOCOL_VERSION


def test_cal_result_roundtrip():
    b = pack_cal_result(385, 178)
    m = unpack(b)
    assert m.type == APP_CAL_RESULT
    assert m.fields["x"] == 385
    assert m.fields["y"] == 178


def test_cal_result_max_coords():
    b = pack_cal_result(640, 360)
    m = unpack(b)
    assert m.fields["x"] == 640
    assert m.fields["y"] == 360


def test_stats_roundtrip():
    b = pack_stats(tx=12, rx=8, crc_fail=1, retrans=2,
                   dpll_loss=3, baseline=110, delta=85)
    m = unpack(b)
    assert m.type == APP_STATS
    assert m.fields["tx"] == 12
    assert m.fields["rx"] == 8
    assert m.fields["crc_fail"] == 1
    assert m.fields["retrans"] == 2
    assert m.fields["dpll_loss"] == 3
    assert m.fields["baseline"] == 110
    assert m.fields["delta"] == 85


def test_text_roundtrip():
    b = pack_text("HELLO")
    m = unpack(b)
    assert m.type == APP_TEXT
    assert m.fields["text"] == "HELLO"


def test_text_at_single_frame_limit():
    # type byte (1) + text = 16 total. Max text = 15.
    text = "A" * (MAX_SINGLE_FRAME - 1)
    b = pack_text(text)
    assert len(b) == MAX_SINGLE_FRAME
    m = unpack(b)
    assert m.fields["text"] == text


def test_text_over_single_frame_raises():
    with pytest.raises(ValueError, match="too long"):
        pack_text("A" * MAX_SINGLE_FRAME)


def test_bye_roundtrip():
    b = pack_bye("session done")
    m = unpack(b)
    assert m.type == APP_BYE
    assert m.fields["reason"] == "session done"


def test_bye_empty():
    b = pack_bye()
    m = unpack(b)
    assert m.fields["reason"] == ""


def test_nack_roundtrip():
    b = pack_nack(0xAA, 0x01)
    m = unpack(b)
    assert m.type == APP_NACK
    assert m.fields["rejected_type"] == 0xAA
    assert m.fields["reason"] == 0x01


def test_unknown_type_raises():
    with pytest.raises(ValueError, match="unknown app type"):
        unpack(bytes([0xCC, 0xDD]))


def test_empty_payload_raises():
    with pytest.raises(ValueError, match="empty"):
        unpack(b"")


# ---------- Fragmentation ----------


def test_fragment_40_byte_payload_produces_correct_chunks():
    # 40-byte APP_TEXT payload → body = [APP_TEXT byte] + 40 text bytes = 41 bytes
    # 41 bytes / 12 per chunk = 4 chunks (12+12+12+5)
    payload = b"X" * 40
    chunks = fragment(APP_TEXT, payload, max_chunk=12)
    assert len(chunks) == 4

    # Every chunk must fit inside the drift budget
    for c in chunks:
        assert len(c) <= MAX_SINGLE_FRAME, f"chunk too big: {len(c)}"

    # Parse each and check structure
    parsed = [unpack(c) for c in chunks]
    assert all(p.type == APP_CHUNK for p in parsed)
    assert [p.fields["seq"] for p in parsed] == [0, 1, 2, 3]
    assert all(p.fields["total"] == 4 for p in parsed)


def test_fragment_reassemble_roundtrip():
    original = b"The quick brown fox jumps over the lazy dog" * 2  # 88 bytes
    chunks = fragment(APP_TEXT, original)
    app_type, reassembled = reassemble(chunks)
    assert app_type == APP_TEXT
    assert reassembled == original


def test_fragment_reassemble_exact_chunk_boundary():
    # Body = [app_type] + 12 bytes = 13 bytes → 2 chunks (12+1)
    payload = b"A" * 12
    chunks = fragment(APP_TEXT, payload, max_chunk=12)
    assert len(chunks) == 2
    app_type, reassembled = reassemble(chunks)
    assert app_type == APP_TEXT
    assert reassembled == payload


def test_fragment_small_payload_one_chunk():
    payload = b"Hi"
    chunks = fragment(APP_TEXT, payload, max_chunk=12)
    assert len(chunks) == 1
    app_type, reassembled = reassemble(chunks)
    assert app_type == APP_TEXT
    assert reassembled == payload


def test_reassemble_missing_chunk_raises():
    chunks = fragment(APP_TEXT, b"X" * 40)
    del chunks[2]
    with pytest.raises(ValueError, match="missing chunks"):
        reassemble(chunks)


def test_reassemble_mismatched_msg_id_raises():
    a = fragment(APP_TEXT, b"X" * 40, msg_id=1)
    b = fragment(APP_TEXT, b"Y" * 40, msg_id=2)
    mixed = a[:2] + b[2:]
    with pytest.raises(ValueError, match="inconsistent msg_id"):
        reassemble(mixed)


def test_fragment_chunk_budget_assertion():
    # If someone tries max_chunk > 12, function rejects (would bust budget)
    with pytest.raises(ValueError, match="max_chunk"):
        fragment(APP_TEXT, b"X" * 40, max_chunk=13)


# ---------- Chunk-ACK bitmap ----------


def test_chunk_ack_bitmap_full():
    b = pack_chunk_ack(msg_id=5, received_seqs=[0, 1, 2, 3], total=4)
    m = unpack(b)
    assert m.fields["msg_id"] == 5
    missing = missing_chunks(m.fields["bitmap"], total=4)
    assert missing == []


def test_chunk_ack_bitmap_partial():
    # total=4, received {0, 2} → missing {1, 3}
    b = pack_chunk_ack(msg_id=5, received_seqs=[0, 2], total=4)
    m = unpack(b)
    missing = missing_chunks(m.fields["bitmap"], total=4)
    assert missing == [1, 3]


def test_chunk_ack_bitmap_empty():
    b = pack_chunk_ack(msg_id=5, received_seqs=[], total=4)
    m = unpack(b)
    missing = missing_chunks(m.fields["bitmap"], total=4)
    assert missing == [0, 1, 2, 3]


def test_chunk_ack_spans_multiple_bitmap_bytes():
    b = pack_chunk_ack(msg_id=5, received_seqs=[0, 7, 8, 15], total=16)
    m = unpack(b)
    missing = missing_chunks(m.fields["bitmap"], total=16)
    assert missing == [1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14]


# ---------- Drift budget enforcement ----------


def test_all_single_frame_types_fit_budget():
    # HELLO name is variable — caller's responsibility to keep short
    assert len(pack_hello("cam1-window")) <= MAX_SINGLE_FRAME
    assert len(pack_hello("cam2-laptop")) <= MAX_SINGLE_FRAME
    assert len(pack_meta(0, STATUS_IDLE, 1)) <= MAX_SINGLE_FRAME
    assert len(pack_meta_ack()) <= MAX_SINGLE_FRAME
    assert len(pack_cal_result(640, 360)) <= MAX_SINGLE_FRAME
    assert len(pack_text("x" * 15)) <= MAX_SINGLE_FRAME
    assert len(pack_nack(0x01, 0x02)) <= MAX_SINGLE_FRAME
    assert len(pack_chunk(0, 0, 1, b"A" * 12)) <= MAX_SINGLE_FRAME
    # pack_bye(""): 1 byte. pack_bye with reason may exceed; v1: keep short
    assert len(pack_bye()) <= MAX_SINGLE_FRAME
