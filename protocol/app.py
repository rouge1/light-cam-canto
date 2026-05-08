"""Application-layer messages for the LiWiFi IR link.

Sits on top of the DATA payload of the existing protocol layer. Every app
message is a single DATA payload of the form:

    [app_type: 1 byte] [app_data: 0..MAX_APP_DATA bytes]

The first byte identifies the application message type; the rest is type-
specific. Pack functions produce bytes; unpack returns (app_type, fields).

Fragmentation: payloads larger than the clock-drift budget are split into
APP_CHUNK frames (max 12 data bytes each — chosen so that a full CHUNK
frame's DATA payload is 4 header + 12 data = 16 bytes, matching the
empirically-validated 16-byte single-frame ceiling at 120ms/symbol).

See /home/user/.claude/plans/steady-finding-pearl.md for the design rationale.
"""

from __future__ import annotations
import struct
from dataclasses import dataclass

APP_HELLO = 0x01
APP_META = 0x02
APP_META_ACK = 0x03
APP_CAL_RESULT = 0x04
APP_STATS = 0x05
APP_TEXT = 0x06
APP_BYE = 0x07
APP_CHUNK = 0x08
APP_CHUNK_ACK = 0x09
APP_CAL_VISUAL = 0x0E
APP_NACK = 0xFE

CAL_VISUAL_ZOOM_DIM = 4                   # 4x4 grid of cells (single panel)
CAL_VISUAL_PANEL_BYTES = 8                # 4x4 4-bit packed (16 cells / 2)
CAL_VISUAL_ZOOM_BYTES = CAL_VISUAL_PANEL_BYTES  # 8 bytes — single DATA frame, no fragmentation

PROTOCOL_VERSION = 1

MAX_DATA_PAYLOAD = 253
MAX_SINGLE_FRAME = 16
MAX_CHUNK_DATA = 12
CHUNK_HEADER = 4

NACK_UNKNOWN_TYPE = 0x01
NACK_MALFORMED = 0x02
NACK_TOO_LONG = 0x03


def pack_hello(name: str) -> bytes:
    n = name.encode("utf-8")
    return bytes([APP_HELLO]) + n + b"\x00"


STATUS_IDLE = 0
STATUS_READY = 1
STATUS_BUSY = 2
STATUS_ERROR = 3


def pack_meta(ts: int, status: int, ver: int = PROTOCOL_VERSION) -> bytes:
    """Compact node-state metadata. Name is NOT included — it's conveyed by APP_HELLO.

    Body: ts(4 BE) status(1) ver(2 BE) = 7 bytes. Total with type byte: 8.
    """
    return bytes([APP_META]) + struct.pack(">IBH", ts & 0xFFFFFFFF, status & 0xFF, ver & 0xFFFF)


def pack_meta_ack(ver: int = PROTOCOL_VERSION) -> bytes:
    """Minimal receipt for META. Body: ver(2 BE) = 2 bytes. Total: 3."""
    return bytes([APP_META_ACK]) + struct.pack(">H", ver & 0xFFFF)


def pack_cal_result(x: int, y: int) -> bytes:
    return bytes([APP_CAL_RESULT]) + struct.pack(">HH", x & 0xFFFF, y & 0xFFFF)


def pack_cal_visual(x: int, y: int, brightness: int, delta: int,
                    zoom_4bit: bytes) -> bytes:
    """Cal result + 4x4 grid-delta zoom from cam1's view (over-light cal).

    Body: x(2 BE) y(2 BE) brightness(1) delta(1) zoom(8) = 14 bytes.
    Total with type byte = 15 — fits the 16-byte single-frame ceiling, no
    fragmentation needed. Single DATA frame, ~30 sec over IR @ 160 ms/sym.

    zoom_4bit: 8 bytes encoding a 4x4 grid of 4-bit cells (two per byte;
    high nibble first). Each cell is a per-block brightness delta (LEDs-on
    minus LEDs-off) sampled from a 4x4 sub-area of the 20x12 brightness grid
    centered on the peak block. Quantized via /16 to 0..15.

    Single-frame chosen for reliability: any fragmented variant requires per-
    chunk ACK and breaks the bicall protocol margins on overloaded cams.
    The 4x4 visual is coarse but spatially honest — shows the peak block
    surrounded by adjacent blocks at the same scale as the on-camera grid.
    """
    if len(zoom_4bit) != CAL_VISUAL_ZOOM_BYTES:
        raise ValueError(
            f"zoom_4bit must be {CAL_VISUAL_ZOOM_BYTES} bytes, got {len(zoom_4bit)}"
        )
    return bytes([APP_CAL_VISUAL]) + struct.pack(
        ">HHBB", x & 0xFFFF, y & 0xFFFF, brightness & 0xFF, delta & 0xFF,
    ) + zoom_4bit


def unpack_cal_visual_panel(panel_bytes: bytes) -> list[list[int]]:
    """Expand a 32-byte 4-bit-packed panel into an 8x8 grid of 0-15 cells."""
    if len(panel_bytes) != CAL_VISUAL_PANEL_BYTES:
        raise ValueError(f"expected {CAL_VISUAL_PANEL_BYTES} bytes")
    dim = CAL_VISUAL_ZOOM_DIM
    grid = [[0] * dim for _ in range(dim)]
    for i, b in enumerate(panel_bytes):
        row, col_pair = divmod(i, dim // 2)
        grid[row][col_pair * 2] = (b >> 4) & 0x0F
        grid[row][col_pair * 2 + 1] = b & 0x0F
    return grid


def pack_cal_visual_panel(grid: list[list[int]]) -> bytes:
    """Inverse of unpack_cal_visual_panel — pack an 8x8 0-15 grid into 32 bytes."""
    dim = CAL_VISUAL_ZOOM_DIM
    if len(grid) != dim or any(len(row) != dim for row in grid):
        raise ValueError(f"grid must be {dim}x{dim}")
    out = bytearray(CAL_VISUAL_PANEL_BYTES)
    for row in range(dim):
        for col_pair in range(dim // 2):
            hi = grid[row][col_pair * 2] & 0x0F
            lo = grid[row][col_pair * 2 + 1] & 0x0F
            out[row * (dim // 2) + col_pair] = (hi << 4) | lo
    return bytes(out)


# Back-compat aliases — older code referred to a single panel as "the zoom".
# The triptych format ships TWO panels (OFF + ON). Helpers retained so the
# pre-triptych call sites keep working until cleaned up.
unpack_cal_visual_zoom = unpack_cal_visual_panel
pack_cal_visual_zoom = pack_cal_visual_panel


def pack_stats(tx: int, rx: int, crc_fail: int, retrans: int,
               dpll_loss: int, baseline: int, delta: int) -> bytes:
    return bytes([APP_STATS]) + struct.pack(
        ">HHHHHBB",
        tx & 0xFFFF, rx & 0xFFFF, crc_fail & 0xFFFF,
        retrans & 0xFFFF, dpll_loss & 0xFFFF,
        baseline & 0xFF, delta & 0xFF,
    )


def pack_text(text: str) -> bytes:
    b = text.encode("utf-8")
    if len(b) > MAX_SINGLE_FRAME - 1:
        raise ValueError(
            f"text too long for single frame ({len(b)} > {MAX_SINGLE_FRAME - 1}); use fragment()"
        )
    return bytes([APP_TEXT]) + b


def pack_bye(reason: str = "") -> bytes:
    return bytes([APP_BYE]) + reason.encode("utf-8")


def pack_chunk(msg_id: int, seq: int, total: int, data: bytes) -> bytes:
    if len(data) > MAX_CHUNK_DATA:
        raise ValueError(f"chunk data > {MAX_CHUNK_DATA}")
    if seq >= total:
        raise ValueError(f"seq {seq} >= total {total}")
    return bytes([APP_CHUNK, msg_id & 0xFF, seq & 0xFF, total & 0xFF]) + data


def pack_chunk_ack(msg_id: int, received_seqs: list[int], total: int) -> bytes:
    nbytes = (total + 7) // 8
    bitmap = bytearray(nbytes)
    for s in received_seqs:
        if 0 <= s < total:
            bitmap[s // 8] |= 1 << (s % 8)
    return bytes([APP_CHUNK_ACK, msg_id & 0xFF]) + bytes(bitmap)


def pack_nack(rejected_type: int, reason: int) -> bytes:
    return bytes([APP_NACK, rejected_type & 0xFF, reason & 0xFF])


@dataclass
class AppMessage:
    type: int
    fields: dict


def unpack(payload: bytes) -> AppMessage:
    if len(payload) == 0:
        raise ValueError("empty payload")
    t = payload[0]
    body = payload[1:]

    if t == APP_HELLO:
        name = body.split(b"\x00", 1)[0].decode("utf-8", errors="replace")
        return AppMessage(t, {"name": name})

    if t == APP_META:
        if len(body) < 7:
            raise ValueError("APP_META too short")
        ts, status, ver = struct.unpack(">IBH", body[:7])
        return AppMessage(t, {"ts": ts, "status": status, "ver": ver})

    if t == APP_META_ACK:
        if len(body) < 2:
            raise ValueError("APP_META_ACK too short")
        (ver,) = struct.unpack(">H", body[:2])
        return AppMessage(t, {"ver": ver})

    if t == APP_CAL_RESULT:
        if len(body) < 4:
            raise ValueError("APP_CAL_RESULT too short")
        x, y = struct.unpack(">HH", body[:4])
        return AppMessage(t, {"x": x, "y": y})

    if t == APP_CAL_VISUAL:
        if len(body) < 6 + CAL_VISUAL_ZOOM_BYTES:
            raise ValueError(
                f"APP_CAL_VISUAL too short ({len(body)} < {6 + CAL_VISUAL_ZOOM_BYTES})"
            )
        x, y, bright, delta = struct.unpack(">HHBB", body[:6])
        zoom = bytes(body[6:6 + CAL_VISUAL_ZOOM_BYTES])
        return AppMessage(t, {
            "x": x, "y": y, "brightness": bright, "delta": delta,
            "zoom_4bit": zoom,
        })

    if t == APP_STATS:
        if len(body) < 12:
            raise ValueError("APP_STATS too short")
        tx, rx, crc_f, rtx, dll, baseline, delta = struct.unpack(">HHHHHBB", body[:12])
        return AppMessage(t, {
            "tx": tx, "rx": rx, "crc_fail": crc_f, "retrans": rtx,
            "dpll_loss": dll, "baseline": baseline, "delta": delta,
        })

    if t == APP_TEXT:
        return AppMessage(t, {"text": body.decode("utf-8", errors="replace")})

    if t == APP_BYE:
        return AppMessage(t, {"reason": body.decode("utf-8", errors="replace")})

    if t == APP_CHUNK:
        if len(body) < 3:
            raise ValueError("APP_CHUNK too short")
        msg_id, seq, total = body[0], body[1], body[2]
        data = body[3:]
        return AppMessage(t, {"msg_id": msg_id, "seq": seq, "total": total, "data": data})

    if t == APP_CHUNK_ACK:
        if len(body) < 1:
            raise ValueError("APP_CHUNK_ACK too short")
        msg_id = body[0]
        bitmap = body[1:]
        return AppMessage(t, {"msg_id": msg_id, "bitmap": bitmap})

    if t == APP_NACK:
        if len(body) < 2:
            raise ValueError("APP_NACK too short")
        return AppMessage(t, {"rejected_type": body[0], "reason": body[1]})

    raise ValueError(f"unknown app type 0x{t:02x}")


def fragment(app_type: int, payload: bytes, max_chunk: int = MAX_CHUNK_DATA,
             msg_id: int = 0) -> list[bytes]:
    """Split a large payload into APP_CHUNK frames.

    The reassembled payload is the raw bytes passed in — caller is
    responsible for preserving the app_type context out-of-band (e.g., by
    sending an APP_TEXT or APP_META chunk-group header first, or by
    agreeing on type via the session flow). For v1, fragment() is only
    used for APP_TEXT bodies; if other types need fragmentation we'll
    prefix the payload with the original type byte.
    """
    if max_chunk > MAX_CHUNK_DATA:
        raise ValueError(f"max_chunk {max_chunk} > {MAX_CHUNK_DATA}")
    if max_chunk < 1:
        raise ValueError("max_chunk must be >= 1")

    body = bytes([app_type]) + payload
    total = (len(body) + max_chunk - 1) // max_chunk
    if total > 255:
        raise ValueError(f"payload too large: {total} chunks > 255")

    chunks = []
    for seq in range(total):
        start = seq * max_chunk
        data = body[start:start + max_chunk]
        frame = pack_chunk(msg_id, seq, total, data)
        assert len(frame) <= MAX_SINGLE_FRAME, (
            f"chunk frame {len(frame)} bytes exceeds drift budget {MAX_SINGLE_FRAME}"
        )
        chunks.append(frame)
    return chunks


def reassemble(chunks: list[bytes]) -> tuple[int, bytes]:
    """Reassemble a complete set of APP_CHUNK frames.

    Returns (original_app_type, payload_bytes). Raises if incomplete or
    inconsistent (mismatched msg_id, missing seq, differing total).
    """
    if not chunks:
        raise ValueError("no chunks")

    parsed = [unpack(c) for c in chunks]
    for m in parsed:
        if m.type != APP_CHUNK:
            raise ValueError(f"not a chunk: type 0x{m.type:02x}")

    total = parsed[0].fields["total"]
    msg_id = parsed[0].fields["msg_id"]
    if any(m.fields["total"] != total for m in parsed):
        raise ValueError("inconsistent total")
    if any(m.fields["msg_id"] != msg_id for m in parsed):
        raise ValueError("inconsistent msg_id")

    by_seq = {m.fields["seq"]: m.fields["data"] for m in parsed}
    if set(by_seq.keys()) != set(range(total)):
        missing = sorted(set(range(total)) - set(by_seq.keys()))
        raise ValueError(f"missing chunks: {missing}")

    body = b"".join(by_seq[i] for i in range(total))
    if len(body) < 1:
        raise ValueError("reassembled body empty")
    return body[0], body[1:]


def missing_chunks(ack_bitmap: bytes, total: int) -> list[int]:
    """Return the list of chunk seqs NOT marked in the bitmap."""
    missing = []
    for seq in range(total):
        byte_idx = seq // 8
        bit = seq % 8
        if byte_idx >= len(ack_bitmap) or not (ack_bitmap[byte_idx] & (1 << bit)):
            missing.append(seq)
    return missing
