# protocol/ — Wire Format, Framing, FEC, App Layer

Pure-Python modules that define the wire format and app-layer message types. No I/O — consumed by `host/pixel_rx.py`, `host/tx_resync.py`, `host/session.py`, and the test suite. On-camera side (`irlink`) implements the same framing in C.

- `frame.py` — Classic Manchester framing + `encode_frame_symbols_with_resync` / `encode_frame_symbols_with_resync_fec` / `strip_resync_symbols` / `symbols_to_hex`
- `manchester.py` — bit↔symbol conversion
- `crc.py` — CRC-8/CCITT (poly 0x07)
- `app.py` — Typed app messages on top of DATA; `fragment()`/`reassemble()` for >16B payloads

Decoders live in `../experiments/resync_decoder.py` (used by `host/pixel_rx.py --decoder {resync,resync-fec}`).

## Manchester Encoding (IEEE 802.3)

- `0` → `[1, 0]` (falling edge)
- `1` → `[0, 1]` (rising edge)

Self-clocking — mid-bit transition on every bit.

## Classic Frame Format

```
[PREAMBLE 8] [SYNC 8] [LENGTH 8] [PAYLOAD N×8] [CRC-8] [POSTAMBLE 4] → Manchester encoded
```

- **Preamble**: `10101010` — clock recovery
- **Sync word**: `11001011` — frame boundary marker
- **Length**: payload byte count (0-255)
- **CRC-8/CCITT**: polynomial 0x07, covers length + payload
- **Postamble**: `1010` — clean termination

## Resync-Framed Format (opt-in, for long messages)

For frames that would otherwise exceed the DPLL's drift budget (~80 symbols), the TX side interleaves short resync preambles between data chunks. Wire layout (all in symbol space, after Manchester encoding):

```
[PREAMBLE 16 syms] [SYNC 16 syms]
[CHUNK chunk_syms] [RESYNC resync_syms]  ← 16 raw 1010... symbols
[CHUNK chunk_syms] [RESYNC resync_syms]
...
[CHUNK last ≤chunk_syms]  ← no trailing resync
```

Default: `chunk_syms=48`, `resync_syms=16`. The resync block is raw alternating `1010…` at the symbol level — edges every symbol time let the DPLL re-lock as easily as the initial preamble. Built via `encode_frame_symbols_with_resync`, transmitted via `irlink tx-symbols <hex>` (host encodes; C just toggles GPIO per symbol). Decoded by `experiments.resync_decoder.decode_samples_resync`.

**Empirical results** (host RTSP @ 30fps, indoor 2ft, Δ~200):
- 43-char pangram: 100% decode at 70/80/90/100/110 ms/sym (≥2.1 samples/sym). Fails at 60ms (1.8 samples/sym).
- 82-char double-pangram (no FEC): 80ms gives 3 wrong chars — marginal (noise-limited, not drift-limited).
- Best raw rate: **4.91 bps @ 70ms/sym** for single-frame messages up to ~50 chars.
- **chunk_syms sweep** (43-char, 80ms/sym): chunk=48 → 4.30 bps; chunk=128 → 5.12 bps; chunk=1024 → **5.66 bps**. At high SNR the mid-frame resync blocks are pure overhead. Resync is a safety margin for marginal-SNR / through-window links, not a throughput win.

## Reed-Solomon FEC (opt-in wrapper around resync framing)

`encode_frame_symbols_with_resync_fec` wraps the payload in Reed-Solomon before the resync framing. Receiver in `experiments.resync_decoder.decode_samples_resync_fec` (wired into `pixel_rx.py --decoder resync-fec`).

Wire layout inside the frame:
- LENGTH byte = ORIGINAL message length (not the encoded length)
- PAYLOAD = `msg_bytes + RS_parity` — output of `RSCodec.encode(msg_bytes)`. For RS(n,k) and input L bytes, encoded = `L + ceil(L/k)*(n-k)` bytes.
- CRC-8 over `[LENGTH || encoded_payload]` — on-wire bytes, not the original message.

Default RS(15,11): 4 parity bytes per 11 data bytes → ~36% overhead, corrects 2 random byte errors (or 4 erasures) per 15-byte block.

**Erasure-aware decode**: invalid Manchester pairs (e.g. `(1,1)` from near-threshold noise) are placed in bit=0 at that position AND the containing byte is marked as an erasure. `RSCodec.decode(..., erase_pos=[...])` corrects up to **2T erasures per block** vs T random errors.

**Empirical results**: 82-char pangram at 80ms/sym with Δ≈95 (half the SNR that lets non-FEC resync work): non-FEC fails with ~15 bit errors; RS-with-erasures recovers cleanly. 36% TX-time overhead for reliable decode in marginal SNR.

## Application Layer (`app.py`)

Typed protocol sitting on top of irlink's `MSG_DATA` frames. Reserves `payload[0]` as an app-msg-type byte; `payload[1:]` is type-specific. No changes to irlink's protocol-layer semantics — just structured content inside DATA.

| Code | Name | Body | Purpose |
|------|------|------|---------|
| 0x01 | APP_HELLO | `name\0` | Initiator announces identity (≤14 chars) |
| 0x02 | APP_META | `ts(4) status(1) ver(2)` | Compact node state — fixed 7B body |
| 0x03 | APP_META_ACK | `ver(2)` | Receipt for META |
| 0x04 | APP_CAL_RESULT | `x(2) y(2)` | "I see your LED at this pixel" |
| 0x05 | APP_STATS | `tx(2) rx(2) crc(2) rtx(2) dll(2) baseline(1) delta(1)` | Link-health snapshot |
| 0x06 | APP_TEXT | UTF-8 (≤15B) | Single-frame app payload |
| 0x07 | APP_BYE | optional reason | Clean teardown |
| 0x08 | APP_CHUNK | `msg_id(1) seq(1) total(1) data(≤12)` | Fragment of a longer payload |
| 0x09 | APP_CHUNK_ACK | `msg_id(1) bitmap(...)` | Selective ACK — bitmap of received chunk seqs |
| 0xFE | APP_NACK | `rejected_type(1) reason(1)` | Receiver rejected the message |

### Drift-budget rule (16-byte single-frame ceiling)

Every single-frame app message must fit in **16 bytes total** (1 type byte + ≤15 data bytes). Empirical on-camera DPLL lock budget at 120ms/sym — see pitfall below. Larger payloads call `fragment()` which produces APP_CHUNK frames sized exactly to the budget; receiver reassembles via `reassemble()` and ACKs missing chunks via the bitmap.

The pack functions enforce the budget — `pack_text("..." * 20)` raises `ValueError`. Tests in `tests/test_app.py` assert every single-frame type stays ≤16B.

## Pitfalls

- **DPLL clock drift was the long-frame failure mode (now mitigated).** Live test at 120ms/sym, indoor, Δ~85: 16-byte payloads decoded 100%, 25-byte 50%, 43-byte 0%. Failure mode was "valid Manchester for ~80 syms then illegal pairs mid-frame" — DPLL lost lock, not bit errors from low SNR. **FEC doesn't help** — it corrects flipped bits, not lost symbol-boundary lock. Two mitigations exist:
  1. **Fragment to short frames** (`app.py` — each frame gets a fresh preamble, DPLL re-locks). 16-byte ceiling enforced. Use for half-duplex reliable protocol.
  2. **Mid-frame resync markers** (`frame.py:encode_frame_symbols_with_resync`). Injects 16 raw `1010...` symbols every 48 data symbols; baseline DPLL's existing edge-correction re-locks on them. Lets 43-char single-frame messages decode at 70-110ms/sym with 100% success.
- **New ceiling is isolated symbol noise, not drift.** At 80+ chars a handful of samples land near threshold and extract to wrong bits; `_manchester_decode_tolerant` in `experiments/resync_decoder.py` catches 1-3 bit errors via pair-candidate enumeration. Many "wrong" bits come from valid-looking pairs where the symbol was extracted as 1 when TX sent 0 — tolerance can't detect those. Fix path (not implemented): probabilistic symbol decision weighted by brightness distance from threshold.
- **`encode_frame_symbols_with_resync` operates at SYMBOL level, not data-bit level.** Resync blocks are injected AFTER Manchester encoding — raw `1010…` sequences, NOT `0xAA` data bits (which would Manchester-encode to `0110…` at symbol level). Receiver must strip resync blocks from the symbol stream BEFORE Manchester decoding — `strip_resync_symbols` does this. If you Manchester-decode the full stream first, each resync block injects 8 zero-bit garbage into the bit stream, CRC fails. `chunk_syms` MUST be a multiple of 16 (one Manchester data byte) so resync boundaries align with byte boundaries.
- **DPLL `first_edge_t` skips one symbol (off-by-one at start).** The baseline DPLL in `pixel_rx.py` / `experiments/resync_decoder.py` searches for the first rising edge, then places `phase = first_edge + T*phase_init_off`. For `phase_init_off=0.5` this centers on symbol 1 (the first HIGH symbol), not symbol 0 (the initial LOW quiet period). Extracted symbol stream is offset by 1 relative to the TX schedule. The sliding sync-word search handles this automatically — sync lands at position 15 instead of 16 — but code comparing against an exact expected stream must account for it. Don't try to "fix" by prepending a zero; you'll mis-align the sync search when the first real symbol is HIGH for some other reason.
- **DPLL truncates 1-2 symbols at start AND end; hardware trailer masks this, synthetic tests don't.** The DPLL's `while phase < times[-1] - T*0.3` termination cuts off the last 1-2 symbols if the signal ends exactly at the final symbol's rising edge. On hardware, irlink's C-side `transmit_symbols` adds trailer `(0,1,0)` + 400ms SETTLE_MS padding, so there are plenty of samples past the frame. In pure-Python synthetic tests, the signal ends at frame boundary and decode falls 1 sym short. Add explicit trailing samples in synthetic tests.
- **Resync at high SNR is safety margin, not throughput.** At Δ≈200 indoors, `chunk_syms=1024` (effectively no mid-frame resync for a 43-char message) gives 5.66 bps vs chunk=48's 4.30 bps. Use `chunk=128-256` as a default; drop to `chunk=48` only for through-window / marginal-SNR links. The original "16-byte drift wall" came from a Δ≈85 test where SNR was marginal.
- **RS-with-erasures corrects 2× what RS-random corrects.** `_decode_manchester_with_erasures` in `experiments/resync_decoder.py` marks invalid Manchester pairs as byte-level erasures and passes them to `RSCodec.decode(..., erase_pos=...)`. For RS(15,11) this lifts the correction ceiling from 2 byte errors per block (random) to 4 byte erasures (localized). Key lever making FEC practical on real captures — any `(1,1)` or `(0,0)` is unambiguously bad and localizable.
- **FEC length byte is ORIGINAL length, not encoded length.** Receiver reads LENGTH=orig_len then computes expected on-wire payload size via `_rs_encoded_size(orig_len, rs_n, rs_k)`. Designs where LENGTH is encoded-size lose the ability to tell the RS decoder how many data bytes to return. Don't swap them.
- **RS max message size at RS(15,11) is 187 bytes.** Encoded output = `L + ceil(L/11)*4`. At L=187, encoded=255. At L=188, encoded=260 — exceeds the 8-bit wire-size cap. `encode_frame_symbols_with_resync_fec` raises ValueError past that. For longer messages, use larger RS block sizes (RS(31,27), RS(63,55)) or fall back to APP_CHUNK fragmentation.
