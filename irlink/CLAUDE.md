# irlink — On-Camera Half-Duplex Transceiver

The `irlink` binary is the combined half-duplex transceiver with a TCP-like protocol layer. It runs on-camera, reading brightness from BrightnessMonitor (patched prudynt-t) and toggling both IR LEDs (850nm + 940nm) simultaneously via sysfs GPIO for maximum signal strength.

Related: `../protocol/CLAUDE.md` for frame/app-layer/FEC details; `../host/CLAUDE.md` for the SSH-driven orchestrator (`host/session.py`).

## Protocol Message Types

| Type | Code | Purpose |
|------|------|---------|
| SYN | 0x01 | Initiate connection |
| SYN_ACK | 0x02 | Acknowledge connection |
| ACK | 0x03 | Acknowledge data (with seq number) |
| DATA | 0x04 | Payload data |
| CAL_REQ | 0x05 | Request calibration |
| CAL_ACK | 0x06 | Acknowledge calibration |
| CAL_DONE | 0x07 | Calibration complete |
| PING | 0x08 | Measure RTT |
| PONG | 0x09 | Ping response |
| RATE_CHANGE | 0x0A | Synchronize symbol rate between peers (2-byte BE payload) |

Frame payload format: `[msg_type] [seq_num] [data...]`

Subcommands: `calibrate`, `listen`, `connect`, `daemon-listen`, `daemon-connect`, `send`, `tx`, `tx-symbols`. Interactive cmds inside `connect`/`listen`: `send`, `send-hex`, `ping`, `cal`, `rate <ms>`, `stats`, `quit`.

Stdout contract (line-buffered via `fflush` after each line):
- `MSG-HEX:<hex>` — binary-safe DATA payload (use this in orchestrators)
- `MSG:<text>` — legacy/human DATA payload, truncates at `\x00`
- `STATS: tx=N rx=N crc=N rtx=N dll=N rate=N rung=N`
- `PROTO: ACK received for seq=N` / `PROTO: send failed`
- `PROTO: ACK wait extended (peer carrier Xms ago, ext=N)` — carrier-aware extension firing
- `RATE: ms=N rung=N reason=<probe-up|fallback|peer-initiated|probe-rollback|manual|split-brain-recovery>` — emitted on every rate change

## Adaptive Symbol Rate

irlink self-tunes symbol rate within a session: start at `--speed N`, probe faster after PROBE_UP_AFTER (5) successful ACKed DATA frames, fall back slower after FALLBACK_AFTER (3) retransmit-exhausted sends. Rate ladder `[60, 80, 100, 120, 160, 200]` ms mirrors `host/config.py::RATE_LADDER_MS`.

Rate sync uses `MSG_RATE_CHANGE`: sender transmits `[rate_hi, rate_lo]` at OLD rate; peer auto-ACKs at OLD rate then switches. `ack_timeout_ms` recomputes on every rate change.

Split-brain recovery: if `rx_thread` sees no valid frame for `2 * ack_timeout_ms` (and at least one frame was decoded previously), force slowest rung. Peer hits this too — both reconverge at 200ms. `last_valid_frame_ms > 0` guard prevents firing during startup.

## Usage

```bash
# Calibrate: toggle the other camera's LED during the 4-second window
irlink calibrate
# Output: block index (e.g., 212)

# Listen with pixel-level ROI (recommended, requires host calibration first)
irlink listen --pixel 385,178 --speed 120

# Listen with grid block (legacy, lower SNR)
irlink listen --block 212

# Raw TX — no handshake, for use with pixel_rx or passive monitoring
irlink tx "HELLO" --speed 120
irlink tx "HELLO" --pixel 385,178 --speed 120

# Connect and send a message (with protocol handshake)
irlink send "HELLO" --pixel 385,178 --speed 120

# Daemon mode (auto-responds to incoming connections)
irlink daemon-listen --pixel 385,178 --speed 120

# Interactive mode (type commands after handshake)
irlink connect --pixel 385,178 --speed 120
# Then: send <text>, ping, cal, quit

# Adjust ROI size (default 15, range 3-31)
irlink listen --pixel 385,178 --roi-size 21 --speed 120
```

## AE Freeze During Communication

irlink freezes auto-exposure before any protocol exchange and unfreezes after. Critical because:
- The camera's own LED reflections cause massive AE swings (3-5s settle time)
- With AE frozen, raw block brightness is stable and predictable
- The peer's IR LED causes a clean delta (23-45 units) on the tracked block
- Between communications, AE runs normally to adapt to ambient light

Controlled via `/run/prudynt/ae_freeze` — irlink writes `1` before starting, `0` when done. Patched BrightnessMonitor in prudynt calls `IMP_ISP_Tuning_SetAeFreeze()`.

## ROI Calibration

**On-camera (grid-based, legacy):** 20x12 grid (240 blocks) finds the transmitter's block:
```bash
irlink calibrate    # toggle peer LED during 4s window
```

**Host-side (pixel-level, recommended):** see `../host/CLAUDE.md` — delta +120-150 on-camera vs +10-15 with the grid.

## Boot Automation

Both cameras run `/etc/rc.local` at boot (via `S94rc.local`):
1. Mounts `/opt` JFFS2 partition if not mounted
2. Stops stock prudynt, starts patched version
3. Opens IR cut filter, kills daynightd

## Deploying

```bash
cd irlink && make deploy                 # builds + SCPs to both cameras
```

Toolchain: `thingino-firmware/output/stable/wyze_cam3_t31x_gc2053_atbm6031-3.10.14-musl/host/bin/mipsel-linux-gcc`.

## Pitfalls

- **Adaptive rate: split-brain after probe-up ACK loss.** When a side probes up via `MSG_RATE_CHANGE`, the peer auto-ACKs at the OLD rate. If that ACK doesn't decode on the originator, the peer has switched to the new rate but the originator still thinks it's at the old rate. Neither side immediately notices. Split-brain recovery catches this after `2 × ack_timeout_ms` (~5 min at 160ms) by forcing the slowest rung. The recovery gate must check `last_valid_frame_ms` **only** — do NOT gate on `last_carrier_ms` as well, because a peer stuck at a mismatched rate still produces carrier (their TX is visible as activity) but never produces a valid decode, which IS the signal of split-brain.
- **Adaptive rate: split-brain startup false-trigger.** A fixed 15s split-brain window fires during handshake — a SYN→SYN_ACK→ACK round trip at 160ms/sym is ~60-90s. Two guards together prevent this: `last_valid_frame_ms > 0` (never fires before the first decode) AND scaling the window to `2 × ack_timeout_ms` so handshake flows fit inside.
- **Adaptive rate: on-camera sym rate floor is 160ms.** BrightnessMonitor runs ~18fps → ≥2 samples/sym requires ≥110ms; reliability demands ≥160ms. `pick_initial_rate_ms` maps Δ=180+ to 70ms for RTSP (30fps), but `host/session.py` clamps to ≥160ms before passing to on-camera irlink. Adaptive probe-up pushes below 160ms only in-session once link is proven.
- **Adaptive rate: probe-up threshold fires at the NEXT send.** `success_at_rate >= PROBE_UP_AFTER (5)` is checked before each DATA TX. With exactly 5 sends in a session, probe-up never fires (need a 6th to trigger the check). Include a STATS or CAL_RESULT round trip if you want to exercise probe-up in the default flow.
- **AE freeze is required for half-duplex protocol.** Without AE freeze, the camera's own LED reflections cause massive AE swings that take 3-5 seconds to settle after TX ends. With `IMP_ISP_Tuning_SetAeFreeze(ENABLE)`, exposure locks and raw block brightness changes are instantaneous and predictable.
- **AE over-compensates for IR LEDs (when not frozen).** When the IR LED turns ON, the ISP reduces exposure, making the whole-frame MEAN brightness DROP (inverted from expected). The IR block gets brighter, but everything else gets darker. Use residual-based detection (block - mean) to cancel AE, not absolute brightness.
- **Self-reflection dominates raw brightness.** When a camera toggles its own IR LED, reflections off walls/ceiling cause the tracked block to change by 8-10 units (vs 23-45 from the peer). With AE frozen, these reflections are small and consistent — not a problem. With AE active, they trigger AE compensation that ruins subsequent reception.
- **Consecutive-frame filter prevents false activity detection.** Single-frame brightness spikes (self-reflection or noise) trigger false IDLE→ACTIVE transitions. Requiring 2 consecutive frames above threshold eliminates these (reduced from 3 with AE freeze).
- **Inter-message gap must exceed SETTLE_MS.** Back-to-back transmissions (ACK immediately followed by DATA) merge into one continuous activity period for the receiver. The gap must be at least SETTLE_MS (400ms) + 100ms margin so the receiver detects "end of TX" and decodes each message separately.
- **RX thread stack overflow on MIPS/musl.** The default pthread stack (~128KB on musl) overflows with large local arrays. `sample_t samples[8192]` (~128KB) must be declared `static`, not stack-local.
- **On-camera DPLL needs a slower symbol rate than RTSP-side.** RTSP runs ~30fps (3.6 samples/sym at 120ms); on-camera BrightnessMonitor runs ~18fps (2.2 samples/sym at 120ms). Same payload that decodes via `pixel_rx.py` at 120ms/sym can fail in `irlink listen --pixel` at 120ms. Bump on-camera flows to **160ms/sym** (default in `host/session.py`).
- **`irlink` post-TX baseline-capture races peer's TX.** During the 500ms post-self-TX settling window, the original code set `baseline = current_brightness` every sample. If the peer started TXing during that window (fast turnaround in half-duplex protocol), baseline got captured at the peer's BRIGHT state. Then post-peer-TX the brightness returned to ambient ~80 units away, with `diff` permanently above SETTLE_MS reset threshold — RX stuck in ACTIVE state, never decoded. Fixed by tracking **min** over the settle window and committing baseline = min after the window closes.
- **`wait_for_msg` wiped peer DATA as "stale".** When both sides have in-flight DATA sends, each enters `wait_for_msg(MSG_ACK)`. The original code set `rx_msg.valid = 0` for any non-matching message, silently dropping the peer's DATA without ACKing it. Result: mutual deadlock. Fixed by **inline-ACKing peer DATA** (and inline-PONGing peer PING) inside `wait_for_msg` before continuing the wait.
- **`do_send` used `strlen()` on its text arg** — binary app payloads with `\x00` got truncated. Use `do_send_bytes(data, len)` for any caller that may pass binary bytes. The `send-hex` interactive command uses this path.
- **`MSG:` output truncates at `\x00`.** App-layer payloads are binary (start with a 1-byte type code, may contain NULs). The `send-hex` path is paired with a **`MSG-HEX: <hex>`** stdout line that's binary-safe. The original `MSG: <text>` is still emitted for human/legacy use but should not be parsed by orchestrators handling typed payloads.
- **Thingino busybox has no `stdbuf`.** Don't try to wrap remote commands in `stdbuf -oL -eL`. `irlink` calls `fflush(stdout)` after every `MSG-HEX/MSG/STATS/PONG` line; stderr is line-buffered by default.
- **Premature ACK timeout on top of in-flight peer DATA causes mutual collision.** If A is mid-TX of a long DATA while B's ACK timeout fires, B retransmits — now both sides are TXing at once. Fixed via **carrier-aware ACK timeout**: `irlink.c` exposes `last_carrier_ms` (updated by `rx_thread` whenever it sees `diff >= MIN_BRIGHTNESS_DELTA`), and `wait_for_msg` extends the deadline by another `ack_timeout_ms` (capped at `ACK_HARD_CEILING_X * ack_timeout_ms`, currently 3×) whenever the timeout fires within `CARRIER_RECENT_MS` (1.5s) of a peer carrier sample.
- **`interactive_mode` main thread is blocked in `fgets(stdin)` and does NOT wake on `rx_cond`.** Originally the loop processed `rx_msg.valid` only AFTER each stdin command — meaning incoming DATA sat un-ACKed until the orchestrator happened to issue another command to that camera. **Fix: `rx_thread` auto-ACKs DATA and auto-PONGs PING** — protocol-layer ACK is now decoupled from main-thread I/O scheduling. Code in `wait_for_msg`, `interactive_mode`, and `daemon_mode` no longer ACKs DATA themselves (would double-ACK). CAL_REQ stays main-handled because it has to hold the LED on for several seconds.
- **`ack_timeout_ms` sized for one max-frame round-trip, not multi-frame.** Formula: `450 * symbol_ms + 5000`. At 160ms/sym → ~77s base, with carrier-aware extension up to 3× = ~230s ceiling. Sized for one 16-byte DATA (~324 sym) + one ACK (~84 sym) round-trip with slack for AE settle and decode. Don't bump the base — bump `ACK_HARD_CEILING_X` instead, so dead-link retransmits stay fast while legitimate slow flows still complete.
- **`irlink --pixel` writes ROI config to `/run/prudynt/roi_config`.** BrightnessMonitor reads this every frame and outputs pixel ROI to `/run/prudynt/brightness_roi`. The config persists until overwritten or prudynt restarts. Switching between `--pixel` and `--block` leaves the old config file — harmless, since `read_brightness()` checks `pixel_x >= 0` first.
- **Whole-frame mean is useless for ROI detection.** At 2-3 feet, the IR LEDs occupy ~1 grid block out of 240. Mean delta is ~1-3 units (noise level), but the specific block delta is 30-100+ units. Always calibrate and track the specific block/pixel.
- **Cameras too close (<6 inches) gives poor signal.** The IR LEDs flood the entire sensor when very close, and the image is out of focus (min focus ~50cm). Best at 2-3 feet where the LEDs form a focused spot.
- **GPIO mux state persists.** Running `tx_pwm` (legacy) switches GPIO 49 to PWM function mode. If the binary exits without resetting, the pin stays in PWM mode and GPIO commands have no effect. Reset with `gpio-diag PB17 func 1` or reboot.
