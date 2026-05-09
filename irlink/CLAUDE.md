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
| CAL_REQ | 0x05 | Request calibration. 1-byte payload, bit-flagged: bit 0 = `bidi` (responder swaps roles + scans back), bit 1 = `peer_scans` (responder is the scanner — we hold LEDs). Mutually exclusive. |
| CAL_ACK | 0x06 | Acknowledge calibration |
| CAL_DONE | 0x07 | Calibration complete. 6-byte payload: `[x_hi, x_lo, y_hi, y_lo, peak_b, grid_delta]` (TX pixel coords as seen by the scanner) |
| PING | 0x08 | Measure RTT |
| PONG | 0x09 | Ping response |
| RATE_CHANGE | 0x0A | Synchronize symbol rate between peers (2-byte BE payload) |

After CAL_DONE, the scanner side also ships an APP_CAL_VISUAL (`0x0E`) inside a regular MSG_DATA frame: 15 bytes total = `[0x0E, x_hi, x_lo, y_hi, y_lo, peak_b, delta_clamped, zoom_4x4 (8B)]`. The 4×4 zoom is a sub-area of the grid-delta map centered on the peak block (peak at cell (1,1)), 4-bit per cell. Used by the laptop-driven `cal_procedure --over-light` flow to render a visual confirmation for the cam1-sees-cam2 direction without RTSP into cam1. See `protocol/CLAUDE.md` for the wire format and `host/cal_procedure.py:render_cal_visual_zoom`.

Frame payload format: `[msg_type] [seq_num] [data...]`

Subcommands: `calibrate`, `calibrate-pixel`, `listen`, `connect`, `daemon-listen`, `daemon-connect`, `monocal`, `send`, `tx`, `tx-symbols`. Interactive cmds inside `connect`/`listen`: `send`, `send-hex`, `ping`, `cal`, `bicall`, `rate <ms>`, `stats`, `quit`.

Stdout contract (line-buffered via `fflush` after each line):
- `MSG-HEX:<hex>` — binary-safe DATA payload (use this in orchestrators)
- `MSG:<text>` — legacy/human DATA payload, truncates at `\x00`
- `STATS: tx=N rx=N crc=N rtx=N dll=N rate=N rung=N`
- `PROTO: ACK received for seq=N` / `PROTO: send failed`
- `PROTO: ACK wait extended (peer carrier Xms ago, ext=N)` — carrier-aware extension firing
- `RATE: ms=N rung=N reason=<probe-up|fallback|peer-initiated|probe-rollback|manual|split-brain-recovery>` — emitted on every rate change
- `PIXEL: x y peak_b grid_delta block_idx` — emitted by the scanner side after a `calibrate-pixel`, `cal`, or `bicall` (its own view of the peer's TX)
- `PEER-CAL: x y peak_b grid_delta` — emitted by the holder side after receiving CAL_DONE (the peer's view of US)
- `BICAL: phase 1 done`, `BICAL: got reverse CAL_REQ`, `BICAL: bidirectional cal complete` — bicall progress markers
- `PROTO: monocal — …` and `PROTO: peer requested monocal …` — monocal progress markers (REQUESTOR + RESPONDER sides). monocal also writes a structured JSON to `/run/monocal-status.json` for webui polling — see Monocal section below.

## Adaptive Symbol Rate

irlink self-tunes symbol rate within a session: start at `--speed N`, probe faster after PROBE_UP_AFTER (5) successful ACKed DATA frames, fall back slower after FALLBACK_AFTER (3) retransmit-exhausted sends. Rate ladder `[60, 80, 100, 120, 160, 200]` ms mirrors `host/config.py::RATE_LADDER_MS`.

Rate sync uses `MSG_RATE_CHANGE`: sender transmits `[rate_hi, rate_lo]` at OLD rate; peer auto-ACKs at OLD rate then switches. `ack_timeout_ms` recomputes on every rate change.

Split-brain recovery: if `rx_thread` sees no valid frame for `2 * ack_timeout_ms` (and at least one frame was decoded previously), force slowest rung. Peer hits this too — both reconverge at 200ms. `last_valid_frame_ms > 0` guard prevents firing during startup.

## Monocal — autonomous-cam1 calibration over light

Mirror of `cal`/`bicall` with **inverted roles**: REQUESTOR holds LEDs, RESPONDER scans. Designed for the `[laptop]→[cam2]→light→[cam1]` deployment topology where cam1 is autonomous (no IP path from laptop) and only needs to refresh its own `/opt/etc/calibration.json`.

**Trigger paths:**
```bash
# CGI-driven (preferred — used by webui MONOCAL CAM1 button):
curl -X POST -d 'coords=243,177' http://dacam2/x/monocal-trigger.cgi
# direct subcommand on cam2 (manual debug):
ssh dacam2 "/opt/bin/irlink monocal --pixel 243,177 --speed 160"
```

**Protocol:** new `peer_scans` flag (CAL_REQ payload[0] bit 1). When set, the responder runs `do_calibrate_pixel_to()` instead of holding LEDs; the requestor holds LEDs (1.5s OFF + 12s ON, same pattern as `handle_cal_request`). One round trip:
1. cam2 sends CAL_REQ(0x02) → cam1 sends CAL_ACK
2. cam2 holds LEDs OFF 1.5s + ON 12s
3. cam1 scans during the LED hold (writes its own `/opt/etc/calibration.json`)
4. cam1 sends CAL_DONE with 6-byte coord payload
5. cam2 receives, logs `PEER-CAL: x y b d`, writes `/run/monocal-status.json`, exits

**SYNC_BARRIER (timing fix):** `handle_monocal_request` waits until `cal_ack_sent_at + PEER_HOLD_MS (13500ms) + DRAIN_MARGIN_MS (1500ms)` before TXing CAL_DONE. This is the explicit fix for the bicall phase-2 pitfall where the scanner finishes before the holder's LED-on window ends and TXes into the holder's `tx_active=1` window. Total cycle ~50s at 160ms/sym (vs bicall's ~3.5 min).

**Status JSON (`/run/monocal-status.json`):** atomic write at every state transition. State machine on cam2:
```
spawning → starting → handshaking → monocal_req → monocal_ack_wait
  → monocal_holding_off → monocal_holding_on → monocal_awaiting_done
  → monocal_done | monocal_failed
```
Fields: `state`, `started_ms`, `updated_ms`, `ended_ms`, `ok`, `error`, `ack_received_ms`, `led_hold_started_ms`, `led_hold_ended_ms`, `peer_pixel: {x, y, peak_b, grid_delta}`, `speed_ms`. The webui polls cam2's `/x/monocal-status.cgi` for these and merges with cam1's `/x/cal-status.cgi` for a two-cam view.

**Backwards compat:** old peers see CAL_REQ payload `0x02` and check `data[0] == 1` for bidi → bidi=0 → fall through to `handle_cal_request(0)` (regular cal — both sides try to hold LEDs, neither scans). Deploy the new binary to BOTH cams before using monocal.

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

# Monocal — autonomous-cam1 cal: cam2 holds LEDs, cam1 scans, ~50s wall.
# cam1 must be running daemon-listen (rc.local autostarts it). Writes
# /run/monocal-status.json on cam2; updates cam1's /opt/etc/calibration.json.
irlink monocal --pixel 243,177 --speed 160
```

## AE Freeze During Communication

irlink freezes auto-exposure before any protocol exchange and unfreezes after. Critical because:
- The camera's own LED reflections cause massive AE swings (3-5s settle time)
- With AE frozen, raw block brightness is stable and predictable
- The peer's IR LED causes a clean delta (23-45 units) on the tracked block
- Between communications, AE runs normally to adapt to ambient light

Controlled via `/run/prudynt/ae_freeze` — irlink writes `1` before starting, `0` when done. Patched BrightnessMonitor in prudynt calls `IMP_ISP_Tuning_SetAeFreeze()`.

## ROI Calibration

Three calibration paths, in order of preference for the autonomous-cam1 deployment:

**1. On-camera pixel cal (`calibrate-pixel`, recommended).** Grid coarse → 6×6 stride-5 ROI sweep within the peak block. Auto-expands the search rectangle if the peak lands on its edge (handles IR LEDs straddling block boundaries). Writes `/opt/etc/calibration.json` (JFFS2-persistent). Used as the standalone CLI cal AND as the inner step of CAL_REQ over the link.
```bash
# Standalone (laptop SSH-drives both sides during a 4s+12s LED-on window):
ssh dacam2 "light ir850 on; light ir940 on; sleep 12; light ir850 off; light ir940 off" &
sleep 2  # let baseline phase finish first
ssh dacam1 "/opt/bin/irlink calibrate-pixel"
# stdout (cam1): PIXEL: 315 142 235 14 89  (x y peak_b grid_delta block_idx)
```

**2. Over-link cal (`cal` / `bicall` interactive cmds, `monocal` subcommand).** The CAL_REQ/CAL_ACK/CAL_DONE protocol triggers `do_calibrate_pixel` on whichever side is the scanner; CAL_DONE carries the result.
- `cal` — one-way, requestor scans, responder holds (~80s @ 160ms/sym)
- `bicall` — both directions via role-swap (`bidi` flag), both `/opt/etc/calibration.json` files refresh from one trigger (~3.5 min)
- `monocal` — one-way, **responder scans, requestor holds** (`peer_scans` flag) — for the autonomous-cam1 case where laptop only reaches cam2; only cam1's `/opt/etc/calibration.json` updates (~50s)

See "Monocal" section above for the full state machine and CGI integration.

**3. Grid-only legacy (`calibrate`).** Block-only, no pixel refinement. Kept for backwards compatibility; prefer `calibrate-pixel`.

**Host-side RTSP cal:** `host/cal_procedure.py` — see `../host/CLAUDE.md`. Uses RTSP frame-diff; can underreport delta when AE compensates (the on-camera grid sees +67 delta where the laptop reports +24). Use the on-camera path when the laptop's cal looks suspiciously weak.

## Boot Automation

Both cameras run `/etc/rc.local` at boot (via `S94rc.local`):
1. Mounts `/opt` JFFS2 partition if not mounted
2. Stops stock prudynt, starts patched version
3. Opens IR cut filter, kills daynightd
4. **(cam1)** Reads `tx_pixel` from `/opt/etc/calibration.json` and starts `irlink daemon-listen --pixel X,Y --speed 160` in background (logs to `/var/log/irlink-boot.log`). Skips if no calibration.json exists. Backup of pre-edit script lives at `/etc/rc.local.bak`.

## Deploying

```bash
cd irlink && make deploy                 # builds + SCPs to both cameras
```

Toolchain: `thingino-firmware/output/stable/wyze_cam3_t31x_gc2053_atbm6031-3.10.14-musl/host/bin/mipsel-linux-gcc`.

## CGI endpoints (`irlink/cgi/`)

Small read-only shell-CGIs deployed to `/var/www/x/` on each camera. They give the laptop `host/cam_status.py` lightweight HTTP access to per-camera state, replacing the SSH-`cat` pattern that overloads sshd during high-rate polling (notably `aim_assist.read_grid` at 4 Hz).

```bash
# One-time deploy on each camera:
scp -O irlink/cgi/cal-status.cgi       root@dacamN:/var/www/x/
scp -O irlink/cgi/brightness-grid.cgi  root@dacamN:/var/www/x/
scp -O irlink/cgi/ae-freeze.cgi        root@dacamN:/var/www/x/
scp -O irlink/cgi/proc-status.cgi      root@dacamN:/var/www/x/
scp -O irlink/cgi/gpio-state.cgi       root@dacamN:/var/www/x/
# Monocal CGIs go to cam2 ONLY (cam2 is requestor / laptop-tethered;
# cam1 is responder, no trigger CGI needed):
scp -O irlink/cgi/monocal-trigger.cgi  root@dacam2:/var/www/x/
scp -O irlink/cgi/monocal-status.cgi   root@dacam2:/var/www/x/
ssh dacamN "chmod 755 /var/www/x/cal-status.cgi /var/www/x/brightness-grid.cgi /var/www/x/ae-freeze.cgi /var/www/x/proc-status.cgi /var/www/x/gpio-state.cgi"
ssh dacam2 "chmod 755 /var/www/x/monocal-trigger.cgi /var/www/x/monocal-status.cgi"
# `make deploy` (Makefile target) ships all of the above in one command.
# IMPORTANT: must be 755, not just `chmod +x`. The default umask yields 077,
# so `chmod +x` produces mode 700 — uhttpd then serves the CGI as 403 Forbidden.
```

| Endpoint | Reads | Consumed by |
|----------|-------|-------------|
| `/x/cal-status.cgi` | `/run/irlink-status.json` (irlink writes atomically per protocol event) | `cam_status.CamStatusClient.get()` |
| `/x/brightness-grid.cgi` | `/run/prudynt/brightness_grid` (242 ints: ts + 240 blocks) | `cam_status.CamStatusClient.get_brightness_grid()` → `aim_assist.read_grid()` |
| `/x/ae-freeze.cgi` GET | `/run/prudynt/ae_freeze` (single int 0 or 1) | `cam_status.CamStatusClient.get_ae_freeze()` → `aim_assist.preflight()` |
| `/x/ae-freeze.cgi` POST `value=0\|1` | atomic write (temp+rename) of single char into `/run/prudynt/ae_freeze`; rejects anything else with HTTP 400 | `cam_status.CamStatusClient.set_ae_freeze()` → `aim_assist.set_ae_freeze()`, `cam_setup.sh:ae_freeze_set()` |
| `/x/proc-status.cgi` | `pgrep irlink \| wc -l`, `pgrep daynightd \| wc -l`, `pidof prudynt-patched` → JSON `{irlink:N, daynightd:N, prudynt:N, ts_s:T}` | `cam_status.CamStatusClient.get_proc_status()` → `aim_assist.preflight()`, `aim_assist.restart_cam1_daemon()` |
| `/x/json-imp.cgi` POST `{"cmd":<C>,"val":<V>}` | Stock Thingino IMP control (daynight, color, ircut, ir850, ir940) | `cam_status.CamStatusClient.imp_cmd()` → `aim_assist.turn_leds_on/force_leds_off`, `cal_procedure.leds_on_hold/_off`, `cam_setup.sh:imp_cmd` |
| `/x/gpio-state.cgi` | `gpio read 47` (ir850) + `gpio read 49` (ir940) → JSON `{ir850, ir940}` | `cam_status.CamStatusClient.get_led_state()` → `webui` real-time IR LED reflection |
| `/x/monocal-trigger.cgi` POST `coords=X,Y[&speed=N]` (cam2 only) | spawns `irlink monocal --pixel X,Y --speed N` via `setsid nohup`; refuses 409 if any `irlink` already running; pre-writes `/run/monocal-status.json` so the first poll sees `state=spawning` | `cam_status.CamStatusClient.start_monocal()` → `webui` MONOCAL CAM1 button |
| `/x/monocal-status.cgi` (cam2 only) | `cat /run/monocal-status.json`; HTTP 503 + `{state: transient}` on mid-rename empty file; HTTP 200 + `{state: never_run}` if file missing | `cam_status.CamStatusClient.get_monocal_status()` → `webui` M1..M5 step polling |

All endpoints share the cookie auth from `/x/login.cgi`; the laptop client logs in once and replays the cookie.

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
- **`calibrate-pixel` peak-on-edge auto-expansion.** The 6×6 stride-5 ROI sweep within the picked grid block can land its peak on the rectangle edge if the actual IR center sits near a block boundary. `find_peak_pixel_in_block` then expands the swept rectangle by one stride in each edge-adjacent direction and rescans only the new strips, capped at `MAX_EDGE_EXPANSIONS = 4`. Adds ~700 ms when triggered; 0 ms when peak is interior. Without this, a peak at xmax=315 in block 89 gets reported truthfully but offers no proof it isn't actually further right. With it, we scan x=320 column too and confirm.
- **Bicall split-brain trip during long CAL_DONE wait.** When the responder's wait_for_msg(CAL_DONE) takes longer than `2 × ack_timeout_ms` (e.g. CAL_DONE was lost or scanner's pixel cal couldn't decode our TX), the responder hits split-brain recovery and force-drops to the slowest rate rung (200 ms) — but the peer is still at 160 ms. Phase 2 then deadlocks at CAL_ACK timeout because the rates desynced silently. Fix is to either (a) suppress split-brain during CAL_REQ flows, or (b) require split-brain to send a RATE_CHANGE before changing locally. **Open issue.**
- **Bicall phase 2 timing margin is thin.** Holder's LED hold = 1.5 s OFF + 12 s ON = 13.5 s total. Scanner's `do_calibrate_pixel_to` takes ~12 s (1 s baseline + 4 s grid + ~6 s ROI sweep + edge expansion). If the scanner's scan finishes more than ~1.5 s before the holder's LED hold ends, the scanner's CAL_DONE TX starts during the holder's `tx_active=1` window and the holder misses the preamble (1.28 s @ 160 ms). Phase 1 always works because the responder's wait window starts when its LED hold ends, aligned with the scanner's TX start. Phase 2 is more fragile — if you lengthen the pixel sweep (e.g., add stride-1 sub-block refinement), bump the holder's hold to keep the margin.
- **AE freeze stuck at 1 after `killall -9 irlink`.** The `-9` skips irlink's SIGTERM cleanup that writes `0` to `/run/prudynt/ae_freeze`. Subsequent operations (e.g. `cal_procedure.py`) silently fail because exposure is still locked to whatever it was when irlink was killed. `cam_setup.sh` now detects and resets this; manual fix is `echo 0 > /run/prudynt/ae_freeze`. Prefer `kill` (SIGTERM) over `kill -9` so cleanup fires.
- **busybox `date +%s%3N` doesn't expand `%N`** — emits seconds only (Thingino busybox 1.37). The `monocal-trigger.cgi` pre-write originally had `started_ms:$(date +%s%3N)` and produced bogus `started_ms:1776061326` (seconds, not ms). Fix: `NOW_MS=$(date +%s)000` for one-second-granularity timestamps. The `irlink monocal` binary's first `write_monocal_status()` overwrites within ~hundreds of ms with real `clock_gettime` ms, so the bad CGI pre-write only shows for one webui poll tick.
- **Monocal CAL_REQ flag dispatch must be bit-checked, not equality-checked.** Old code used `int bidi = (data[0] == 1) ? 1 : 0` — that catches bidi but interprets `0x02` (peer_scans) as bidi=0 → falls through to `handle_cal_request(0)` (regular cal — both sides hold LEDs, neither scans). Fix: `int flags = data[0]; int bidi = !!(flags & 0x01); int peer_scans = !!(flags & 0x02);`. Both `interactive_mode` and `daemon_mode` dispatch sites updated. **Cam1 must run the new binary** before any monocal trigger from cam2; old cam1 silently degrades to a no-op cycle.
- **Monocal SYNC_BARRIER vs bicall race.** `handle_monocal_request` waits until `cal_ack_sent_at + 13500ms (peer hold) + 1500ms (drain margin)` before TXing CAL_DONE. This guarantees the responder's CAL_DONE preamble doesn't collide with the requestor's `tx_active=1` LED-on window — the explicit fix for the existing "Bicall phase 2 timing margin is thin" pitfall above. If you tune the requestor's hold duration in `do_monocal_request`, change `PEER_HOLD_MS` in `handle_monocal_request` to match.
