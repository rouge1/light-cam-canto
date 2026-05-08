# host/ — Host-Side Orchestration, RX, Calibration

Laptop-side Python drivers: RTSP receivers, SSH-based orchestrators, calibration, TX drivers. All app logic runs here; cameras just run `irlink`.

- `config.py` — Camera IPs, GPIO pins, timing constants
- `ssh.py` — SSH/SCP wrappers (uses `scp -O`)
- `cam_setup.sh` — Camera pre-flight (prudynt, night mode, ircut, LEDs, AE freeze)
- `cam_status.py` — **Single per-host HTTP client** (`CamStatusClient`) for every camera CGI. Cookie-cached, auto-relogin on 403, fail-soft `None` returns. All Phase 1-5 endpoints route through here. See "Camera HTTP Client" below.
- `cal_procedure.py` — Pixel-level calibration: diff LED-on/off RTSP frames to find TX. LED on/off goes through web API (`imp_cmd`).
- `calibration.json` — Saved calibration data (TX pixel coords per camera pair)
- `pixel_rx.py` — RTSP pixel receiver (DPLL decode, 120ms/symbol default). `--decoder {baseline,resync,resync-fec}`. `--dump-samples` for offline replay.
- `tx_resync.py` — TX driver for resync wire format: encode + ship hex via SSH to `irlink tx-symbols`
- `session.py` — App-layer orchestrator: SSH-driven HELLO→META→CAL→TEXT→BYE flow. Optional `--status-poll` for per-cam CGI telemetry.
- `aim_assist.py` — Live-feedback aim tool. `read_grid` is HTTP-first (4 Hz via `/x/brightness-grid.cgi`) with SSH fallback; preflight + LED toggle + ae-freeze + restart-daemon all migrated to HTTP. Phase 6 (kill_cam1_daemon, restart_cam1_daemon spawn) intentionally still SSH.
- `send_message.py` — Legacy CLI

Related: `../protocol/CLAUDE.md` for wire format; `../irlink/CLAUDE.md` for on-camera receiver and the CGI endpoint table; `../webui/` for the live-status webpage that proxies through `cam_status.CamStatusClient`; `../experiments/` for offline decoders and replay.

## Camera HTTP Client (`cam_status.py`)

Centralizes all `http://<cam>/x/*.cgi` access behind one `CamStatusClient` per cam. Cookie file at `/tmp/cam_cookie_<ip>.txt` is reused across calls (login once, replay many). All public methods return `None` on transport failure rather than raising — the contract every caller can rely on:

| Method | Endpoint | Returns | Phase |
|--------|----------|---------|-------|
| `get()` | GET `/x/cal-status.cgi` | irlink status dict (state/mode/rate/peak/counters) | — |
| `get_brightness_grid()` | GET `/x/brightness-grid.cgi` | `(ts_ms, [240 blocks])` | 1 |
| `get_ae_freeze()` | GET `/x/ae-freeze.cgi` | `0` or `1` | 2 |
| `set_ae_freeze(bool)` | POST `/x/ae-freeze.cgi` | `True`/`False` (fail-loud) | 3 |
| `get_proc_status()` | GET `/x/proc-status.cgi` | `{irlink, daynightd, prudynt}` | 4 |
| `imp_cmd(cmd, val)` | POST `/x/json-imp.cgi` | `True`/`False` (fail-loud) | 5 |
| `get_led_state()` | GET `/x/gpio-state.cgi` | `{ir850, ir940}` | webui |

Read paths in `aim_assist` keep an SSH fallback (`read_grid` does HTTP-first, SSH on `None`); write paths fail loud by design — silent SSH fallback for writes risks state divergence. Use `host.cam_status.resolve_ip(host)` to turn an SSH alias (`dacam1`) into a numeric IP for `CamStatusClient(ip)`.

## Host-Side Pixel Receiver (`pixel_rx.py`)

For long-distance (10-20 ft) where grid-based on-camera RX has insufficient signal. Reads RTSP directly, tracks a small pixel ROI (15x15) around the calibrated TX position. DPLL clock recovery with early-late gate. Delta +150-200 at 10-20 feet vs +10-15 with the grid.

```bash
conda activate light
python -m host.cal_procedure --no-interactive       # calibrate first
python -m host.pixel_rx --cam cam2                  # auto-loads calibration, 160ms default
python -m host.pixel_rx --cam cam2 --symbol-ms 120  # faster speed
python -m host.pixel_rx --cam cam2 --tx-pos 385,178 # manual position
```

**Decoder architecture**: two-pass — DPLL on raw sample timestamps first (handles irregular RTSP frame timing), brute-force t0 search on interpolated signal as fallback. The DPLL works directly with irregular sample times to avoid interpolation artifacts from frame gaps.

## TX Driver (`tx_resync.py`)

```bash
python -m host.tx_resync --cam cam1 --speed 80 "The quick brown fox..."
python -m host.tx_resync --cam cam1 --speed 80 --fec "..."   # RS(15,11)
```

Host encodes symbols, ships hex via SSH to `irlink tx-symbols`. Camera is a dumb GPIO toggler.

## Orchestrator (`session.py`)

Laptop-side driver that runs both cameras in `irlink listen/connect` interactive mode over SSH:

```bash
python -m host.session --dry-run                        # exercise pack/unpack/fragment offline
python -m host.session --handshake-only --symbol-ms 160 # SSH + irlink handshake only
python -m host.session --symbol-ms 160 --text "HI"      # full HELLO→META→CAL→TEXT→STATS→BYE
python -m host.session --text "HI"                      # auto symbol_ms from calibration (clamped ≥160ms)
python -m host.session --skip-cal --skip-stats          # subset
```

**Adaptive symbol rate** (2026-04-22): without `--symbol-ms`, session reads `recommended_rate_ms` from `calibration.json` (derived from Δ by `host.config.pick_initial_rate_ms`) and clamps to `≥160ms` for on-camera RX. In-session, irlink itself probes faster after PROBE_UP_AFTER successful ACKed frames, falls back after FALLBACK_AFTER retransmit-exhausted sends, and split-brain-recovers to slowest rung if no valid decode for `2 × ack_timeout_ms`. See `../irlink/CLAUDE.md` → Adaptive Symbol Rate and root `MEMORY.md` → Adaptive symbol rate.

Architecture:
- `IRSession.start()` spawns `ssh -T root@<host> /opt/bin/irlink listen/connect ...` per cam, parses `MSG-HEX:` lines from stdout, writes `send-hex <hex>` to stdin.
- Calibration runs out-of-band beforehand via `host/cal_procedure.py`; `APP_CAL_RESULT` cross-verifies the in-link reading against saved RTSP-derived coords.
- All app logic lives on the laptop; cameras run unmodified `irlink`.
- STATS request prints a parseable line (`STATS: tx=N rx=N crc=N rtx=N dll=N`); read via `request_stats()`.

## Calibration (`cal_procedure.py`)

```bash
python -m host.cal_procedure                    # both directions, interactive
python -m host.cal_procedure --no-interactive   # automated
python -m host.cal_procedure --show             # show saved calibration
```

Frame-differencing (LED-on vs LED-off RTSP frames) finds the exact TX pixel. Saves to `host/calibration.json`. Consumed by `pixel_rx.py` (auto-load) and passed to `irlink --pixel x,y` on-camera.

For the cam1-autonomous deployment path (no PC reading cam1's RTSP), prefer `irlink calibrate-pixel` on-camera or `bicall` over the link — see `../irlink/CLAUDE.md`. cal_procedure stays useful for laptop-driven pixel discovery during dev.

## Aim Assist (`aim_assist.py`)

```bash
python -m host.aim_assist                              # aim → bicall on Enter
python -m host.aim_assist --no-cal                     # aim only
python -m host.aim_assist --hold 180                   # longer LED-on window
python -m host.aim_assist --cam2-pixel 271,91          # cam2's pixel for bicall
```

Workflow for the "operator carrying laptop+cam2 into position" case:

1. Stops cam1's daemon (frees LED control).
2. Measures cam2 baseline (8 grid samples × 250ms).
3. SSH cam1 to hold IR LEDs ON for `--hold` seconds.
4. Polls cam2's `/run/prudynt/brightness_grid` 4× per second via `/x/brightness-grid.cgi` (HTTP-first, with SSH fallback if the CGI is missing); shows live max-block delta vs baseline with `★ GOOD AIM` / `◆ OK` / `· weak` indicator. Walk cam2 around until the marker hits ★.
5. **Enter** → kill LEDs, restart cam1's `daemon-listen` from the saved coords, run `bicall` from cam2 → fresh `/opt/etc/calibration.json` on both sides. **q** → quit without cal.

Currently uses SSH for the LED-hold step (dev mode — cam1 still on WiFi). For production-autonomous cam1, replace the SSH call with an `AIM_REQ` protocol message that triggers cam1's existing handle_cal_request-style LED hold. The script's structure already separates "trigger" from "feedback loop" — only the trigger needs to swap.

Thresholds (line ~30 of `aim_assist.py`): `THRESHOLD_GOOD=80`, `THRESHOLD_OK=40`. Tune for your scene.

## Camera Setup Script (`cam_setup.sh`)

Run before any calibration or communication:
```bash
./host/cam_setup.sh                # both cameras
./host/cam_setup.sh dacam1     # one camera only
./host/cam_setup.sh --check        # verify only, don't fix
```

Checks/fixes: patched prudynt, night/monochrome mode, IR cut filter, daynightd killed, LEDs off, AE freeze idle, brightness grid active, irlink deployed. Uses Thingino web API (curl) for ISP controls, SSH for GPIO and process management.

## Recommended Speeds

- **120ms/sym**: works for host-side `pixel_rx` (RTSP @ 30fps → 3.6 samples/sym).
- **160ms/sym**: required for on-camera RX (BrightnessMonitor @ 18fps → 2.2 samples/sym at 120ms is marginal). Adds ~33% TX time but reliable. `session.py` defaults to 160ms.

## Pitfalls

- **RTSP first frames are black.** First 5-15 frames from a fresh RTSP stream are often completely black (codec init / ISP startup). Always flush 10-15 frames. `cal_procedure.py` uses `grab_frames(cap, n=10)`; `pixel_rx.py` flushes 15 frames on startup.
- **RTSP stream buffers stale frames.** When toggling LEDs and capturing via RTSP, OpenCV's VideoCapture buffer contains frames from before the LED state changed. Open a fresh stream after the state change (rather than flushing frames on an existing stream).
- **OSD timestamp changes create false calibration peaks.** The on-screen timestamp (top-left) changes every second, creating large brightness deltas that can be mistaken for IR LEDs during frame differencing. Mask out the top 30 and bottom 30 pixel rows when searching for the TX peak.
- **Grid-based brightness (20x12 blocks) dilutes IR signal at distance.** At 10-20 ft, the IR LED spot is ~10-20 pixels across but the grid block is 32x30 pixels. The spot's +200 pixel delta gets averaged down to +10-15. Host-side pixel-level RTSP reading (`pixel_rx.py`) gives the full undiluted delta.
- **Dual LED TX (850nm + 940nm) doubles signal.** `irlink` toggles both GPIO 47 and 49 simultaneously. At 10-20 ft: single LED gives +15 grid delta, dual gives +23. At pixel level: +200 for both.
- **Calibration coordinates shift when cameras move.** Pixel calibration is position-sensitive — even a small camera bump invalidates the (x,y) coordinates. Re-run `python -m host.cal_procedure --no-interactive` after any physical change. Stale coordinates point at dark pixels, giving baseline=0 and failed decodes.
- **Camera setup order matters** (`cam_setup.sh`): (1) start prudynt, (2) kill daynightd, (3) set night/monochrome mode, (4) turn off LEDs, (5) set ircut filter LAST (because other commands re-close it).
- **30fps RTSP caps symbol rate at ~120ms minimum.** At 30fps, frames arrive every ~33ms with occasional gaps up to 200ms. At 120ms/symbol, most symbols get 3+ samples. At 100ms, too many symbols fall entirely in frame gaps. DPLL handles jitter but cannot recover symbols with zero samples.
- **DPLL must work on raw sample times, not interpolated grid.** Linear interpolation between sparse RTSP samples (33ms apart) creates fake transitions during frame gaps. The DPLL detects edges in the actual sample timestamps, not in the interpolated 1ms signal. The brute-force t0 search on the interpolated grid is kept as fallback.
- **On-camera frame rate is ~18fps (not 30).** BrightnessMonitor's `IMP_FrameSource_SnapFrame` on channel 1 (640x360) runs at ~18fps. Combined with DPLL, 120ms/sym still works (2+ samples/sym), but 160ms is more reliable. Host-side RTSP gives ~30fps — better oversampling.
- **Frame rate is the receiver bottleneck.** Max practical bps ≈ fps / (2 * samples_per_symbol) → 3-5 bps via RTSP.
- **`session.py` must read SSH pipes as bytes, not text.** irlink's legacy `MSG: <text>` line dumps binary app payload directly; META and other typed messages contain non-UTF-8 bytes. With `text=True`, Python's stdout reader thread crashes with `UnicodeDecodeError` mid-line and the orchestrator silently stops processing messages. Open `subprocess.Popen` with `bufsize=0` (no text mode) and decode each line manually with `errors="replace"`.
- **Orchestrator must serialize back-to-back sends.** `CamLink.send_app(payload, wait=True)` blocks until our irlink prints `PROTO: ACK received for seq=` (success) or `PROTO: send failed` (after retries). Without this, the orchestrator's next `send_app()` queues a new DATA via stdin while our irlink is still mid-TX of the previous one. If the test then issues a *peer* send, the peer starts TXing before this side has finished its ACK round-trip — mutual collision. Pass `wait=False` only for genuine pipelined sends (none in v1).
- **Dump-and-replay beats live debugging for RX changes.** `pixel_rx.py --dump-samples runs/foo.jsonl` writes every TX burst's raw `(t_ms, brightness)` samples to JSONL. `experiments/replay.py runs/foo.jsonl --decoder {baseline,resync,resync-fec}` runs any decoder offline at repo speed. `experiments/debug_capture.py` compares the DPLL-extracted symbol stream against the expected symbols for a known message. Use these before touching decoder code — the root cause is almost never where you'd guess from a live terminal.
- **Camera RTC is not synced** — `ls -la` timestamps on the camera filesystem are unreliable for verifying a deploy. Compare `md5sum` instead.
- **`cal_procedure.py` underreports delta vs `brightness_grid` when AE is active.** RTSP frame-diff between sequential OFF/ON captures lets the camera's auto-exposure drift between captures. When the LED comes on, AE compresses exposure to compensate — the IR LED's apparent brightness in the H.264-encoded frame is much lower than the raw YUV. Direct comparison: at one position cal_procedure reported delta=24 (peak-blurred 9) while a `head /run/prudynt/brightness` snapshot showed max-block jumping 153 → 220 (delta +67). **Workaround:** if cal_procedure delta looks suspiciously weak (<40) but the cameras are physically aimed at each other, switch to `irlink calibrate-pixel` on-camera or run `aim_assist.py` to verify there's actually no signal vs. just AE masking it. **Long-term fix:** cal_procedure should write `1` to `/run/prudynt/ae_freeze` on the RX camera before the OFF capture and `0` after the ON capture (~5 LOC, not yet implemented).
- **Aim asymmetry is geometric, not an electrical issue.** If cam1 sees cam2 at delta 200 but cam2 sees cam1 at delta 20, it's almost certainly aiming angle, not LED power. IR LED radiation patterns are narrow (~30° half-power); a 20-30° off-aim drops received power by 5-10×. Use `aim_assist.py` to find the angle where both directions are strong before running cal — if you can only get one direction strong, the round-trip protocol can't form a reliable link.
