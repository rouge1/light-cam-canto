# Light-Cam-Canto (LiWiFi) — Development Guide

## Project Overview

IR light communication system between Wyze V3 cameras using their built-in IR illuminators. Cameras transmit data by modulating IR LEDs and receive by analyzing video frames for brightness changes. Custom firmware on top of Thingino enables on-camera TX/RX for a half-duplex reliable link with TCP-like protocol (SYN/ACK handshake, retransmit).

## Subsystem Guides (nested CLAUDE.md)

- `irlink/CLAUDE.md` — On-camera transceiver (C, MIPS): protocol, AE freeze, TX/RX threading, carrier-aware ACK
- `protocol/CLAUDE.md` — Manchester/frame/resync/FEC wire format + app-layer message types + DPLL pitfalls
- `host/CLAUDE.md` — `pixel_rx.py`, `tx_resync.py`, `session.py`, calibration, `cam_setup.sh`
- `thingino-firmware/CLAUDE.md` — Firmware build, patched prudynt-t, USB NCM networking, ISP/LED quirks

Nested CLAUDE.md files load only when Claude touches files in that directory. Root stays short.

## Tech Stack

- **Python 3.10+** — receiver, protocol, benchmarks, test orchestration, calibration
- **C (cross-compiled for MIPS)** — on-camera GPIO transmitter, brightness monitor
- **OpenCV / NumPy** — host-side frame capture, RTSP, signal processing
- **requests** — Thingino web API control (LED toggle, ISP settings)
- **pytest** — testing
- **Conda env: `light`** — `conda activate light`

## Target Hardware

Wyze Cam V3 only. Both test cameras are the `wyze_cam3_t31x_gc2053_atbm6031` variant (Ingenic T31X SoC, ATBM6031 WiFi, 4x 850nm + 4x 940nm IR LEDs, 15-20 fps). Thingino custom firmware.

## GPIO Map (Wyze V3 / T31)

```
GPIO 47  — 850nm IR LEDs (GPIO only, no PWM, faint visible red glow)
GPIO 49  — 940nm IR LEDs (PWM channel 0, invisible) ← primary TX path
GPIO 52  — IR cut filter (open)
GPIO 53  — IR cut filter (close)
GPIO 38  — Red status LED (active low)
GPIO 39  — Blue status LED (active low)
```

The IR cut filter physically blocks infrared. Must be opened before any IR reception:
```bash
ircut off           # Open filter (night mode) — IR passes through
killall daynightd   # Prevent auto-switching back
```

## Protocol Stack Summary

**Manchester encoding** (IEEE 802.3): `0` → `[1,0]`, `1` → `[0,1]`. Self-clocking.

**Classic frame**: `[PREAMBLE 8][SYNC 8][LENGTH 8][PAYLOAD N×8][CRC-8][POSTAMBLE 4]` → Manchester encoded. Sync word `11001011`, CRC-8/CCITT poly 0x07.

**Resync framing** (opt-in, long messages): injects 16 raw `1010...` symbols every 48 data symbols to keep DPLL locked. See `protocol/CLAUDE.md`.

**RS FEC** (opt-in): `encode_frame_symbols_with_resync_fec`, default RS(15,11), ~36% overhead. See `protocol/CLAUDE.md`.

**Adaptive symbol rate** (on-camera, since 2026-04-22): irlink self-tunes rate within a session via `MSG_RATE_CHANGE=0x0A`. Ladder `[60, 80, 100, 120, 160, 200]` ms. Start rate from calibration Δ → `pick_initial_rate_ms()` → `host/session.py` clamps to ≥160ms for on-camera RX. In-session: probe-up after 5 ACKs, fallback-down after 3 retransmit-exhausted sends, split-brain recovery to slowest rung on `2×ack_timeout_ms` without valid decode. See `irlink/CLAUDE.md` → Adaptive Symbol Rate.

### Speeds Achieved

| Phase | Method | Speed | Status |
|-------|--------|-------|--------|
| Phase 1 | Shell `gpio set` + `usleep` | ~1 bps | Done |
| Phase 2 | C binary, sysfs GPIO | ~3 bps | Done |
| On-camera RX (grid) | irlink + BrightnessMonitor grid | ~3 bps | Working |
| On-camera RX (pixel) | irlink --pixel + DPLL | ~4.2 bps | Working (120ms/sym, delta 120+) |
| Half-duplex link | irlink (TX+RX+protocol) | ~3 bps | Working (AE freeze + raw block) |
| Host pixel RX | pixel_rx.py (RTSP + pixel ROI + DPLL) | ~4.2 bps | Working (120ms/sym, delta 150+) |
| Resync-framed RX (host) | pixel_rx --decoder resync | **4.91 bps @ 70ms/sym** | Single-frame ≤50-char, beats drift |
| FEC-wrapped RX (host) | pixel_rx --decoder resync-fec | ~3.3 bps @ 80ms | RS(15,11), erasure-aware, decodes 82-char at Δ≈95 |
| **Adaptive rate (on-camera)** | irlink probe-up / fallback / split-brain | 2–5 bps, self-tuning | **Working** — starts at cal-picked rate (≥160ms), ramps up/down with SNR |
| **Autonomous cam1 + over-link cal** | rc.local autostart + `bicall` | full bidi cal in ~3.5 min | **Working** — cam1 boots, autostarts `daemon-listen` with saved coords; cam2 connects over light, runs `bicall`, both `/opt/etc/calibration.json` files refresh |
| **Monocal (one-way over-light cal, autonomous-cam1)** | `irlink monocal` + cam2-only CGIs + webui MONOCAL CAM1 button | ~50s wall, single-cam refresh | **Working** — cam2 holds LEDs, cam1 scans, only cam1's `/opt/etc/calibration.json` updates. CGI-driven (zero SSH from laptop). For the [pc]→[cam2]→light→[cam1] deployment topology — see `irlink/CLAUDE.md` Monocal section. |

## Project Structure

```
irlink/            — Combined half-duplex transceiver (C, MIPS). See irlink/CLAUDE.md
  cgi/             — Shell CGIs deployed to /var/www/x/ on each cam: brightness-grid,
                     ae-freeze, proc-status, gpio-state, cal-status. See irlink/CLAUDE.md
protocol/          — Manchester/frame/app-layer (pure Python). See protocol/CLAUDE.md
host/              — Host orchestration, RX, calibration, TX driver. See host/CLAUDE.md
  cam_status.py    — Single CamStatusClient for all camera HTTP endpoints (cookie auth + retry)
webui/             — 8-bit live-status webpage + http.server proxy (live cal sequence,
                     real-time LED state, embedded calibration viewer). See "Live-Status
                     Webpage" below.
experiments/       — Offline decoders, replay.py, debug_capture.py
photos/            — Calibration images, calibration.html viewer
runs/              — JSONL captures from pixel_rx --dump-samples
tests/             — pytest (protocol + app layer + resync framing + cam_status HTTP)
transmitter/       — Legacy Phase 2 C TX binary
receiver/          — Legacy standalone on-camera decoder + rx_stream.py
thingino-firmware/ — Firmware build tree (not committed). See thingino-firmware/CLAUDE.md
```

## Setup

### Host Machine
```bash
conda create -n light python=3.12 -y
conda activate light
pip install opencv-python numpy paramiko matplotlib pytest requests
```

### Camera SSH
```bash
ssh-keygen -t ed25519 -f ~/.ssh/cam_key
ssh-copy-id -i ~/.ssh/cam_key root@192.168.50.113
ssh-copy-id -i ~/.ssh/cam_key root@192.168.50.143
```

Add to `~/.ssh/config`:
```
Host dacam1
    HostName 192.168.50.113
    User root
    IdentityFile ~/.ssh/cam_key
Host dacam2
    HostName 192.168.50.143
    User root
    IdentityFile ~/.ssh/cam_key
Host usb-cam1
    HostName 172.16.1.1
    User root
    IdentityFile ~/.ssh/cam_key
Host usb-cam2
    HostName 172.16.2.1
    User root
    IdentityFile ~/.ssh/cam_key
```

### Code Deployment
```bash
scp -O my_program root@dacam1:/opt/bin/       # Thingino lacks sftp-server → use -O
ssh dacam1 "chmod +x /opt/bin/my_program"
```

### Cross-Compilation (C for MIPS)
```bash
cd irlink && make           # or transmitter/ or receiver/
make deploy
```

Toolchain: `thingino-firmware/output/stable/wyze_cam3_t31x_gc2053_atbm6031-3.10.14-musl/host/bin/mipsel-linux-gcc`. Building firmware itself and patched prudynt: see `thingino-firmware/CLAUDE.md`.

## Usage

```bash
# 1. Setup cameras (after every reboot)
./host/cam_setup.sh

# 2. Calibrate (find TX pixel locations)
conda activate light
python -m host.cal_procedure --no-interactive
python -m host.cal_procedure --show             # verify calibration

# 3a. On-camera pixel RX (recommended, ~4.2 bps)
# On cam2: irlink listen --pixel 385,178 --speed 120
# On cam1: irlink tx "HELLO" --speed 120
# With protocol: irlink send "HELLO" --pixel <coords> --speed 120

# 3b. Host-side pixel RX (~4.2 bps, best SNR)
python -m host.pixel_rx --cam cam2 --symbol-ms 120
# Trigger TX from cam1: ssh dacam1 "/opt/bin/irlink tx HELLO --speed 120"

# 3c. Resync-framed long messages (~4.9 bps @ 70ms/sym)
python -m host.pixel_rx --cam cam2 --symbol-ms 80 --decoder resync \
    --dump-samples runs/capture.jsonl &
python -m host.tx_resync --cam cam1 --speed 80 \
    "The quick brown fox jumps over the lazy dog"

# 3d. Offline replay of a dumped capture
python -m experiments.replay runs/capture.jsonl --decoder baseline
python -m experiments.replay runs/capture.jsonl --decoder resync -v

# 4. Multi-app orchestration (HELLO→META→CAL→TEXT→BYE)
python -m host.session --symbol-ms 160 --text "HI"

# 5. Aim cam2 at cam1 with live feedback, then trigger bicall over light
python -m host.aim_assist                      # ★ GOOD AIM indicator + Enter to trigger bicall
python -m host.aim_assist --no-cal             # aim only

# 6. On-camera pixel cal (no laptop RTSP needed — cam1-autonomous path)
ssh dacam2 "light ir850 on; light ir940 on; sleep 12; light ir850 off; light ir940 off" &
sleep 2
ssh dacam1 "/opt/bin/irlink calibrate-pixel"   # writes cam1's /opt/etc/calibration.json

# 7. Bidirectional cal entirely over light (no SSH to cam1 once it's autonomous)
echo bicall | ssh dacam2 "/opt/bin/irlink connect --pixel <cam2sees> --speed 160"

# 8. Monocal — one-way cal over light, CGI-driven (zero SSH from laptop).
#    Use this once cam1 is deployed: only cam1's /opt/etc/calibration.json
#    refreshes (cam2's is laptop-cal'd via cal_procedure). ~50s wall.
curl -X POST -d '' http://127.0.0.1:8765/api/cal/monocal           # webui flow
# or via webui: open http://127.0.0.1:8765/ → click MONOCAL CAM1
# or via cam2 CGI directly:
curl -X POST -d 'coords=243,177' http://dacam2/x/monocal-trigger.cgi
```

### Calibration Viewer
```bash
cd photos && python -m http.server 8888
# Open http://localhost:8888/calibration.html
```

### Live-Status Webpage (`webui/`)

8-bit themed live dashboard showing the 5 SSH→web-migrated CGIs in action, with two big action buttons:
- **▶ RUN CAL SEQUENCE** (green) — drives `cal_procedure --no-interactive` end-to-end through phases 1-5, laptop-driven, both cams reachable. ~90s.
- **▶ MONOCAL CAM1** (cyan) — drives `irlink monocal` over light via cam2's `/x/monocal-trigger.cgi`. cam2 holds LEDs, cam1 scans, only cam1's `/opt/etc/calibration.json` refreshes. M1..M5 step stack tracks the state machine via `/api/cal/monocal-status` (which merges cam2's `monocal-status.cgi` with cam1's `cal-status.cgi`). ~50s. For the autonomous-cam1 deployment topology where laptop only reaches cam2.

Two pixel-art Wyze V3 cameras at the top reflect real GPIO state (green-pulsing lens when idle, red-flickering 8-LED ring when IR is firing).

```bash
# Terminal 1 — calibration viewer (iframe target)
cd photos && python -m http.server 8889

# Terminal 2 — main live-status server (8765 default)
conda activate light
python -m webui.server                 # default 127.0.0.1:8765
python -m webui.server --port 9000     # different port
python -m webui.server --bind 0.0.0.0  # expose on LAN

# open http://127.0.0.1:8765/
```

The server proxies `/api/{cam}/grid|ae|proc|leds|status` and `POST /api/{cam}/ae|led` through a per-host `CamStatusClient`, plus `/api/cal/start` + `/api/cal/status` for the cal worker. See `webui/server.py` for the routing table.

**Stack push behavior:** as the sequence runs, each new step prepends to the top with a slide-in animation; previous steps stack below in completion order. Step 5 expands to show the live `cal_procedure` stdout.

**Post-cal behavior (since 2026-05-08):** on `DONE · OK` the page auto-switches to the CAL RESULTS tab and force-reloads the iframe via `about:blank` → `?_t=<now>` (cache-bust isn't enough on its own — `calibration.html` is rewritten with new image filenames each run, but the browser tends to hold the old DOM until you fully tear down the iframe). Manual **▸ RELOAD CAL VIEWER** button uses the same `refreshCalViewer()` helper.

**Beam channel is direction-aware:** the top beam (`CAM1 ▶▶▶ CAM2`) only animates when cam1's IR LEDs are physically firing; the bottom (`CAM1 ◀◀◀ CAM2`) only when cam2's are. Idle = neutral-gray dotted lines, no traveling photon — the UI signals are tied to real GPIO state via `/api/{cam}/leds`, not decorative. Both labels pulse the same cyan→white when their direction is live.

## Running Tests

```bash
conda activate light
pytest tests/ -v
```

## Cross-Cutting Gotchas

The few that bite regardless of which subsystem you're in. Subsystem-specific pitfalls live in the nested CLAUDE.md files.

- **IR cut filter must be open** (`ircut off`). Without this, IR is physically blocked — no amount of software can detect it. After every reboot, the filter resets to day mode. Kill `daynightd` to prevent auto-switching. The filter ALSO resets when prudynt restarts.
- **SCP needs `-O` flag** (legacy protocol). Thingino lacks `sftp-server`.
- **940nm is invisible.** Use a phone camera or the other camera's RTSP stream to verify TX. 850nm has a faint visible red glow for sanity checks.
- **Overlay filesystem is tiny** (~224KB `/etc` + 8MB `/opt`). Never `cp` large files to `/usr/bin/` — it will fill the overlay and corrupt the binary. Deploy to `/opt/bin/`.
- **Wyze V3 USB port carries power AND data** — every USB cable change is a hard camera reboot. Wait ~60s for WiFi to come back before SSHing.
- **AE freeze is required for half-duplex protocol.** Without it, the camera's own LED reflections cause massive AE swings (3-5s settle). `irlink` handles this via `/run/prudynt/ae_freeze`. See `irlink/CLAUDE.md`.
- **Calibration coordinates shift when cameras move** — even a small bump invalidates pixel coordinates. Re-run `host.cal_procedure --no-interactive` after any physical change, or `aim_assist.py` + `bicall` once cam1 is deployed.
- **DHCP IP rotation on reboot.** Both cameras get DHCP leases that may shift on reboot. `cam1` rotated `.110 → .113` mid-session and broke every hardcoded IP at once. When it happens, update in: `/etc/hosts`, `~/.ssh/config`, `host/cam_setup.sh:cam_ip()`, `host/cal_procedure.py:CAMERAS`, `host/config.py:TX_IP/RX_IP`, root `CLAUDE.md`, `thingino-firmware/CLAUDE.md`. Long-term fix is DHCP reservation on the router or use static USB-NCM (`usb-cam1`/`usb-cam2`).
- **`killall -9 irlink` leaves AE freeze stuck at 1.** SIGKILL skips irlink's cleanup that resets `/run/prudynt/ae_freeze` to 0. Cameras then refuse to AE-adapt to LED on/off — `cal_procedure.py` quietly returns near-zero deltas. `cam_setup.sh` detects this; manual fix is `ssh <cam> "echo 0 > /run/prudynt/ae_freeze"`. Prefer plain `kill` so SIGTERM fires.
- **`irlink daemon-listen` ignores SIGTERM.** `pkill -TERM irlink` and `killall irlink` cleanly stop irlink in `connect`/`listen` modes (rx thread halts + AE freeze resets), but in `daemon-listen` mode the main process lingers indefinitely after the rx thread stops. Use `kill -9 <pid>` + manual `echo 0 > /run/prudynt/ae_freeze` to fully tear down a daemon-listen instance. `aim_assist.kill_cam1_daemon()` already does this. Pre-existing irlink behavior — surfaced when Phase 4's HTTP `pgrep` made it observable that `kill_cam1_daemon` + `restart_cam1_daemon` could leave 2 instances briefly.
- **CGI files on Thingino need `chmod 755`, not `chmod +x`.** When deploying to `/var/www/x/`, scp transfers the file under root's umask 077 → mode `600`; `chmod +x` then yields `700`, which uhttpd serves as `403 Forbidden`. Always `chmod 755 /var/www/x/<file>.cgi` after scp. A 403 from a freshly-deployed CGI is almost always this.
- **busybox `pgrep -c` is not supported.** Thingino's busybox 1.37 pgrep doesn't accept `-c`; the call exits non-zero with a usage error, so `pgrep -c <name> || echo 0` always returns "0" silently. The pre-Phase-4 `aim_assist.preflight()` had this latent bug for irlink/daynightd counts. Use `pgrep <name> | wc -l` instead. For binaries with hyphens in the name (e.g. `prudynt-patched`), prefer `pidof` since pgrep matches argv[0] basename only.
- **`/x/json-imp.cgi` returns `{"code":200,"result":"success",...}`, not `{"success":true}`.** The legacy convention in `cam_setup.sh` and `cal_procedure.py` is to grep for the bare substring `"success"` rather than the structured field. `CamStatusClient.imp_cmd()` follows the same convention. Don't assume a JSON `success: true` field — inspect the actual body shape before parsing strictly.
- **`/x/json-heartbeat-slow.cgi` reports cached daynightd state, NOT actual GPIO.** Its `ir850_state` / `ir940_state` fields track an internal model that isn't synced when `imp_cmd ir850/ir940` writes happen directly. We added `irlink/cgi/gpio-state.cgi` (reads `gpio read 47`/`gpio read 49`) for the live UI's real-time LED reflection. Use that when the UI needs to know whether IR LEDs are actually firing right now.
- **busybox `date +%s%3N` doesn't expand `%N`.** Thingino busybox 1.37 emits seconds only — the `%3N` literally appears in output or gets dropped depending on path. The original `monocal-trigger.cgi` JSON pre-write produced bogus `started_ms:1776061326` (seconds). Use `$(date +%s)000` for ms-granularity timestamps; the consumer (irlink C binary) overwrites with real `clock_gettime` ms within hundreds of ms.
- **Monocal needs new `irlink` on BOTH cams.** The new `peer_scans` flag is bit 1 of CAL_REQ payload[0]. Old responders that check `data[0] == 1` interpret `0x02` as bidi=0 → fall through to regular `cal` (both sides hold LEDs, neither scans, then 90s CAL_DONE timeout). Always `make deploy` before flipping to a monocal-driven workflow. See irlink/CLAUDE.md → Monocal section.

## Key References

- Thingino firmware: https://github.com/themactep/thingino-firmware
- Prudynt-T streamer: https://github.com/gtxaspec/prudynt-t
- ingenic-pwm: https://github.com/gtxaspec/ingenic-pwm
- Rolling shutter OCC research: https://pmc.ncbi.nlm.nih.gov/articles/PMC7061997/
