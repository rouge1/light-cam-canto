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

## Project Structure

```
irlink/            — Combined half-duplex transceiver (C, MIPS). See irlink/CLAUDE.md
protocol/          — Manchester/frame/app-layer (pure Python). See protocol/CLAUDE.md
host/              — Host orchestration, RX, calibration, TX driver. See host/CLAUDE.md
experiments/       — Offline decoders, replay.py, debug_capture.py
photos/            — Calibration images, calibration.html viewer
runs/              — JSONL captures from pixel_rx --dump-samples
tests/             — pytest (protocol + app layer + resync framing)
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
ssh-copy-id -i ~/.ssh/cam_key root@192.168.50.110
ssh-copy-id -i ~/.ssh/cam_key root@192.168.50.141
```

Add to `~/.ssh/config`:
```
Host da-camera1
    HostName 192.168.50.110
    User root
    IdentityFile ~/.ssh/cam_key
Host da-camera2
    HostName 192.168.50.141
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
scp -O my_program root@da-camera1:/opt/bin/       # Thingino lacks sftp-server → use -O
ssh da-camera1 "chmod +x /opt/bin/my_program"
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
# Trigger TX from cam1: ssh da-camera1 "/opt/bin/irlink tx HELLO --speed 120"

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
```

### Calibration Viewer
```bash
cd photos && python -m http.server 8888
# Open http://localhost:8888/calibration.html
```

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
- **Calibration coordinates shift when cameras move** — even a small bump invalidates pixel coordinates. Re-run `host.cal_procedure --no-interactive` after any physical change.

## Key References

- Thingino firmware: https://github.com/themactep/thingino-firmware
- Prudynt-T streamer: https://github.com/gtxaspec/prudynt-t
- ingenic-pwm: https://github.com/gtxaspec/ingenic-pwm
- Rolling shutter OCC research: https://pmc.ncbi.nlm.nih.gov/articles/PMC7061997/
