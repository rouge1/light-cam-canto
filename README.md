# Light-Cam-Canto (LiWiFi)

IR light communication between Wyze V3 cameras using their built-in infrared illuminators.

## What is this?

An experimental system that turns Wyze V3 security cameras into IR Li-Fi transceivers. One camera transmits data by modulating its 850nm + 940nm IR LEDs; another camera receives by analyzing brightness changes in its video feed. Uses Manchester encoding with CRC-8 error detection, a TCP-like half-duplex protocol (SYN/ACK handshake, retransmit), and ISP auto-exposure freeze for reliable data transfer.

## How it works

```
    [PC] ---- WiFi ---- [Cam A] <====IR====> [Cam B] ---- WiFi ---- [PC]
                         tx/rx   ~4.2 bps     tx/rx
```

1. **Calibration**: Host-side frame differencing locates the exact pixel position of the peer camera's IR LEDs
2. **Transceiver**: On-camera C binary (`irlink`) handles both TX (dual GPIO LED toggle) and RX (pixel-level ROI from patched prudynt-t)
3. **DPLL clock recovery**: Digital phase-locked loop tracks symbol timing from edge transitions, handling irregular frame intervals and 100-200ms gaps
4. **Protocol**: TCP-like half-duplex — SYN/SYN_ACK/ACK handshake, DATA with ACK/retransmit, PING/PONG, over-the-link calibration
5. **Encoding**: Manchester (self-clocking) + CRC-8 checksums + frame sync
6. **AE Freeze**: Locks ISP auto-exposure during communication for stable raw pixel detection
7. **Host PC**: Orchestrates calibration, can also run host-side RTSP pixel receiver

## Current Status

| Phase | Speed | Status |
|-------|-------|--------|
| Phase 1 (shell-based OOK) | ~1 bps | Done |
| Phase 2 (C GPIO transmitter) | ~3 bps | Done |
| On-camera grid RX (legacy) | ~3 bps | Working (20x12 block grid + AE freeze) |
| On-camera pixel RX + DPLL | ~4.2 bps | **Working** (pixel ROI + DPLL, 120ms/symbol) |
| Host-side pixel RX + DPLL | ~4.2 bps | **Working** (RTSP + calibrated pixel ROI) |
| Half-duplex protocol | ~3 bps | Working (SYN/ACK + AE freeze) |
| Host-side pixel calibration | N/A | Working (frame diff, saves to calibration.json) |
| Raw TX mode | ~4.2 bps | Working (`irlink tx` — no handshake, for pixel_rx) |
| Application layer | — | Working (`protocol/app.py` — typed messages on top of DATA, fragmentation for >16B) |
| Session orchestrator | — | Working (`host/session.py` — handshake + bilateral HELLO/META/META_ACK; carrier-aware ACK + send-complete serialization handle the back-to-back multi-message case) |

## Requirements

### Hardware
- 2x Wyze Cam V3 with [Thingino](https://github.com/themactep/thingino-firmware) firmware
- Host PC (Linux)
- Connection to cameras: either (a) WiFi on a shared subnet, or (b) USB CDC-NCM direct to the PC (cam1 at `172.16.1.1`, cam2 at `172.16.2.1`) — see [CLAUDE.md → Networking](CLAUDE.md#networking-usb--wifi)

### Software
```bash
conda create -n light python=3.12 -y
conda activate light
pip install opencv-python numpy paramiko matplotlib pytest
```

## Quick Start

```bash
# 1. Flash Thingino to both cameras via SD card

# 2. Set up SSH keys
ssh-copy-id root@192.168.50.110   # da-camera1
ssh-copy-id root@192.168.50.141   # da-camera2

# 3. Setup cameras (night mode, IR cut filter, patched prudynt)
./host/cam_setup.sh

# 4. Calibrate — find exact pixel location of peer's IR LEDs
conda activate light
python -m host.cal_procedure --no-interactive

# 5a. On-camera pixel RX (use coords from calibration)
ssh da-camera2 "irlink listen --pixel 385,178 --speed 120"  # RX
ssh da-camera1 "irlink tx 'HELLO' --speed 120"              # TX

# 5b. Host-side pixel RX (auto-loads calibration)
python -m host.pixel_rx --cam cam2 --symbol-ms 120          # RX
ssh da-camera1 "/opt/bin/irlink tx 'HELLO' --speed 120"     # TX

# 5c. App-layer session orchestrator (laptop drives both cams over SSH)
python -m host.session --dry-run                            # offline pack/unpack check
python -m host.session --handshake-only --symbol-ms 160     # SSH + handshake
python -m host.session --symbol-ms 160 --text "HI"          # full HELLO→META→CAL→TEXT→BYE

# 6. Run tests
pytest tests/ -v   # 50 tests: protocol + app layer
```

## Project Structure

```
irlink/        — Combined half-duplex transceiver: TX+RX, DPLL decode, pixel ROI (C, MIPS)
                 interactive cmds: send / send-hex / ping / cal / stats / quit
protocol/      — Manchester encoding, CRC-8, frame encode/decode (Python)
  app.py            — Typed app-layer messages on top of DATA: HELLO/META/CAL_RESULT/STATS/TEXT/BYE/CHUNK/NACK + fragment/reassemble
host/          — Calibration, pixel RX, SSH orchestration, camera setup
  cal_procedure.py  — Pixel-level calibration via RTSP frame differencing
  pixel_rx.py       — Host-side RTSP pixel receiver with DPLL clock recovery
  cam_setup.sh      — Camera pre-flight: night mode, ircut, LEDs, prudynt
  session.py        — App-layer orchestrator: SSH-driven HELLO→META→CAL→TEXT→BYE flow
transmitter/   — TX: shell scripts (Phase 1), C GPIO binary (Phase 2)
receiver/      — RX: standalone on-camera decoder + legacy host-side RTSP decoder
photos/        — Calibration images, calibration.html viewer
tests/         — 50 pytest tests (protocol + app layers)
```

See [CLAUDE.md](CLAUDE.md) for detailed hardware docs, GPIO pinouts, firmware build instructions, and common pitfalls.

## License

MIT
