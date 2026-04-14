# Light-Cam-Canto (LiWiFi)

IR light communication between Wyze V3 cameras using their built-in infrared illuminators.

## What is this?

An experimental system that turns Wyze V3 security cameras into IR Li-Fi transceivers. One camera transmits data by modulating its 940nm IR LEDs; another camera receives by analyzing brightness changes in its video feed. Uses Manchester encoding with CRC-8 error detection for reliable data transfer.

## How it works

```
    [PC] ---- WiFi ---- [Cam A] <====IR====> [Cam B] ---- WiFi ---- [PC]
                         tx/rx    ~3 bps      tx/rx
```

1. **Transmitter**: C binary on camera toggles 940nm IR LEDs via sysfs GPIO with precise timing
2. **Receiver**: On-camera C binary reads brightness from patched prudynt-t, decodes Manchester symbols using ROI tracking with AE residual cancellation
3. **Protocol**: Manchester encoding (self-clocking) + CRC-8 checksums + frame sync
4. **Host PC**: Orchestrates TX/RX, can also run host-side RTSP decoder

## Current Status

| Phase | Speed | Status |
|-------|-------|--------|
| Phase 0 (SSH, RTSP, GPIO) | N/A | Done |
| Phase 1 (shell-based OOK) | ~1 bps | Done |
| Phase 2 (C GPIO transmitter) | ~3 bps | Done |
| On-camera brightness monitor | N/A | Working (prudynt-t patch) |
| On-camera receiver (irlink_rx) | ~3 bps | Working (ROI + AE residual) |
| Duplex protocol (ACK/retransmit) | TBD | In progress |

## Requirements

### Hardware
- 2x Wyze Cam V3 with [Thingino](https://github.com/themactep/thingino-firmware) firmware
- Host PC (Linux)
- WiFi network (cameras and PC on same subnet)

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

# 3. Open IR cut filters (required for IR reception)
ssh da-camera1 "ircut off; killall daynightd"
ssh da-camera2 "ircut off; killall daynightd"

# 4. Send a message via IR light
conda activate light
python -m host.send_message "HELLO"

# 5. Run tests
pytest tests/ -v
```

## Project Structure

```
protocol/      — Manchester encoding, CRC-8, frame encode/decode
transmitter/   — TX: shell scripts (Phase 1), C GPIO binary (Phase 2)
receiver/      — RX: on-camera decoder (irlink_rx.c) + host-side RTSP decoder
host/          — SSH orchestration, config, end-to-end CLI
tests/         — 26 pytest tests (protocol layer)
```

See [CLAUDE.md](CLAUDE.md) for detailed hardware docs, GPIO pinouts, firmware build instructions, and common pitfalls.

## License

MIT
