# Light-Cam-Canto (LiWiFi)

IR light communication between Wyze V3 cameras using their built-in infrared illuminators.

## What is this?

An experimental system that turns Wyze V3 security cameras into IR Li-Fi transceivers. One camera transmits data by modulating its 940nm IR LEDs; another camera receives by analyzing brightness changes in its video feed. Uses Manchester encoding with CRC-8 error detection, a TCP-like half-duplex protocol (SYN/ACK handshake, retransmit), and ISP auto-exposure freeze for reliable data transfer.

## How it works

```
    [PC] ---- WiFi ---- [Cam A] <====IR====> [Cam B] ---- WiFi ---- [PC]
                         tx/rx    ~3 bps      tx/rx
```

1. **Transceiver**: On-camera C binary (`irlink`) handles both TX (GPIO LED toggle) and RX (brightness grid from patched prudynt-t)
2. **Protocol**: TCP-like half-duplex — SYN/SYN_ACK/ACK handshake, DATA with ACK/retransmit, PING/PONG, over-the-link calibration
3. **Encoding**: Manchester (self-clocking) + CRC-8 checksums + frame sync
4. **AE Freeze**: Locks ISP auto-exposure during communication for stable raw pixel detection
5. **Host PC**: Orchestrates TX/RX, can also run host-side RTSP decoder

## Current Status

| Phase | Speed | Status |
|-------|-------|--------|
| Phase 0 (SSH, RTSP, GPIO) | N/A | Done |
| Phase 1 (shell-based OOK) | ~1 bps | Done |
| Phase 2 (C GPIO transmitter) | ~3 bps | Done |
| On-camera brightness monitor | N/A | Working (prudynt-t patch) |
| On-camera receiver (irlink_rx) | ~3 bps | Working (ROI + AE residual) |
| Half-duplex protocol (irlink) | ~3 bps | Working (SYN/ACK + AE freeze) |
| Over-the-link calibration | N/A | Implemented (CAL_REQ/ACK/DONE) |
| PING/PONG RTT measurement | N/A | Implemented |

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

# 4. Calibrate (toggle other camera's LED during 5s window)
ssh da-camera2 "irlink calibrate"    # outputs block index, e.g. 45

# 5. Send a message via IR light (on-camera protocol)
ssh da-camera2 "irlink daemon-listen --block 45 &"
ssh da-camera1 "irlink send 'HELLO' --block 45"

# 6. Run tests
pytest tests/ -v
```

## Project Structure

```
irlink/        — Combined half-duplex transceiver with protocol layer (C, MIPS)
protocol/      — Manchester encoding, CRC-8, frame encode/decode
transmitter/   — TX: shell scripts (Phase 1), C GPIO binary (Phase 2)
receiver/      — RX: standalone on-camera decoder + host-side RTSP decoder
host/          — SSH orchestration, config, end-to-end CLI
tests/         — 26 pytest tests (protocol layer)
```

See [CLAUDE.md](CLAUDE.md) for detailed hardware docs, GPIO pinouts, firmware build instructions, and common pitfalls.

## License

MIT
