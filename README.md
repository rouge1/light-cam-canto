# Light-Cam-Canto (LiWiFi)

IR light communication between Wyze V3 cameras using their built-in infrared illuminators.

## What is this?

An experimental system that turns Wyze V3 security cameras into IR Li-Fi transceivers. One camera transmits data by modulating its 940nm IR LEDs (via hardware PWM); another camera receives by analyzing brightness changes in its video feed.

**Research questions**: What data rate and range can we achieve with off-the-shelf camera hardware?

## How it works

```
         USB              IR (full-duplex)        USB
    [PC] <==> [Cam A] <==================> [Cam B] <==> [PC]
    port1    tx/rx     5-1000+ bps          tx/rx    port2
```

- **PC <-> Camera**: USB Direct (RNDIS) — power, video, SSH over one cable. No WiFi.
- **Camera <-> Camera**: IR light — 940nm LEDs modulated via hardware PWM, received via frame brightness analysis.
- **Full-duplex**: Both cameras can transmit and receive IR simultaneously (LEDs point outward, away from own sensor).

1. **Transmitter** modulates 940nm IR LEDs using PWM on `/dev/pwm` (GPIO 49 → PWM channel 0)
2. **Receiver** captures RTSP video stream and detects brightness transitions
3. **Protocol layer** handles sync, framing, and error detection
4. **Host PC** orchestrates tests and measures Bit Error Rate (BER)

## Expected Performance

| Method | Speed | Range | Status |
|--------|-------|-------|--------|
| Phase 0 (infrastructure) | N/A | — | In progress |
| Phase 1 (shell toggle) | 5-10 bps | ~9m | Planned |
| Phase 2 (PWM) | 100-1000 bps | ~9m | Planned |
| Phase 2+ (rolling shutter) | 1-10 kbps | ~5m | Research |

## Requirements

### Hardware
- 2x Wyze Cam V3 with [Thingino](https://github.com/themactep/thingino-firmware) firmware
- Host PC with 2 USB ports
- USB cables for camera connection

> **Note**: Only Wyze Cam V3 is supported. Wyze Cam OG, V4, V2, and Pan V3 are **not supported**.

### Software
```bash
pip install opencv-python numpy paramiko matplotlib pytest
```

## Quick Start

```bash
# 1. Flash Thingino to both cameras via SD card
#    (see https://github.com/themactep/thingino-firmware)

# 2. Connect both cameras via USB (RNDIS mode)

# 3. Verify SSH over USB
ssh root@<cam_a_ip>
ssh root@<cam_b_ip>

# 4. Verify RTSP stream over USB
ffplay rtsp://thingino:thingino@<cam_a_ip>/ch0

# 5. Run tests
pytest tests/ -v
```

## Project Structure

```
transmitter/   — IR LED control (shell scripts, C PWM driver)
receiver/      — Frame capture and brightness analysis
protocol/      — OOK encoding, Manchester coding, framing, CRC
host/          — SSH-based test orchestration
benchmarks/    — Range/speed sweep tests and result plotting
tests/         — Unit tests
```

## License

MIT
