# Light-Cam-Canto (LiWiFi) — Development Guide

## Project Overview

IR light communication system between Wyze V3 cameras using their built-in IR illuminators. Cameras transmit data by modulating IR LEDs and receive by analyzing video frames for brightness changes.

## Tech Stack

- **Python 3.10+** — receiver, protocol, benchmarks, test orchestration
- **C (cross-compiled)** — low-level GPIO/PWM transmitter for Ingenic T31
- **OpenCV** — frame capture and brightness analysis
- **Paramiko** — SSH orchestration to cameras
- **NumPy** — signal processing
- **Matplotlib** — benchmark plotting
- **pytest** — testing

## Firmware: Thingino

This project uses **Thingino** (https://github.com/themactep/thingino-firmware), a full replacement firmware for Ingenic-based cameras. It replaces the stock Wyze firmware entirely — no Wyze code runs.

### Why Thingino (not wz_mini_hacks)

- **Hardware PWM** on 940nm IR LEDs via `/dev/pwm` ioctls (nanosecond precision)
- Full kernel/system control — custom modules, ISP tuning, no cloud
- Reliable USB RNDIS networking with built-in DHCP
- Prudynt-T RTSP streamer (dual streams, ONVIF, audio, motion)
- Writable JFFS2 overlay — SCP binaries that persist across reboots
- `daynightd` fully controllable or disableable (no auto IR switching surprises)

## Target Hardware

### Supported: Wyze Cam V3 only

| Camera | SoC | IR LEDs | FPS | Firmware |
|--------|-----|---------|-----|----------|
| Wyze V3 | Ingenic T31X (MIPS) | 4x 850nm + 4x 940nm | 15-20 | Thingino |

Three V3 hardware variants exist (different WiFi chips):

| Thingino config | WiFi chip | SoC |
|---|---|---|
| `wyze_cam3_t31x_gc2053_atbm6031` | ATBM6031 | T31X |
| `wyze_cam3_t31x_gc2053_rtl8189ftv` | RTL8189FTV | T31X |
| `wyze_cam3_t31al_gc2053_atbm6031` | ATBM6031 | T31AL |

### Unsupported Cameras

| Camera | SoC | Why |
|--------|-----|-----|
| Wyze Cam OG | Fulhan (not Ingenic) | Different SoC, no custom firmware support |
| Wyze V4 | Ingenic T41 (secure boot) | Requires physical chip swap to bypass secure boot |
| Wyze Cam Pan V3 | — | Not supported by Thingino |
| Wyze V2 | Ingenic T20 (MIPS) | Out of scope — focusing on V3 only |

## GPIO and PWM Map (Wyze V3 / T31)

```
GPIO 47  — 850nm IR LEDs (GPIO only, no PWM)
GPIO 49  — 940nm IR LEDs (PWM channel 0 on T31!) ← primary TX path
GPIO 52  — IR cut filter (open)
GPIO 53  — IR cut filter (close)
GPIO 38  — Red status LED (active low)
GPIO 39  — Blue status LED (active low)
GPIO 51  — Reset button
GPIO 48  — MMC power (active low)
GPIO 57  — WLAN enable
GPIO 59  — MMC card detect
```

### PWM Control (940nm LEDs)

The 940nm LEDs (GPIO 49) map to hardware PWM channel 0 on the T31 SoC. This enables precision modulation from C code:

```c
// ioctl commands on /dev/pwm
PWM_CONFIG       0x001  // Set period + duty + polarity (nanoseconds)
PWM_CONFIG_DUTY  0x002  // Change duty only (fast — use for OOK modulation)
PWM_ENABLE       0x010  // Enable channel
PWM_DISABLE      0x100  // Disable channel
```

Shell shorthand (for testing only — too slow for data):
```bash
pwm-ctrl 49 50   # GPIO 49 at 50% duty
pwm -c 0 -P 1000000 -D 500000 -p 1 -e  # PWM0, 1ms period, 50% duty
```

**850nm LEDs (GPIO 47)** are GPIO-only: on/off, no PWM. Useful for visible-glow testing (faint red) but not for high-speed modulation.

## PC-to-Camera Connectivity

Primary connection is **USB Direct / RNDIS** (no WiFi). The camera's USB port carries both power and data.

### USB Modes (via Thingino)
- **USB Direct / RNDIS**: Network-over-USB. Camera gets an IP, SSH and RTSP work over the USB cable.
- **CDC-NCM**: Alternative USB networking mode.
- **USB Direct NCM**: Acts as DHCP server for plug-and-play networking.

WiFi is **not used** in this project. Most use cases have cameras connected directly to a laptop/PC via USB.

### System Topology (Single-PC Test Bench)

```
         USB              IR              USB
    [PC] <==> [Cam A] <==> [Cam B] <==> [PC]
    port1    tx/rx         tx/rx        port2
```

Both cameras connected to the same PC via USB. Cameras pointed at each other for IR communication.

### Duplex Mode

IR communication is **full-duplex**. Both cameras can transmit and receive simultaneously — IR LEDs point outward (same direction as the lens), so a camera's own LEDs cannot blind its own sensor. However, signal-to-noise may degrade in full-duplex if both cameras' IR interferes at the receiver. Half-duplex (taking turns) is available as a fallback.

## Development Phases

### Phase 0 — Infrastructure (current)

Prove the basic toolchain works over USB-only, no IR yet.

1. **0.1 — Repeatable SSH over USB**: Paramiko connects to both cameras reliably
2. **0.2 — Code deployment**: SCP files to camera's JFFS2 overlay, verify persistence
3. **0.3 — RTSP video over USB**: Capture frames from Prudynt-T via OpenCV
4. **0.4 — GPIO toggle + verify**: Toggle IR LEDs on Cam A, verify brightness change in Cam B's RTSP stream

### Phase 1 — Proof of Concept (shell-based)

IR modulation via shell commands (~5-10 bps). Slow but proves the concept end-to-end.

### Phase 2 — Direct GPIO/PWM (on-camera C)

C program on camera uses `/dev/pwm` ioctls for high-speed 940nm modulation (~100-1000 bps).

### Phase 2+ — Rolling Shutter Exploitation

Use rolling shutter timing to encode multiple bits per frame (~1-10 kbps). Research phase.

## Setup

### Host Machine
```bash
pip install opencv-python numpy paramiko matplotlib pytest
```

### Wyze V3 Cameras (Thingino)
1. Identify your V3 hardware variant (WiFi chip — check via `dmesg` after first flash)
2. Download the matching Thingino firmware image from https://github.com/themactep/thingino-firmware
3. Flash via SD card: rename image to `autoupdate-full.bin`, place on FAT32 SD card, boot camera
4. Connect camera to PC via USB cable (RNDIS mode)
5. Verify SSH: `ssh root@<cam_ip>` (default creds: `root/root`)
6. Disable day/night auto-switching if needed (disable `daynightd`)
7. Verify RTSP: `ffplay rtsp://<cam_ip>/ch0` (Prudynt-T, port 554, creds `thingino/thingino`)

### Code Deployment to Camera
```bash
# SCP a binary or script to the camera (persists across reboots via JFFS2 overlay)
scp my_program root@<cam_ip>:/usr/local/bin/
ssh root@<cam_ip> "chmod +x /usr/local/bin/my_program && /usr/local/bin/my_program"
```

### Cross-Compilation
- Wyze V3 (T31): MIPS cross-compiler via Thingino's Buildroot toolchain
- Alternatively: `mipsel-linux-gnu-gcc` with appropriate sysroot

## Project Structure

```
transmitter/   — code that runs ON the transmitting camera (shell scripts, C GPIO/PWM)
receiver/      — code that runs on host or receiving camera (frame analysis)
protocol/      — encoding, framing, link layer (pure Python, shared)
host/          — test orchestration (runs on host PC, SSHs into cameras)
benchmarks/    — range/speed test scripts and results
tests/         — pytest unit tests
docs/          — hardware notes, GPIO pinouts
```

## IR Channel Limitations

The IR link between cameras is a **very slow serial channel**. It cannot carry video or audio.

| Phase | Speed | Realistic payload |
|-------|-------|-------------------|
| Phase 0 (infra) | N/A | Connectivity and toolchain validation |
| Phase 1 (shell) | 5-10 bps | Trigger signals, status codes |
| Phase 2 (PWM) | 100-1000 bps | Short text, sensor readings, commands |
| Phase 2+ (rolling shutter) | 1-10 kbps | Larger text, small files (slowly) |

Compressed audio needs ~8 kbps minimum (codec2). Video is completely out of reach.

## Common Pitfalls

- **940nm is invisible**: You can't see 940nm IR. Use a phone camera to verify, or verify programmatically via the other camera's RTSP brightness. 850nm (GPIO 47) has a faint visible red glow for quick sanity checks.
- **Day/night auto-switching**: Thingino's `daynightd` may toggle IR based on ambient light. Disable it before testing: `killall daynightd` or disable in config.
- **RTSP stream latency**: Prudynt-T adds 0.5-2s latency. Account for this in TX/RX synchronization. Use timestamps, not assumed timing.
- **RTSP credentials**: Prudynt-T defaults to `thingino/thingino`. OpenCV needs them in the URL: `rtsp://thingino:thingino@<ip>/ch0`.
- **Frame rate drops at night**: Sensor drops to 10-15fps in night/IR mode due to longer exposure. This directly reduces receive bandwidth.
- **850nm vs 940nm for TX**: 940nm has hardware PWM (fast modulation). 850nm is GPIO-only (on/off). Use 940nm for data transmission.
- **Rolling shutter direction**: Stripe patterns from rolling shutter depend on scan direction (typically top-to-bottom). Verify orientation before implementing stripe decoder.
- **Full-duplex SNR**: Both cameras transmitting simultaneously is physically possible but may degrade signal-to-noise. Test full-duplex vs half-duplex BER early.
- **JFFS2 overlay is small**: The writable config partition is ~224 KB. For larger files, use the SD card.
- **USB port assignment**: With two cameras on one PC, USB device IPs may swap on reboot. Use MAC-based DHCP or static IP assignment to keep them stable.

## Running Tests

```bash
pytest tests/ -v
```

## Key References

- Thingino firmware: https://github.com/themactep/thingino-firmware
- Prudynt-T streamer: https://github.com/gtxaspec/prudynt-t
- ingenic-pwm: https://github.com/gtxaspec/ingenic-pwm
- OpenIPC (alternative firmware): https://github.com/OpenIPC
- Ingenic T31 datasheet: search "Ingenic T31 programming manual"
- Rolling shutter OCC research: https://pmc.ncbi.nlm.nih.gov/articles/PMC7061997/
