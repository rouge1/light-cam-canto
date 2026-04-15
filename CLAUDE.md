# Light-Cam-Canto (LiWiFi) — Development Guide

## Project Overview

IR light communication system between Wyze V3 cameras using their built-in IR illuminators. Cameras transmit data by modulating IR LEDs and receive by analyzing video frames for brightness changes. The project builds custom firmware on top of Thingino to enable on-camera TX/RX for a half-duplex reliable link with TCP-like protocol (SYN/ACK handshake, retransmit).

## Tech Stack

- **Python 3.10+** — receiver, protocol, benchmarks, test orchestration, calibration
- **C (cross-compiled for MIPS)** — on-camera GPIO transmitter, brightness monitor
- **OpenCV** — host-side frame capture, RTSP stream reading, calibration imaging
- **NumPy** — signal processing and decoding
- **requests** — Thingino web API control (LED toggle, ISP settings)
- **Matplotlib** — debug plots
- **pytest** — testing
- **Conda env: `light`** — `conda activate light`

## Firmware: Thingino (Custom Build)

This project builds **custom firmware** on top of Thingino (https://github.com/themactep/thingino-firmware). The firmware source is cloned into `thingino-firmware/` (not committed — has its own repo).

### Custom Firmware Changes

- **USB CDC-NCM networking** — added `BR2_PACKAGE_USBNET=y` and `BR2_PACKAGE_USBNET_USB_DIRECT_NCM=y` to defconfig (requires USB data cable, not yet tested)
- **BrightnessMonitor** — patched prudynt-t to expose per-frame brightness via `/run/prudynt/brightness`, per-block grid via `/run/prudynt/brightness_grid`, and pixel-level ROI via `/run/prudynt/brightness_roi` using `IMP_FrameSource_SnapFrame` API
- **AE Freeze control** — BrightnessMonitor reads `/run/prudynt/ae_freeze` and calls `IMP_ISP_Tuning_SetAeFreeze` to lock/unlock auto-exposure. Written by irlink during communication.

### BrightnessMonitor Output Files

| File | Format | Purpose |
|------|--------|---------|
| `/run/prudynt/brightness` | `<timestamp_ms> <mean> <max_block>` | Whole-frame brightness (fast reads) |
| `/run/prudynt/brightness_grid` | `<timestamp_ms> <b0> <b1> ... <b239>` | 20x12 block grid (legacy ROI detection) |
| `/run/prudynt/brightness_roi` | `<timestamp_ms> <mean>` | Pixel-level ROI brightness (high SNR) |
| `/run/prudynt/roi_config` | `<x> <y> <size>` | Control: set pixel ROI center and size (written by irlink) |
| `/run/prudynt/ae_freeze` | `0` or `1` | Write `1` to freeze AE, `0` to unfreeze |

The grid divides the 640x360 frame into 240 blocks (20x12, 32x30 pixels each). The pixel ROI reads a small square (default 15x15 pixels) around the calibrated transmitter position, sampling every pixel (no subsampling). Delta comparison: grid block gives +10-15 at 10-20ft, pixel ROI gives +120-150 at the same distance. The `irlink --pixel` mode and host-side `pixel_rx.py` both use pixel-level ROI for reliable decoding.

### Why Thingino

- Full kernel/system control — custom modules, ISP tuning, no cloud
- Prudynt-T RTSP streamer (dual streams, ONVIF, audio, motion)
- Writable JFFS2 overlay — SCP binaries that persist across reboots
- `daynightd` fully controllable or disableable

## Target Hardware

### Supported: Wyze Cam V3 only

| Camera | SoC | IR LEDs | FPS | Firmware |
|--------|-----|---------|-----|----------|
| Wyze V3 | Ingenic T31X (MIPS) | 4x 850nm + 4x 940nm | 15-20 | Thingino (custom) |

Both test cameras are the `wyze_cam3_t31x_gc2053_atbm6031` variant (ATBM6031 WiFi, T31X SoC).

## GPIO and PWM Map (Wyze V3 / T31)

```
GPIO 47  — 850nm IR LEDs (GPIO only, no PWM, faint visible red glow)
GPIO 49  — 940nm IR LEDs (PWM channel 0, invisible) ← primary TX path
GPIO 52  — IR cut filter (open)
GPIO 53  — IR cut filter (close)
GPIO 38  — Red status LED (active low)
GPIO 39  — Blue status LED (active low)
```

### IR Cut Filter

**Critical for IR communication.** The IR cut filter physically blocks infrared light from reaching the sensor. Must be opened before any IR reception:

```bash
ircut off           # Open filter (night mode) — IR passes through
ircut on            # Close filter (day mode) — IR blocked
killall daynightd   # Prevent auto-switching back
```

## Protocol Stack

### Manchester Encoding (IEEE 802.3)

Every data bit maps to two symbols with a guaranteed mid-bit transition:
- `0` → `[1, 0]` (falling edge)
- `1` → `[0, 1]` (rising edge)

Self-clocking — the receiver can never lose sync.

### Frame Format

```
[PREAMBLE 8] [SYNC 8] [LENGTH 8] [PAYLOAD N×8] [CRC-8] [POSTAMBLE 4] → Manchester encoded
```

- **Preamble**: `10101010` — clock recovery
- **Sync word**: `11001011` — frame boundary marker
- **Length**: payload byte count (0-255)
- **CRC-8/CCITT**: polynomial 0x07, covers length + payload
- **Postamble**: `1010` — clean termination

### Speeds Achieved

| Phase | Method | Speed | Status |
|-------|--------|-------|--------|
| Phase 1 | Shell `gpio set` + `usleep` | ~1 bps | Done, reliable |
| Phase 2 | C binary, sysfs GPIO | ~3 bps | Done, reliable |
| Phase 2+ | Hardware PWM ioctls | TBD | PWM_ENABLE fails — needs investigation |
| On-camera RX (grid) | irlink_rx + BrightnessMonitor grid | ~3 bps | Working (ROI + AE residual) |
| On-camera RX (pixel) | irlink --pixel + DPLL decode | ~4.2 bps | Working (120ms/symbol, delta 120+) |
| Half-duplex link | irlink (TX+RX+protocol) | ~3 bps | Working (AE freeze + raw block) |
| Dual LED TX | irlink (850nm + 940nm together) | ~3 bps | Working, 2x signal at distance |
| Host pixel RX | pixel_rx.py (RTSP + pixel ROI + DPLL) | ~4.2 bps | Working (120ms/symbol, delta 150+) |
| Raw TX | irlink tx (no handshake) | ~4.2 bps | Working (for pixel_rx or passive monitoring) |

## Project Structure

```
irlink/            — Combined half-duplex transceiver with protocol layer
  irlink.c         — TX+RX threads, SYN/ACK/DATA/PING/CAL protocol, DPLL decode, pixel ROI (C, MIPS)
  Makefile         — Cross-compilation, deploy to both cameras
protocol/          — Manchester encoding, CRC-8, frame encode/decode (pure Python)
transmitter/       — TX code: shell scripts, C GPIO binary, Makefile
  tx_pwm.c         — Phase 2: C binary using sysfs GPIO, cross-compiled for MIPS
  Makefile         — Cross-compilation via Thingino toolchain
receiver/          — RX code: standalone on-camera decoder + host-side RTSP decoder
  irlink_rx.c      — Standalone Manchester decoder with ROI tracking (C, MIPS)
  rx_stream.py     — Host-side RTSP brightness capture + dual-strategy decoder
  Makefile         — Cross-compilation for irlink_rx
host/              — Host-side orchestration: SSH, config, end-to-end CLI
  config.py        — Camera IPs, GPIO pins, timing constants
  ssh.py           — SSH/SCP wrappers (uses `scp -O` for Thingino compatibility)
  send_message.py  — CLI: `python -m host.send_message "HELLO"`
  cam_setup.sh     — Camera pre-flight: night mode, ircut, LEDs, prudynt, AE freeze
  cal_procedure.py — Pixel-level calibration: diff LED-on/off RTSP frames to find TX
  calibration.json — Saved calibration data (TX pixel coords per camera pair)
  pixel_rx.py      — Host-side RTSP pixel receiver (DPLL decode, 120ms/symbol)
photos/            — Calibration images, debug snapshots, calibration.html viewer
  find_transmitter.py — One-shot TX locator with bounding box
  grab_grid.py     — RTSP frame grab with brightness grid overlay
  calibration.html — Visual calibration report (serve with python -m http.server)
tests/             — pytest: 26 tests covering protocol layer
thingino-firmware/ — Thingino build tree (not committed, clone separately)
```

## Setup

### Host Machine
```bash
conda create -n light python=3.12 -y
conda activate light
pip install opencv-python numpy paramiko matplotlib pytest requests
```

### Camera SSH Setup
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
```

### Code Deployment to Camera
```bash
# Use scp -O (legacy mode) — Thingino lacks sftp-server
scp -O my_program root@da-camera1:/opt/bin/
ssh da-camera1 "chmod +x /opt/bin/my_program && /opt/bin/my_program"
```

### Cross-Compilation (C for MIPS)
```bash
cd transmitter && make        # Uses Thingino's Buildroot toolchain
make deploy                   # SCP to da-camera1
```

Toolchain path: `thingino-firmware/output/stable/wyze_cam3_t31x_gc2053_atbm6031-3.10.14-musl/host/bin/mipsel-linux-gcc`

### Building Custom Firmware
```bash
cd thingino-firmware
BOARD=wyze_cam3_t31x_gc2053_atbm6031 make defconfig
BOARD=wyze_cam3_t31x_gc2053_atbm6031 make
# Output: output/stable/.../images/thingino-wyze_cam3_t31x_gc2053_atbm6031.bin
# Flash: copy to SD card as autoupdate-full.bin, boot camera
```

### Rebuilding Patched Prudynt-T
```bash
# Edit source in dl/prudynt-t/git/src/ AND copy to build dir:
# output/stable/.../build/prudynt-t-<hash>/src/
# Then:
BOARD=wyze_cam3_t31x_gc2053_atbm6031 make prudynt-t-rebuild
# Deploy: scp -O .../per-package/prudynt-t/target/usr/bin/prudynt root@<cam>:/opt/bin/prudynt-patched
```

## Camera Filesystem

| Path | Type | Persists | Use |
|------|------|----------|-----|
| `/opt/` | JFFS2 (extras) | Yes, ~8MB | Deploy binaries here |
| `/etc/` | JFFS2 (config) | Yes, ~224KB | Config files (very small!) |
| `/usr/bin/`, `/usr/lib/` | SquashFS | Read-only | Stock firmware |
| `/tmp/` | RAM | No | Temp files |
| `/run/prudynt/` | RAM | No | Prudynt runtime data |

## Common Pitfalls

- **IR cut filter must be open** (`ircut off`). Without this, IR is physically blocked — no amount of software can detect it. After every reboot, the filter resets to day mode (closed). Kill `daynightd` to prevent auto-switching.
- **940nm is invisible**. Use a phone camera to verify, or check the other camera's RTSP stream. 850nm (GPIO 47) has a faint visible red glow for quick sanity checks.
- **Auto-exposure fights the signal**. The ISP compensates for brightness changes within ~200ms. This means sustained brightness levels are unreliable — use edge detection or transition-based decoding instead.
- **SCP needs `-O` flag**. Thingino doesn't have `sftp-server`. Use `scp -O` (legacy SCP protocol).
- **RTSP creds**: `thingino/thingino`. Web UI creds: `root/<your_password>`.
- **Overlay filesystem is tiny** (~224KB config + 8MB /opt). Don't try to `cp` large files to `/usr/bin/` — it will fill the overlay and corrupt the binary. Deploy to `/opt/bin/` instead.
- **Corrupted overlay recovery**: If you accidentally filled the overlay (e.g., failed `cp`), remove the overlay file: `rm /overlay/usr/bin/<file>` and reboot.
- **PWM hardware doesn't work** (as of current firmware). `ioctl(fd, PWM_ENABLE, channel)` returns "Operation not permitted" and kernel logs "pwm could not work!". The `pwm_device` is NULL — the kernel PWM subsystem doesn't initialize the device. Use sysfs GPIO toggle instead.
- **GPIO mux state persists**. Running `tx_pwm` switches GPIO 49 to PWM function mode (`gpio-diag PB17 func 0`). If the binary exits without resetting, the pin stays in PWM mode and GPIO commands have no effect. Reset with `gpio-diag PB17 func 1` or reboot.
- **IMP SDK is per-process**. You cannot call `IMP_ISP_Tuning_*` functions from a separate process — they require ISP initialization done by prudynt-t. The `libimp.so` on camera uses musl libc but was built against uclibc symbols; linking works at runtime but `dlopen` calls that hit ISP tuning will segfault from an uninitialized process.
- **IMP_ISP_Tuning_GetAeLuma is useless for IR detection**. It returns the auto-exposure's internal luminance estimate, which barely changes when IR LEDs toggle (delta ~3-4 units in daylight). Use raw frame brightness instead.
- **Prudynt-t BrightnessMonitor uses SnapFrame API**. `IMP_FrameSource_SnapFrame` grabs a single NV12 frame from an already-running channel without needing IVS binding. Works alongside prudynt-t's encoder pipeline.
- **Buildroot wipes source on rebuild**. Patches to `dl/prudynt-t/git/src/` are NOT used during build. You must also copy changes to `output/stable/.../build/prudynt-t-<hash>/src/`. Use Buildroot's patch system for permanent changes.
- **Frame rate is the receiver bottleneck**. At 15fps RTSP, each symbol needs ~3+ frames for reliable detection. Maximum practical bps = fps / (2 * samples_per_symbol) ≈ 3-5 bps via RTSP.
- **OTA sysupgrade can corrupt kernel module state**. The `-update.bin` may not fully update the kernel, leaving `sensor_gc2053_t31.ko` with unresolved symbols. Use full SD card flash (`autoupdate-full.bin`) for reliable firmware updates.
- **Both cameras must run the same firmware version**. A patched prudynt built against one firmware version will crash on another — config parsing produces garbage values (e.g., `1952542720` for `mic_sample_rate`), encoder initialization fails, and SnapFrame returns -1.
- **Stock prudynt must be stopped via init script, not kill**. Use `/etc/init.d/S31prudynt stop` to cleanly shut down. Direct `kill` or `killall` can leave the IMP SDK in a bad state (zombie process, locked ISP device). If the IMP state is stuck, reboot.
- **IR cut filter resets on prudynt restart too**. Not just on boot — restarting prudynt (even the patched version) triggers `daynightd` to re-evaluate and close the filter. Always run `ircut off; killall daynightd` after starting prudynt.
- **`/opt` may not auto-mount after fresh flash**. The JFFS2 partition at `/dev/mtdblock5` needs `mount -t jffs2 /dev/mtdblock5 /opt`. Check with `mount | grep opt`.
- **AE over-compensates for IR LEDs**. When the IR LED turns ON, the ISP reduces exposure, making the whole-frame MEAN brightness DROP (inverted from expected). The IR block gets brighter, but everything else gets darker. Use residual-based detection (block - mean) to cancel AE, not absolute brightness.
- **Whole-frame mean is useless for ROI detection**. At 2-3 feet, the IR LEDs occupy ~1 grid block out of 60. The mean brightness delta is ~1-3 units (noise level), but the specific block delta is 30-100+ units. Always calibrate and track the specific block.
- **Cameras too close (<6 inches) gives poor signal**. The IR LEDs flood the entire sensor when very close, and the image is out of focus (min focus ~50cm). Best results at 2-3 feet where the LEDs form a focused spot.
- **AE freeze is required for half-duplex protocol**. Without AE freeze, the camera's own LED reflections cause massive AE swings that take 3-5 seconds to settle after TX ends. With `IMP_ISP_Tuning_SetAeFreeze(ENABLE)`, exposure locks and raw block brightness changes are instantaneous and predictable. Freeze AE before comms, unfreeze after.
- **Self-reflection dominates raw brightness**. When a camera toggles its own IR LED, reflections off walls/ceiling cause block 45 to change by 8-10 units (vs 23-45 from the peer). With AE frozen, these reflections are small and consistent — not a problem. With AE active, the reflections trigger AE compensation that ruins subsequent reception.
- **Consecutive-frame filter prevents false activity detection**. Single-frame brightness spikes (from self-reflection or noise) trigger false IDLE→ACTIVE transitions. Requiring 2 consecutive frames above threshold eliminates these (reduced from 3 with AE freeze).
- **Inter-message gap must exceed SETTLE_MS**. Back-to-back transmissions (ACK immediately followed by DATA) merge into one continuous activity period for the receiver. The gap between messages must be at least SETTLE_MS (400ms) + 100ms margin so the receiver detects "end of TX" and decodes each message separately.
- **RX thread stack overflow on MIPS/musl**. The default pthread stack (~128KB on musl) overflows with large local arrays. `sample_t samples[8192]` (~128KB) must be declared static, not stack-local.
- **Boot automation via `/etc/rc.local`**. Both cameras have rc.local that auto-mounts /opt, swaps to patched prudynt, opens IR filter, and kills daynightd. Runs via `S94rc.local` at end of boot sequence.
- **Prudynt daynight overrides GPIO LED state**. Even after `gpio set 47`, prudynt's daynight daemon can immediately turn it off. You MUST disable daynight before toggling LEDs: `echo '{"daynight":{"enabled":false}}' | prudyntctl json -`. The `light` command is the reliable LED control: `light ir850 on`, `light ir940 on`, `light ir850 read` (returns 0 or 1). This is what the web UI uses internally.
- **`color status` and `imp-control ispmode` lie**. The `color` CLI reads a stale file (`/tmp/colormode.txt`), not actual ISP state. `imp-control ispmode 1` may silently fail. Use the Thingino web API instead: `POST /x/json-imp.cgi {"cmd":"color","val":1}`. Verify via heartbeat: `GET /x/json-heartbeat.cgi` (SSE, use `--max-time`) returns `color_mode` (0=color, 1=mono).
- **Thingino web API requires session cookies**. Login with `POST /x/login.cgi {"username":"root","password":"password"}` to get a session cookie. Cookies expire — always re-login before commands. The API endpoint is `/x/json-imp.cgi` with commands: `daynight`, `color`, `ircut`, `ir850`, `ir940`.
- **ircut_state semantics are inverted from what you'd expect**. In the heartbeat API: `ircut_state=0` means filter REMOVED (night mode, IR passes through = GOOD). `ircut_state=1` means filter SET (day mode, IR blocked = BAD). The `daynight night` API command sometimes re-closes the ircut filter — always set ircut as the LAST step after all other mode changes.
- **RTSP stream buffers stale frames**. When toggling LEDs and capturing via RTSP, the OpenCV VideoCapture buffer contains frames from before the LED state changed. Opening a fresh stream after the state change (rather than flushing frames on an existing stream) is the reliable approach.
- **OSD timestamp changes create false calibration peaks**. The on-screen timestamp (top-left corner) changes every second, creating large brightness deltas that can be mistaken for IR LEDs during frame differencing. Mask out the top 30 and bottom 30 pixel rows when searching for the TX peak.
- **Grid-based brightness (20x12 blocks) dilutes IR signal at distance**. At 10-20 feet, the IR LED spot is ~10-20 pixels across but the grid block is 32x30 pixels. The spot's +200 pixel delta gets averaged down to +10-15 in the block. Host-side pixel-level RTSP reading (`pixel_rx.py`) gives the full undiluted delta.
- **Dual LED TX (850nm + 940nm) doubles signal**. `irlink` now toggles both GPIO 47 and 49 simultaneously. At 10-20 feet: single LED gives +15 grid delta, dual gives +23. At pixel level: +200 for both.
- **Camera setup order matters**. The `cam_setup.sh` script must: (1) start prudynt, (2) kill daynightd, (3) set night/monochrome mode, (4) turn off LEDs, (5) set ircut filter LAST (because other commands re-close it).
- **RTSP first frames are black**. When opening a fresh RTSP stream, the first 5-15 frames are often completely black (codec init / ISP startup). Always flush 10-15 frames before using data. The calibration code does this (`grab_frames(cap, n=10)`), and pixel_rx flushes 15 frames on startup.
- **Calibration coordinates shift when cameras move**. Pixel calibration is position-sensitive — even a small camera bump invalidates the (x,y) coordinates. Re-run `python -m host.cal_procedure --no-interactive` after any physical change. Stale coordinates point at dark pixels, giving baseline=0 and failed decodes.
- **30fps RTSP caps symbol rate at ~120ms minimum**. At 30fps, frames arrive every ~33ms with occasional gaps up to 200ms. At 120ms/symbol, most symbols get 3+ samples. At 100ms, too many symbols fall entirely in frame gaps. The DPLL clock recovery handles jitter but cannot recover symbols with zero samples.
- **DPLL must work on raw sample times, not interpolated grid**. Linear interpolation between sparse RTSP samples (33ms apart) creates fake transitions during frame gaps. The DPLL must detect edges in the actual sample timestamps, not in the interpolated 1ms signal. The brute-force t0 search on the interpolated grid is kept as a fallback.
- **On-camera frame rate is ~18fps (not 30)**. BrightnessMonitor's `IMP_FrameSource_SnapFrame` on channel 1 (640x360) runs at ~18fps. Combined with the DPLL, 120ms/symbol still works (2+ samples per symbol). The host-side RTSP stream gives ~30fps — better oversampling.
- **`irlink --pixel` writes ROI config to `/run/prudynt/roi_config`**. BrightnessMonitor reads this every frame and outputs pixel ROI to `/run/prudynt/brightness_roi`. The config persists until overwritten or prudynt restarts. If you switch between `--pixel` and `--block` modes, the old ROI config file may still exist — harmless, since `read_brightness()` checks `pixel_x >= 0` first.

## On-Camera Transceiver (irlink)

The `irlink` binary is the combined half-duplex transceiver with a TCP-like protocol layer. It runs on-camera, reading brightness from BrightnessMonitor and toggling both IR LEDs (850nm + 940nm) simultaneously via sysfs GPIO for maximum signal strength.

### Protocol Message Types

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

Frame payload format: `[msg_type] [seq_num] [data...]`

### Usage

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

### AE Freeze During Communication

irlink freezes auto-exposure before any protocol exchange and unfreezes after. This is critical because:
- The camera's own LED reflections cause massive AE swings (3-5s settle time)
- With AE frozen, raw block brightness is stable and predictable
- The peer's IR LED causes a clean delta (23-45 units) on the tracked block
- Between communications, AE runs normally to adapt to ambient light

The freeze is controlled via `/run/prudynt/ae_freeze` — irlink writes `1` before starting, `0` when done. BrightnessMonitor in prudynt calls `IMP_ISP_Tuning_SetAeFreeze()`.

### ROI Calibration

Two calibration methods:

**On-camera (grid-based, legacy):** The 20x12 grid (240 blocks) finds the transmitter's block:
```bash
irlink calibrate    # toggle peer LED during 4s window
```

**Host-side (pixel-level, recommended):** Frame differencing finds the exact TX pixel:
```bash
conda activate light
python -m host.cal_procedure              # both directions, interactive
python -m host.cal_procedure --no-interactive  # automated
python -m host.cal_procedure --show        # show saved calibration
```
Saves results to `host/calibration.json`. The pixel coordinates are used by:
- `pixel_rx.py` — loads calibration automatically from the JSON file
- `irlink --pixel x,y` — pass the coordinates from calibration to the on-camera receiver

The host-side method gives delta +120-150 on-camera (via pixel ROI) and +200 via RTSP, vs +10-15 with the grid method at the same distance.

### Boot Automation

Both cameras run `/etc/rc.local` at boot (via `S94rc.local`):
1. Mounts `/opt` JFFS2 partition if not mounted
2. Stops stock prudynt, starts patched version
3. Opens IR cut filter, kills daynightd

### Deploying

```bash
# Build and deploy irlink to both cameras
cd irlink && make deploy

# Rebuild and deploy patched prudynt (with AE freeze + BrightnessMonitor)
cd thingino-firmware
BOARD=wyze_cam3_t31x_gc2053_atbm6031 make prudynt-t-rebuild
scp -O .../per-package/prudynt-t/target/usr/bin/prudynt root@<cam>:/opt/bin/prudynt-patched
# Restart prudynt on camera, then: ircut off; killall daynightd
```

### Camera Setup Script

Run before any calibration or communication to ensure cameras are in the correct state:
```bash
./host/cam_setup.sh                  # setup both cameras
./host/cam_setup.sh da-camera1       # one camera only
./host/cam_setup.sh --check          # verify only, don't fix
```

Checks and fixes: patched prudynt, night/monochrome mode, IR cut filter, daynightd killed, LEDs off, AE freeze idle, brightness grid active, irlink deployed. Uses Thingino web API (curl) for ISP controls, SSH for GPIO and process management. Sets ircut filter LAST because other commands re-close it.

### Host-side Pixel Receiver

For long-distance communication (10-20 feet) where the grid-based on-camera RX has insufficient signal:
```bash
conda activate light
python -m host.cal_procedure --no-interactive  # calibrate first
python -m host.pixel_rx --cam cam2             # auto-loads calibration, 160ms default
python -m host.pixel_rx --cam cam2 --symbol-ms 120  # faster speed
python -m host.pixel_rx --cam cam2 --tx-pos 385,178  # manual position
```

Reads the RTSP stream directly and tracks a small pixel ROI (15x15) around the calibrated TX position. Uses DPLL clock recovery with early-late gate for robust symbol extraction. Gets delta +150-200 at 10-20 feet vs +10-15 with the grid. Reliable at 120ms/symbol (~4.2 bps).

**Decoder architecture**: Two-pass decode — DPLL on raw sample timestamps first (handles irregular frame timing), brute-force t0 search on interpolated signal as fallback. The DPLL works directly with irregular RTSP sample times to avoid interpolation artifacts from frame gaps.

### After Fresh Firmware Flash

The `/opt` JFFS2 partition may not auto-mount after a fresh SD card flash:

```bash
mount -t jffs2 /dev/mtdblock5 /opt
mkdir -p /opt/bin
# Then SCP binaries to /opt/bin/
```

## Running Tests

```bash
conda activate light
cd /data/python/light-cam-canto
pytest tests/ -v
```

## Usage

```bash
# 1. Setup cameras (run after every reboot)
./host/cam_setup.sh

# 2. Calibrate (find TX pixel locations)
conda activate light
python -m host.cal_procedure --no-interactive
python -m host.cal_procedure --show    # verify calibration

# 3a. On-camera pixel RX (recommended, ~4.2 bps)
# Use pixel coords from calibration (e.g., cam2 sees cam1 at 385,178)
# On cam2: irlink listen --pixel 385,178 --speed 120
# On cam1: irlink tx "HELLO" --speed 120
# Or with protocol: irlink send "HELLO" --pixel <coords> --speed 120

# 3b. Host-side pixel RX (~4.2 bps, best SNR)
python -m host.pixel_rx --cam cam2 --symbol-ms 120    # auto-loads calibration
# Then trigger TX from cam1: ssh da-camera1 "/opt/bin/irlink tx HELLO --speed 120"

# 3c. On-camera grid RX (legacy, ~3 bps)
# On cam1: irlink listen --block 212
# On cam2: irlink send "HELLO" --block 113

# Legacy: send via Python host orchestration
python -m host.send_message "HELLO"
```

### Calibration Viewer

```bash
cd photos && python -m http.server 8888
# Open http://localhost:8888/calibration.html
```

## Key References

- Thingino firmware: https://github.com/themactep/thingino-firmware
- Prudynt-T streamer: https://github.com/gtxaspec/prudynt-t
- ingenic-pwm: https://github.com/gtxaspec/ingenic-pwm
- Ingenic T31 datasheet: search "Ingenic T31 programming manual"
- Rolling shutter OCC research: https://pmc.ncbi.nlm.nih.gov/articles/PMC7061997/
