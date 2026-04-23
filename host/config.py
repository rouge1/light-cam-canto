"""Camera and communication configuration."""

# Camera network config (cam1 transmits, cam2 receives)
TX_HOST = "da-camera1"
TX_IP = "192.168.50.110"
RX_HOST = "da-camera2"
RX_IP = "192.168.50.141"
RX_RTSP = f"rtsp://thingino:thingino@{RX_IP}/ch0"

# GPIO pins
IR940_PIN = 49
IR850_PIN = 47

# Timing — Phase 1 (shell, ~1 bps)
BIT_DURATION_S = 1.0
BIT_DURATION_US = 1_000_000

# Timing — Phase 2 (PWM, ~3 bps)
# At 15fps RTSP, 333ms/symbol → ~5 frames per symbol for reliable detection.
PWM_BIT_DURATION_S = 0.333
PWM_BIT_DURATION_US = 333_000

# Remote paths
TX_SCRIPT_PATH = "/opt/bin/tx_frame.sh"


# Adaptive symbol rate — ladder of known-good symbol times (ms/sym).
# irlink on-camera mirrors this list; both ends step ±1 rung at a time.
RATE_LADDER_MS = [60, 80, 100, 120, 160, 200]

# Δ (peak brightness above baseline, from cal_procedure.py) → starting rate.
# Ordered highest-Δ first; first match wins. Conservative picks from 2026-04-22 tests:
# Δ=79 decoded at 120ms non-FEC (2.98 bps). Δ=200 decoded at 70ms (morning).
_RATE_FROM_DELTA = (
    (180, 70),
    (130, 80),
    (90, 100),
    (60, 120),
    (0, 160),
)


def pick_initial_rate_ms(delta: int) -> int:
    """Pick a starting symbol_ms from the measured calibration Δ."""
    for threshold, rate_ms in _RATE_FROM_DELTA:
        if delta >= threshold:
            return rate_ms
    return RATE_LADDER_MS[-1]


# Adaptation thresholds (also mirrored in irlink.c):
PROBE_UP_AFTER = 5         # successful ACKed DATA frames before probing faster
FALLBACK_AFTER = 3         # retransmit-exhausted frames before stepping slower
SPLIT_BRAIN_TIMEOUT_MS = 15000   # no valid frame received → force slowest rung
PROBE_COOLDOWN_MS = 30000  # after a failed probe, don't retry that rung for this long
