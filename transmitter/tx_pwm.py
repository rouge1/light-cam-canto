"""PWM-based IR transmitter (Phase 2).

Encodes a message into Manchester symbols, writes them to a file,
deploys to the camera, and runs the tx_pwm C binary for fast modulation.
"""
import tempfile
import os

from protocol.frame import encode_frame
from host.ssh import ssh_cmd, scp_to_camera
from host.config import TX_SCRIPT_PATH

TX_PWM_BINARY = "/opt/bin/tx_pwm"
TX_SYMBOLS_PATH = "/opt/bin/tx_symbols.txt"


def transmit(host: str, message: str, bit_duration_us: int = 100_000):
    """Encode a message and transmit via PWM.

    Args:
        host: Camera hostname or IP.
        message: Text message to send.
        bit_duration_us: Microseconds per data bit. Each Manchester symbol
            is half this. Default 100ms/bit = 10 bps.
    """
    symbols = encode_frame(message)
    symbol_duration_us = bit_duration_us // 2
    total_ms = len(symbols) * symbol_duration_us // 1000

    print(f"TX: '{message}' → {len(symbols) // 2} data bits, "
          f"{len(symbols)} symbols, ~{total_ms / 1000:.1f}s at {1_000_000 // bit_duration_us} bps")

    # Write symbols to temp file
    with tempfile.NamedTemporaryFile(mode="w", suffix=".txt", delete=False) as f:
        f.write("".join(str(s) for s in symbols))
        f.write("\n")
        local_path = f.name

    try:
        scp_to_camera(local_path, host, TX_SYMBOLS_PATH)

        print("TX: transmitting (PWM)...")
        timeout = max(30, total_ms // 1000 + 15)
        ssh_cmd(host, f"{TX_PWM_BINARY} {symbol_duration_us} {TX_SYMBOLS_PATH}",
                timeout=timeout)
        print("TX: done")
    finally:
        os.unlink(local_path)
