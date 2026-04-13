"""Shell-based IR transmitter with Manchester encoding.

Generates a toggle script from a message, deploys it to the camera,
and runs it. All timing is handled on-camera to avoid SSH latency.
"""
import tempfile
import os

from protocol.frame import encode_frame
from host.ssh import ssh_cmd, scp_to_camera
from host.config import IR940_PIN, BIT_DURATION_US, TX_SCRIPT_PATH


def build_tx_script(symbols: list[int], bit_duration_us: int = BIT_DURATION_US) -> str:
    """Build a shell script that toggles GPIO to transmit Manchester symbols."""
    symbol_duration_us = bit_duration_us // 2
    lines = [
        "#!/bin/sh",
        f"# TX: {len(symbols)} symbols at {symbol_duration_us}us/symbol",
        # Quiet period before transmission
        f"gpio set {IR940_PIN} 0",
        f"usleep {bit_duration_us}",
    ]

    for symbol in symbols:
        lines.append(f"gpio set {IR940_PIN} {symbol}")
        lines.append(f"usleep {symbol_duration_us}")

    lines.append(f"gpio set {IR940_PIN} 0")
    return "\n".join(lines) + "\n"


def transmit(host: str, message: str, bit_duration_us: int = BIT_DURATION_US):
    """Encode a message, deploy the TX script, and run it on the camera."""
    symbols = encode_frame(message)
    symbol_duration_us = bit_duration_us // 2
    total_seconds = len(symbols) * symbol_duration_us / 1_000_000

    print(f"TX: '{message}' → {len(symbols) // 2} data bits, {len(symbols)} symbols, ~{total_seconds:.0f}s")

    script = build_tx_script(symbols, bit_duration_us)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".sh", delete=False) as f:
        f.write(script)
        local_path = f.name

    try:
        scp_to_camera(local_path, host, TX_SCRIPT_PATH)
        print("TX: transmitting...")
        ssh_cmd(host, f"chmod +x {TX_SCRIPT_PATH} && {TX_SCRIPT_PATH}",
                timeout=int(total_seconds) + 30)
        print("TX: done")
    finally:
        os.unlink(local_path)
