"""End-to-end IR message sender.

Usage:
    python -m host.send_message MESSAGE             # PWM mode (fast, ~10 bps)
    python -m host.send_message --shell MESSAGE     # Shell mode (slow, ~1 bps)
"""
import sys
import time

from protocol.frame import encode_frame
from receiver.rx_stream import Receiver, decode_samples
from host.ssh import ssh_cmd
from host.config import (
    TX_IP, RX_HOST, RX_RTSP, IR940_PIN,
    BIT_DURATION_S, BIT_DURATION_US,
    PWM_BIT_DURATION_S, PWM_BIT_DURATION_US,
)


def main():
    args = sys.argv[1:]
    use_shell = False

    if "--shell" in args:
        use_shell = True
        args.remove("--shell")

    if not args:
        print("Usage: python -m host.send_message [--shell] MESSAGE")
        sys.exit(1)

    message = args[0]
    if len(message) > 255:
        print("Error: message must be 255 characters or fewer")
        sys.exit(1)

    if use_shell:
        from transmitter.tx_shell import transmit
        bit_s = BIT_DURATION_S
        bit_us = BIT_DURATION_US
        mode = "shell"
    else:
        from transmitter.tx_pwm import transmit
        bit_s = PWM_BIT_DURATION_S
        bit_us = PWM_BIT_DURATION_US
        mode = "PWM"

    symbols = encode_frame(message)
    symbol_duration_s = bit_s / 2
    tx_duration = len(symbols) * symbol_duration_s
    bps = 1.0 / bit_s

    print(f"=== IR Message Transfer ({mode}, {bps:.0f} bps) ===")
    print(f"Message: '{message}'")
    print(f"Frame: {len(symbols) // 2} data bits, {len(symbols)} symbols, ~{tx_duration:.1f}s")
    print()

    # Prepare cameras
    print("Preparing cameras...")
    ssh_cmd(RX_HOST, "ircut off")
    ssh_cmd(TX_IP, f"gpio set {IR940_PIN} 0")
    time.sleep(1)

    # Start receiver
    print("Starting receiver...")
    rx = Receiver(RX_RTSP)
    rx.start()
    time.sleep(3)

    # Transmit
    print("Starting transmitter...")
    transmit(TX_IP, message, bit_duration_us=bit_us)

    # Wait for transmission + RTSP latency to settle
    # TX blocks over SSH, but add extra time for stream buffering
    extra_wait = max(5, tx_duration * 0.3 + 5)
    print(f"Waiting {extra_wait:.0f}s for stream to catch up...")
    time.sleep(extra_wait)

    # Decode
    samples = rx.stop()
    print()

    result = decode_samples(
        samples, bit_duration=bit_s,
        save_plot="/data/python/light-cam-canto/rx_debug.png",
    )

    print()
    print("=" * 40)
    if result is not None:
        print(f"DECODED: '{result}'")
        if result == message:
            print("SUCCESS — message received correctly!")
        else:
            print(f"MISMATCH — expected '{message}', got '{result}'")
    else:
        print("FAILED — could not decode message")


if __name__ == "__main__":
    main()
