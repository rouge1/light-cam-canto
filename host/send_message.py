"""End-to-end IR message sender.

Usage:
    python -m host.send_message "HELLO"
"""
import sys
import time

from protocol.frame import encode_frame
from transmitter.tx_shell import transmit
from receiver.rx_stream import Receiver, decode_samples
from host.ssh import ssh_cmd
from host.config import TX_IP, RX_HOST, RX_RTSP, IR940_PIN, BIT_DURATION_S, BIT_DURATION_US


def main():
    if len(sys.argv) < 2:
        print("Usage: python -m host.send_message MESSAGE")
        sys.exit(1)

    message = sys.argv[1]
    if len(message) > 255:
        print("Error: message must be 255 characters or fewer")
        sys.exit(1)

    symbols = encode_frame(message)
    symbol_duration_s = BIT_DURATION_S / 2
    tx_duration = len(symbols) * symbol_duration_s
    rx_duration = tx_duration + 10

    print(f"=== IR Message Transfer ===")
    print(f"Message: '{message}'")
    print(f"Frame: {len(symbols) // 2} data bits, {len(symbols)} symbols, ~{tx_duration:.0f}s")
    print()

    # Prepare both cameras
    print("Preparing cameras...")
    ssh_cmd(RX_HOST, "ircut off")
    ssh_cmd(TX_IP, f"gpio set {IR940_PIN} 0")
    time.sleep(1)

    # Start receiver
    print("Starting receiver...")
    rx = Receiver(RX_RTSP)
    rx.start()
    time.sleep(3)

    # Transmit (blocks until done)
    print("Starting transmitter...")
    transmit(TX_IP, message, bit_duration_us=BIT_DURATION_US)

    # Wait for RTSP latency to catch up
    print("Waiting for stream to catch up...")
    time.sleep(5)

    # Decode
    samples = rx.stop()
    print()

    result = decode_samples(
        samples, bit_duration=BIT_DURATION_S,
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
