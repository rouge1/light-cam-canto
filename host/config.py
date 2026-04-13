"""Camera and communication configuration."""

# Camera network config
TX_HOST = "da-camera1"
TX_IP = "192.168.50.110"
RX_HOST = "da-camera2"
RX_IP = "192.168.50.141"
RX_RTSP = f"rtsp://thingino:thingino@{RX_IP}/ch0"

# GPIO pins
IR940_PIN = 49
IR850_PIN = 47

# Timing
BIT_DURATION_S = 1.0
BIT_DURATION_US = 1_000_000

# Remote paths
TX_SCRIPT_PATH = "/opt/bin/tx_frame.sh"
