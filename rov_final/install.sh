#!/bin/bash
# install.sh — one-shot install for Raspberry Pi
# Run once after cloning: bash install.sh

set -e
echo "============================================"
echo "ROV Autonomy — Install"
echo "============================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# System packages
sudo apt-get update -qq
sudo apt-get install -y python3-pip python3-venv libopencv-dev python3-opencv \
    python3-picamera2 python3-libcamera libzmq3-dev python3-zmq

# Python venv
python3 -m venv --system-site-packages venv
source venv/bin/activate

# Python packages
pip install --upgrade pip
pip install \
    pyserial>=3.5 \
    numpy>=1.24 \
    matplotlib>=3.7 \
    msgpack>=1.0 \
    pyzmq>=25.0 \
    pytest>=7.0

# Enable UART (disable serial console, keep hardware UART)
if command -v raspi-config &>/dev/null; then
    echo "Enabling hardware UART..."
    sudo raspi-config nonint do_serial_hw 0
    sudo raspi-config nonint do_serial_cons 1
    echo "UART enabled — reboot required"
fi

# Create logs directory
mkdir -p "$SCRIPT_DIR/logs"

echo ""
echo "============================================"
echo "Install complete."
echo "Reboot required if UART was just enabled."
echo "Run with: bash run.sh"
echo "============================================"
