
#!/bin/bash

set -e

echo "=================================="
echo "   Drone GCS Setup and Launcher"
echo "=================================="

# Update package lists
echo "[1/5] Updating packages..."
sudo apt update

# Install system dependencies
echo "[2/5] Installing dependencies..."
sudo apt install -y \
    python3-venv \
    python3-picamera2 \
    python3-opencv \
    python3-numpy

# Create virtual environment
echo "[3/5] Creating Python virtual environment..."
python3 -m venv --system-site-packages .venv

# Activate virtual environment
echo "[4/5] Activating virtual environment..."
source .venv/bin/activate

# Install Python dependencies
pip install fastapi uvicorn pymavlink pyserial

# Launch Drone GCS
echo "[5/5] Starting Drone GCS..."
python -m uvicorn launcher:app \
    --app-dir drone-gcs \
    --host 0.0.0.0 \
    --port 8000

