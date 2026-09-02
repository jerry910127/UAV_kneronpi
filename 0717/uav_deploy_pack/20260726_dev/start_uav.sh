#!/bin/bash
# =====================================================================
#   Kneo Pi (KL730 UAV) One-Touch Streaming Launcher
#   Supports Zero-Config Auto-Discovery or Optional Target IP Override
# =====================================================================

REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$REAL_PATH")" && pwd)"
BIN_DIR="$SCRIPT_DIR/bin"
if [ ! -d "$BIN_DIR" ]; then
    BIN_DIR="/work/kneopi-examples/bin"
fi

TEST_VIDEO="/tmp/uav_test_720p.h264"
MP4_SOURCE="$SCRIPT_DIR/test_video/模擬空拍機1080p_h264.mp4"

echo "=================================================="
echo "   Kneo Pi (KL730) UAV Streaming Launcher        "
echo "=================================================="

# 1. Clean up any existing uav_daemon or venc1 zombie processes
pkill -f uav_daemon.py 2>/dev/null
pkill -f venc1 2>/dev/null
pkill -f example_nnm_sensor 2>/dev/null
sleep 0.5

# 2. Check/Generate test video if needed
if [ ! -f "$TEST_VIDEO" ] && [ -f "$MP4_SOURCE" ]; then
    echo "[UAV Launcher] Generating H.264 test stream at $TEST_VIDEO..."
    ffmpeg -y -i "$MP4_SOURCE" -t 30 -vf scale=1280:720 -c:v libx264 -preset ultrafast -bf 0 -g 10 -pix_fmt yuv420p "$TEST_VIDEO" >/dev/null 2>&1
fi

# 3. Pass CLI arguments to uav_daemon.py
python3 "$SCRIPT_DIR/uav_daemon.py" "$@"
