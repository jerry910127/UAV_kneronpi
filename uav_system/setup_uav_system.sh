#!/bin/bash
# ==============================================================================
#   Kneo Pi (KL730) - UAV System Module Initialization Script (Scheme A)
#   Purpose: Configure uav_system without touching factory /work/kneopi-examples
#   Usage:
#     cd /work/uav_system && chmod +x setup_uav_system.sh && ./setup_uav_system.sh
# ==============================================================================

set -e

UAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "===================================================================="
echo "  🚀 Kneo Pi (KL730) UAV System Modular Initialization (Scheme A)"
echo "===================================================================="
echo "[Setup] UAV Module Location: $UAV_ROOT"

# 1. System Shared Library Dependencies (Leipzig VPU Fixes)
echo "[Setup] 1/5 Configuring Leipzig VPU system shared libraries..."
if [ -f /usr/lib/vtcs_root_leipzig/libvmf.so ]; then
    ln -sf /usr/lib/vtcs_root_leipzig/libvmf.so /usr/lib/libvmf.so.2
    ln -sf /usr/lib/vtcs_root_leipzig/libutil.so /usr/lib/libutil.so.2 2>/dev/null || true
    ldconfig 2>/dev/null || true
    echo "[Setup] libvmf.so.2 and libutil.so.2 linked successfully."
fi

# 2. Setup internal symlinks inside uav_system
echo "[Setup] 2/5 Configuring internal module symlinks..."
if [ -d "$UAV_ROOT/src/ai_application/nnm/app_flow/lib" ]; then
    mkdir -p "$UAV_ROOT/bin"
    ln -sf ../src/ai_application/nnm/app_flow/lib "$UAV_ROOT/bin/lib"
fi
# Optional: convenience link at /work/start_stream -> /work/uav_system/start_stream
if [ "$UAV_ROOT" = "/work/uav_system" ]; then
    ln -sf /work/uav_system/start_stream /work/start_stream 2>/dev/null || true
fi

# 3. Permissions Setup
echo "[Setup] 3/5 Setting execution permissions..."
chmod +x "$UAV_ROOT/start_stream" "$UAV_ROOT/setup_uav_system.sh" 2>/dev/null || true
chmod +x "$UAV_ROOT/bin/"* 2>/dev/null || true
chmod +x "$UAV_ROOT/daemon/"*.py "$UAV_ROOT/daemon/"*.sh 2>/dev/null || true
find "$UAV_ROOT/src" -name "*.sh" -exec chmod +x {} + 2>/dev/null || true

# 4. Check & Prepare H.264 Test Stream for VENC
echo "[Setup] 4/5 Preparing H.264 video streaming source..."
TEST_VIDEO="/tmp/uav_test_720p.h264"
if [ ! -f "$TEST_VIDEO" ] || [ ! -s "$TEST_VIDEO" ]; then
    if [ -f "$UAV_ROOT/daemon/test_video/模擬空拍機1080p_h264.mp4" ]; then
        ffmpeg -y -i "$UAV_ROOT/daemon/test_video/模擬空拍機1080p_h264.mp4" -t 30 -vf scale=1280:720 -c:v libx264 -preset ultrafast -bf 0 -g 10 -pix_fmt yuv420p "$TEST_VIDEO" >/dev/null 2>&1 || true
    elif command -v ffmpeg >/dev/null 2>&1; then
        ffmpeg -y -f lavfi -i testsrc=size=1280x720:rate=10 -t 30 -c:v libx264 -preset ultrafast -g 10 -profile:v baseline -pix_fmt yuv420p "$TEST_VIDEO" >/dev/null 2>&1 || true
    fi
    echo "[Setup] Test video source ready at $TEST_VIDEO"
fi

# 5. System Service Integration (Optional)
echo "[Setup] 5/5 Configuring UAV autostart systemd service..."
SERVICE_FILE="/etc/systemd/system/uav_system.service"
cat << 'EOF' > "$SERVICE_FILE"
[Unit]
Description=Kneo Pi UAV Adaptive Streaming System
After=network.target local-display.service

[Service]
Type=simple
Environment="DISPLAY=:0"
WorkingDirectory=/work/uav_system
ExecStart=/bin/bash /work/uav_system/start_stream
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload 2>/dev/null || true
echo "[Setup] uav_system.service registered in systemd."

echo "===================================================================="
echo "  🎉 UAV System Modular Setup Complete!"
echo "===================================================================="
echo "To start streaming: /work/uav_system/start_stream"
echo "To start AI NNM:    /work/uav_system/bin/example_nnm_sensor -c /work/uav_system/bin/ini/example_sensor.ini"
echo "To enable on boot:  systemctl enable uav_system.service --now"
echo "===================================================================="
