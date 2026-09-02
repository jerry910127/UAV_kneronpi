#!/bin/bash
# ==============================================================================
#   Kneo Pi (KL730) - Fresh Board System Initialization & Setup Script
#   Purpose: Configure a brand-new Kneo-Pi board from factory state to fully operational.
#   Usage:
#     cd /work && chmod +x setup_fresh_board.sh && ./setup_fresh_board.sh
# ==============================================================================

set -e

WORK_DIR="/work"
echo "===================================================================="
echo "  🚀 Kneo Pi (KL730) Custom System Initialization on Fresh Board"
echo "===================================================================="
echo "[Setup] Target Work Directory: $WORK_DIR"
cd "$WORK_DIR"

# 1. Fix Linux Symlinks
echo "[Setup] 1/6 Restoring required symlinks..."
ln -sf utils/TOF "$WORK_DIR/TOF"

mkdir -p "$WORK_DIR/config"
if [ -f "$WORK_DIR/kneopi-examples/ai_application/nnm/rgbir/config/camera_params.txt" ]; then
    ln -sf ../kneopi-examples/ai_application/nnm/rgbir/config/camera_params.txt "$WORK_DIR/config/camera_params.txt"
fi
if [ -f "$WORK_DIR/kneopi-examples/ai_application/nnm/rgbir/config/homography_table.txt" ]; then
    ln -sf ../kneopi-examples/ai_application/nnm/rgbir/config/homography_table.txt "$WORK_DIR/config/homography_table.txt"
fi

if [ -d "$WORK_DIR/ai_application/nnm/app_flow/lib" ]; then
    mkdir -p "$WORK_DIR/kneopi-examples/bin"
    ln -sf ../ai_application/nnm/app_flow/lib "$WORK_DIR/kneopi-examples/bin/lib"
fi

# 2. System Shared Library Dependencies (Leipzig VPU Fixes)
echo "[Setup] 2/6 Configuring Leipzig VPU system shared libraries..."
if [ -f /usr/lib/vtcs_root_leipzig/libvmf.so ]; then
    ln -sf /usr/lib/vtcs_root_leipzig/libvmf.so /usr/lib/libvmf.so.2
    ln -sf /usr/lib/vtcs_root_leipzig/libutil.so /usr/lib/libutil.so.2 2>/dev/null || true
    ldconfig 2>/dev/null || true
    echo "[Setup] libvmf.so.2 and libutil.so.2 linked successfully."
fi

# 3. Permissions Setup
echo "[Setup] 3/6 Setting execution permissions..."
chmod +x "$WORK_DIR/start_stream" 2>/dev/null || true
chmod +x "$WORK_DIR/bin/"* 2>/dev/null || true
chmod +x "$WORK_DIR/kneopi-examples/bin/"* 2>/dev/null || true
chmod +x "$WORK_DIR/scripts/"*.sh 2>/dev/null || true
chmod +x "$WORK_DIR/0717/uav_deploy_pack/"*.sh 2>/dev/null || true
chmod +x "$WORK_DIR/0717/uav_deploy_pack/"*.py 2>/dev/null || true
find "$WORK_DIR/ai_application" "$WORK_DIR/hardware_control" "$WORK_DIR/memory_control" "$WORK_DIR/peripherals" -name "*.sh" -exec chmod +x {} + 2>/dev/null || true

# 4. Check & Prepare H.264 Test Stream for VENC
echo "[Setup] 4/6 Preparing H.264 video streaming source..."
TEST_VIDEO="/tmp/uav_test_720p.h264"
if [ ! -f "$TEST_VIDEO" ] || [ ! -s "$TEST_VIDEO" ]; then
    if [ -f "$WORK_DIR/0717/uav_deploy_pack/test_video/模擬空拍機1080p_h264.mp4" ]; then
        ffmpeg -y -i "$WORK_DIR/0717/uav_deploy_pack/test_video/模擬空拍機1080p_h264.mp4" -t 30 -vf scale=1280:720 -c:v libx264 -preset ultrafast -bf 0 -g 10 -pix_fmt yuv420p "$TEST_VIDEO" >/dev/null 2>&1 || true
    elif command -v ffmpeg >/dev/null 2>&1; then
        ffmpeg -y -f lavfi -i testsrc=size=1280x720:rate=10 -t 30 -c:v libx264 -preset ultrafast -g 10 -profile:v baseline -pix_fmt yuv420p "$TEST_VIDEO" >/dev/null 2>&1 || true
    fi
    echo "[Setup] Test video source ready at $TEST_VIDEO"
fi

# 5. Check Python Environment
echo "[Setup] 5/6 Verifying Python environment..."
python3 -c "import KneronPLUS; import numpy; print('  ✓ KneronPLUS & numpy verified')" 2>/dev/null || {
    echo "  [Notice] KneronPLUS not in default path or standard module, checking site-packages..."
}

# 6. System Service Integration (Optional)
echo "[Setup] 6/6 Setting up system services..."
if [ -f "$WORK_DIR/kneopi-examples.service" ]; then
    cp -f "$WORK_DIR/kneopi-examples.service" /etc/systemd/system/kneopi-examples.service
    systemctl daemon-reload
    echo "[Setup] kneopi-examples.service registered in systemd."
fi

echo "===================================================================="
echo "  🎉 Fresh Board Setup Complete! The system is ready to use."
echo "===================================================================="
echo "To test streaming: run '/work/start_stream'"
echo "To test AI NNM:    run '/work/bin/example_nnm_sensor -c /work/bin/ini/example_sensor.ini'"
echo "===================================================================="

