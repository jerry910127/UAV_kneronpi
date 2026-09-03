#!/bin/bash
# Kneo Pi Deploy and Run Script (UAV side)
# 運作流程：此腳本將自動把打包的修改檔案分發至開發板專案中、自動編譯，並列出啟動指令。

# 1. 設定開發板上的專案根目錄路徑
PROJECT_DIR="/work/kneopi-examples"

if [ ! -d "$PROJECT_DIR" ]; then
    # 如果找不到，嘗試使用當前執行路徑的上一層作為專案根目錄
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
fi

echo "=================================================="
echo "  Kneo Pi (KL730) Scheme B 專案自動部署與編譯腳本"
echo "=================================================="
echo "[Deploy] 專案根目錄定位為: $PROJECT_DIR"

if [ ! -d "$PROJECT_DIR/ai_application" ] || [ ! -d "$PROJECT_DIR/hardware_control" ]; then
    echo "[Error] 定位路徑不正確，無法找到相應的專案目錄。"
    echo "請編輯此腳本，手動修改 PROJECT_DIR 變數。"
    exit 1
fi

# 2. 複製修改過的檔案至專案對應目錄
echo "[Deploy] 正在複製修改的原始碼與設定檔..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cp -f "$SCRIPT_DIR/display_liveview.cpp" "$PROJECT_DIR/ai_application/nnm/example_nnm_sensor/display_liveview.cpp"
cp -f "$SCRIPT_DIR/venc1.c" "$PROJECT_DIR/hardware_control/venc1/venc1.c"
cp -f "$SCRIPT_DIR/h26xenc.c" "$PROJECT_DIR/memory_control/v4l2_h26xe/h26xenc.c"
cp -f "$SCRIPT_DIR/example_webcam.ini" "$PROJECT_DIR/bin/ini/example_webcam.ini"

# 2.5 提取 H.264 原始串流 (強制關閉 B-Frame 以解決解碼/顯示順序抖動問題，並限制為 30 秒以加快處理)
if [ ! -f /tmp/uav_test_720p.h264 ]; then
    echo "[Deploy] 正在將 MP4 測試影片重新編碼為 720p 無 B-Frame 的 H.264 原始位元流..."
    ffmpeg -y -i "$SCRIPT_DIR/test_video/模擬空拍機1080p_h264.mp4" -t 30 -vf scale=1280:720 -c:v libx264 -preset ultrafast -bf 0 -g 10 -pix_fmt yuv420p /tmp/uav_test_720p.h264
else
    echo "[Deploy] /tmp/uav_test_720p.h264 已存在，跳過轉檔。"
fi

echo "[Deploy] 檔案分發完成。"

# 3. 編譯各個模組
echo "--------------------------------------------------"
echo "[Build] 1/3 正在重新編譯 hardware_control/venc1..."
echo "--------------------------------------------------"
cd "$PROJECT_DIR/hardware_control/venc1"
./build.sh

echo "--------------------------------------------------"
echo "[Build] 2/3 正在重新編譯 memory_control/v4l2_h26xe..."
echo "--------------------------------------------------"
cd "$PROJECT_DIR/memory_control/v4l2_h26xe"
rm -rf build && mkdir -p build && cd build
cmake ..
make

echo "--------------------------------------------------"
echo "[Build] 3/3 正在重新編譯 ai_application/nnm/example_nnm_sensor..."
echo "--------------------------------------------------"
cd "$PROJECT_DIR/ai_application/nnm"
# 呼叫 build_all.sh 編譯第 3 個範例 (example_nnm_sensor)
./build_all.sh 3

# 4. 複製編譯完成的執行檔至 20260726_dev/bin 與 $PROJECT_DIR/bin 目錄
echo "[Deploy] 正在複製編譯好的執行檔至 $SCRIPT_DIR/bin 與 $PROJECT_DIR/bin..."
mkdir -p "$PROJECT_DIR/bin" "$PROJECT_DIR/bin/ini"
mkdir -p "$SCRIPT_DIR/bin" "$SCRIPT_DIR/bin/ini"

cp -f "$SCRIPT_DIR/example_webcam.ini" "$PROJECT_DIR/bin/ini/example_webcam.ini"
cp -f "$SCRIPT_DIR/example_webcam.ini" "$SCRIPT_DIR/bin/ini/example_webcam.ini"
cp -f "$SCRIPT_DIR/example_webcam.ini" "$PROJECT_DIR/bin/ini/example_sensor.ini"
cp -f "$SCRIPT_DIR/example_webcam.ini" "$SCRIPT_DIR/bin/ini/example_sensor.ini"

cp -f "$PROJECT_DIR/hardware_control/venc1/bin/venc1" "$PROJECT_DIR/bin/venc1"
cp -f "$PROJECT_DIR/hardware_control/venc1/bin/venc1" "$SCRIPT_DIR/bin/venc1"

cp -f "$PROJECT_DIR/memory_control/v4l2_h26xe/build/bin/v4l2_h26xenc" "$PROJECT_DIR/bin/v4l2_h26xenc"
cp -f "$PROJECT_DIR/memory_control/v4l2_h26xe/build/bin/v4l2_h26xenc" "$SCRIPT_DIR/bin/v4l2_h26xenc"

cp -f "$PROJECT_DIR/ai_application/nnm/bin/example_nnm_sensor" "$PROJECT_DIR/bin/example_nnm_sensor"
cp -f "$PROJECT_DIR/ai_application/nnm/bin/example_nnm_sensor" "$SCRIPT_DIR/bin/example_nnm_sensor"

if [ -d "$PROJECT_DIR/bin/nef" ]; then
    cp -rf "$PROJECT_DIR/bin/nef" "$SCRIPT_DIR/bin/"
fi
if [ -d "$PROJECT_DIR/bin/Resource" ]; then
    cp -rf "$PROJECT_DIR/bin/Resource" "$SCRIPT_DIR/bin/"
fi

chmod +x "$SCRIPT_DIR/uav_daemon.py" "$SCRIPT_DIR/start_uav.sh" "$SCRIPT_DIR/cam_probe.py"
cp -f "$SCRIPT_DIR/start_uav.sh" "$SCRIPT_DIR/bin/start_uav.sh" 2>/dev/null
cp -f "$SCRIPT_DIR/start_uav.sh" "$PROJECT_DIR/bin/start_uav.sh" 2>/dev/null
cp -f "$SCRIPT_DIR/cam_probe.py" "$SCRIPT_DIR/bin/cam_probe.py" 2>/dev/null
cp -f "$SCRIPT_DIR/cam_probe.py" "$PROJECT_DIR/bin/cam_probe.py" 2>/dev/null
chmod +x "$SCRIPT_DIR/bin/start_uav.sh" "$PROJECT_DIR/bin/start_uav.sh" 2>/dev/null

# Create global shortcut uav_stream if permitted
if [ -w /usr/local/bin ]; then
    ln -sf "$SCRIPT_DIR/start_uav.sh" /usr/local/bin/uav_stream 2>/dev/null
fi

echo "=================================================="
echo "              🎉 部署與編譯完成！"
echo "=================================================="
echo "請選擇以下機載端極簡啟動方式："
echo ""
echo "📷 方式一：實體相機 / USB 鏡頭模式 (自動探測相機)"
echo "   uav_stream --cam"
echo "   (或指定 IP: uav_stream --cam 192.168.168.17)"
echo ""
echo "🎬 方式二：測試影片檔模擬模式 (預設)"
echo "   uav_stream --file"
echo "   (或直接輸入: uav_stream)"
echo ""
echo "🔍 方式三：相機規格硬體探測診斷"
echo "   uav_stream --scan"
echo "=================================================="
