# Kneo-Pi UAV Streaming System (自研無人機即時圖傳串流獨立模組)

本模組為專為 **Kneron Kneo Pi (KL730)** 與地面接收站打造的高效能、窄頻寬 (< 300 kbps)、硬體加速無人機圖傳系統。

---

## 📁 目錄結構說明

- **`uav_system/`**：【機載端 UAV 核心子系統】(Kneo Pi 運行)
  - `bin/`：已編譯完成之 VPU 硬體編碼器 (`venc1`)、AI 推論檔與模型庫 (`nef/`)
  - `daemon/`：`uav_daemon.py`（自適應探測與守護核心）、`cam_probe.py`（相機匯流排精準診斷）、`test_video/`（測試視訊庫）
  - `config/`：雙鏡頭相機幾何校正參數 (`camera_params.txt`, `homography_table.txt`, `rgbir.cfg`)
  - `src/`：機載端完整 C/C++ 原始碼（包含 `venc1.c`、EDMC 記憶體驅動、周邊控制與 CMakeLists）
  - `start_stream`：機載端核心啟動腳本
- **`ground_station/`**：【地面站端 PC 核心】(地端電腦運行)
  - `ground_station.py`：30 FPS DIS 光流雙向插幀、即時 HUD 遙測接收器
  - `requirements.txt`：地面站 Python 依賴包
- **`start_stream`**：機載端全域一鍵啟動入口
- **`setup_fresh_board.sh`**：新板子自動初始化與環境修復工具
- **`release/`**：獨立發布安裝包 (`uav_system_v1.0.tar.gz`, `ground_station_v1.0.zip`)

---

## 🚀 常用啟動指令

進入 `uav_streaming` 目錄即可執行：

```bash
cd /work/uav_streaming

# 1. 啟動 USB UVC 鏡頭實時圖傳 (透過 FIFO 管道與 VPU 硬體 H.265 編碼)：
./start_stream --usb 192.168.137.1

# 2. 啟動 MIPI CSI 鏡頭實時圖傳：
./start_stream --mipi 192.168.137.1

# 3. 播放街景 Traffic 測試影片 (1280x720 10fps ABR 控制)：
./start_stream --traffic 192.168.137.1

# 4. 播放自訂測試影片：
./start_stream --video old_town_cross.mp4 192.168.137.1

# 5. 自適應探測模式 (自動監聽地面站心跳封包並連線)：
./start_stream
```
