# UAV Kneron Pi (KL730) 邊緣分工低頻寬圖傳與 AI 遙測系統

本專案是一個基於 **Kneo Pi (KL730)** 開發板與 **地面站電腦（Ground Station PC）** 所開發的高效能無人機低頻寬影像傳輸、AI 目標辨識與即時 30 FPS 插幀遙測系統。

---

## 🎯 系統核心架構 (Edge-Ground Scheme)

系統採用 **邊緣分工架構 (Scheme B)**，兼具極低傳輸頻寬與高流暢度體驗：
* **機載端 (Kneo Pi UAV)**：專注於高效能的 **VPU 硬體 H.265 視訊編碼**（10 FPS）與 **NPU YOLOv5 即時目標偵測**，不浪費機載算力進行插幀。
* **地面站 (Ground Station PC)**：透過 SRT 接收低碼率影像流，接收 UDP 遙測數據，並藉由 **DIS 高速光流法 (Optical Flow) 進行地端即時插幀補償 (10 FPS $\rightarrow$ 30 FPS)** 與 HUD 標註疊加，在極低頻寬條件下實現 **1.0x 真實速度的 30 FPS 流暢觀看體驗**。

---

## 📂 專案目錄結構 (Project Layout)

專案採用單一標準的 `/work` 根目錄結構：

```text
/work/
├── README.md                  # 本說明文件（專案總覽與操作指南）
├── start_stream               # 【UAV 入口】一鍵自癒式串流啟動腳本
├── setup_fresh_board.sh       # 【全新板子】環境初始化與自我修復腳本
├── bin/                       # 【UAV 核心二進位與守護進程】
│   ├── venc1                  # KL730 VPU 硬體 H.265 編碼與 SRT 切片發送器
│   ├── uav_daemon.py          # 自適應 UDP 9002 心跳偵測與串流守護程式
│   └── ground_station.py      # 地面端接收程式
├── ground_station/            # 【地面站端專區】（執行於地端 Windows/Linux/macOS 電腦）
│   ├── ground_station.py      # 地面站 HUD 接收器、DIS 光流插幀引擎 (30 FPS)
│   └── requirements.txt       # 地面站 Python 依賴套件 (opencv, numpy)
├── uav_system/                # 【UAV 系統整合模組】
│   ├── daemon/                # 守護程式與自適應管線
│   ├── config/                # 相機與雙鏡頭參數配置
│   └── src/                   # 核心源碼庫
└── kneopi-examples/           # 【Kneo Pi 官方 SDK 源碼與編譯區】
    ├── hardware_control/venc1/# venc1 C 語言原始碼與 CMakeLists.txt
    ├── ai_application/        # NPU YOLOv5 模型 (NEF) 與推論範例
    └── memory_control/        # EDMC 記憶體驅動模組
```

---

## ⚙️ 關鍵組件與技術參數

### 1. 機載端 (Airborne `work/` 目錄)
* **硬體 H.265 編碼 (`venc1.c`)**：
  * 解析度：`1280x720` (720p) @ 10 FPS。
  * 頻寬限制：編碼位元率硬性鎖定在 **`85,000 bps` (85 kbps)**，最大抑制 I-Frame 頻寬爆發。
  * 發送保護：設定 `SRTO_SNDTIMEO = 10` (10ms)，若遇到網路卡頓自動捨棄過期影格，避免緩衝積壓與圖傳延遲。
* **NPU YOLOv5 AI 偵測**：
  * 使用 Kneo Pi NPU 加速器即時辨識目標物，並透過 UDP Socket (Port 9001) 傳送邊框座標數據（每框 24 位元組），避免將標註渲染在視訊流中佔用額外編碼頻寬。

### 2. 地面站 (Ground PC `ground_station/` 目錄)
* **DIS 光流插幀 (Motion Interpolation)**：
  * 採用 `cv2.DISOpticalFlow` 演算法。
  * **降採樣加速 (FLOW_SCALE = 0.25)**：將 720p 影像降至 `320x180` 進行光流場計算，運算時間控制在 **< 1.5ms**，有效避免卡頓。
* **高精度 FPS 步進器**：
  * 使用 Python `time.perf_counter` 構建 33.33ms 的步進計時器，實現極穩定的 30.0 FPS HUD 渲染與輸出。

---

## 📊 效能基準測試數據 (Resource Benchmark Report)

本專案對比了 **FFmpeg 軟體編碼 (libx264)** 與 **Kneo Pi VPU 硬體編碼 (venc1)** 在處理相同影像來源時的系統負載表現：

| 監測項目 | FFmpeg 軟體編碼 (`libx264`) | Kneo Pi VPU 硬體編碼 (`venc1`) | 效能差異與效益 |
| :--- | :--- | :--- | :--- |
| **進程 CPU 平均使用率** | **12.23%** | **1.70%** | **CPU 負載降低約 86%** (~7.2 倍效能提升) |
| **進程 CPU 最高使用率** | **35.40%** | **1.70%** | 硬體編碼無 CPU 突發高負載現象，表現穩定 |
| **進程記憶體最高佔用** | **51.59 MB** | **18.46 MB** | 硬體編碼記憶體佔用恆定，不易造成 OOM |

> **效益**：使用 VPU 硬體編碼能將 86% 的進程 CPU 資源釋放出來，提供給機載端的 YOLOv5 AI 物態推論、飛控邏輯與感測器通訊使用。

---

## 🚀 快速上手與操作指南

### 1. 部署與啟動步驟

#### **步驟 A：啟動地面站 (Ground Station)**
在地端 PC（Windows / Linux / macOS）進入 `ground_station` 目錄，安裝依賴並執行：
```bash
pip install -r requirements.txt
python ground_station.py
```
> 地面站會開始監聽 Port 9000 (SRT 影音) 與 Port 9001 (YOLO UDP)，同時在 UDP Port 9002 廣播心跳封包以喚醒無人機端的守護進程。

#### **步驟 B：啟動機載端 (Airborne UAV)**
將本專案的 `work` 目錄複製到開發板的 `/work` 路徑下。登入開發板終端機，執行以下一鍵啟動指令：
```bash
cd /work
./start_stream
```
* **參數選擇**：
  * `./start_stream --mipi` : 強制由 MIPI CSI 相機介面讀取畫面編碼。
  * `./start_stream --usb`  : 強制由 USB 外接相機讀取畫面編碼。
  * `./start_stream --file` : 使用內建的模擬影片檔案進行串流測試。
  * `./start_stream <地面站IP>` : 手動指定地面站 IP，跳過自動搜索。

---

## 🎮 地面站鍵盤控制快捷鍵

地面站畫面視窗啟動後，支援以下即時控制鍵：

| 按鍵 | 功能分類 | 詳細操作說明 |
| :--- | :--- | :--- |
| **`ESC`** | 系統控制 | 立即關閉並退出地面站程式。 |
| **`A` / `a`** | 播放模式 | 切換為 **自動插值 (AUTO)** 模式。 |
| **`D` / `d`** | 播放模式 | 切換為 **直接播放 (DIRECT)** 模式。完全不進行光流插值運算。 |
| **`I` / `i`** | 播放模式 | 切換為 **強制插值 (INTERPOLATION)** 模式。 |
| **`M` / `m`** | 倍率控制 | 切換 **自動自適應倍率 (AUTO Adaptive)** 與 **手動固定倍率 (MANUAL)**。 |
| **`3`** | 倍率控制 | 直接切換為 **手動固定 3x 插值倍率 (MANUAL(3x))**。 *(推薦啟動後立即使用)* |
| **`4`** | 倍率控制 | 直接切換為 **手動固定 4x 插值倍率 (MANUAL(4x))**。 |
| **`5`** | 倍率控制 | 直接切換為 **手動固定 5x 插值倍率 (MANUAL(5x))**。 |
| **`6`** | 倍率控制 | 直接切換為 **手動固定 6x 插值倍率 (MANUAL(6x))**。 |
| **`+` 或 `=`** | 倍率控制 | 增加 1x 手動插值倍率（上限 8x）。 |
| **`-` 或 `_`** | 倍率控制 | 減少 1x 手動插值倍率（下限 1x）。 |
| **`U` / `u`** | 遠端控制 | 遠端控制無人機，切換為 **USB Port 攝影機**。 |
| **`P` / `p`** | 遠端控制 | 遠端控制無人機，切換為 **MIPI Port 攝影機**。 |
| **`F` / `f`** | 遠端控制 | 遠端控制無人機，切換為 **影片模擬串流**。 |

> [!TIP]
> **最佳實踐提示**：
> 為了獲得最平滑且低延遲的 30 FPS 觀看體驗，**建議您在啟動地面站程式後，立刻按下鍵盤上的 `3` 鍵** 切換至手動 3x 插值模式。
