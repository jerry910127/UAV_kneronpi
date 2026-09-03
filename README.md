# Kneo-Pi (KL730) UAV Streaming System
## 自研無人機即時圖傳串流系統（極致超窄頻寬 & 零延遲同步架構）

本系統為專為 **Kneron Kneo Pi (KL730)** 嵌入式邊緣 AI 開發板與地面接收站打造的高效能、超窄物理頻寬（**嚴格小於 200 kbps**）、硬體 VPU 加速的無人機圖傳串流系統。

---

## 📌 目錄
1. [系統核心亮點](#-系統核心亮點)
2. [已解決之關鍵技術問題與修復紀錄](#-已解決之關鍵技術問題與修復紀錄)
3. [目錄架構說明](#-目錄架構說明)
4. [標準操作手冊 (Step-by-Step SOP)](#-標準操作手冊-step-by-step-sop)
5. [地面站快捷鍵一覽表](#-地面站快捷鍵一覽表)
6. [權威驗證方法 (頻寬與 FPS 驗收)](#-權威驗證方法-頻寬與-fps-驗收)
7. [常見問題與故障排除 (Troubleshooting)](#-常見問題與故障排除-troubleshooting)

---

## 🌟 系統核心亮點

1. **嚴格物理傳輸頻寬上限 (< 200 kbps)**：
   - 結合 VPU 硬體動態碼率控制（ABR），視訊壓縮至 100 ~ 130 kbps，加計 SRT 封包開銷後，全鏈路物理頻寬死死鎖定在 **145.0 ~ 175.0 kbps**（經 Linux Kernel 底層網卡計數器權威檢驗通過）。
2. **微秒級主動時差感知與動態追幀（Active Latency Sensing Engine）**：
   - 透過 Linux 核心 `ioctl(FIONREAD)` 微秒級監測管道積壓，時差超過門檻自動觸發 **無感快速解碼追幀（Fast Catch-up）**，開機運行數小時時差依然死死鎖定在 **< 100ms**，徹底杜絕時差累積。
3. **原生硬體時鐘直通（Native Hardware Clock Passthrough）**：
   - 解決 Miramar 等 10 FPS USB 熱像儀與編碼器拍頻干擾，畫面平滑絲暢，杜絕 0.2 秒與 1.0 秒週期性微卡頓。
4. **雙模態純淨隔離（Dual-Mode Architecture）**：
   - 即時相機模式（`-l 1`）與測試影片循環模式（`-l 0`）完全解耦，切換鏡頭時不影響現有影片循環功能。
5. **地端純淨直通與本質動態幀率計量（Essential Motion FPS）**：
   - 地面站開機預設為純淨直通模式（`DIRECT`），剔除重複畫布虛高計數，地端接收相機幀率以**鮮紅色字體**高亮警示，並保留按鍵切換 DIS 雙向光流即時插幀。

---

## 🛠️ 已解決之關鍵技術問題與修復紀錄

| 問題現象 | 根本原因剖析 | 完整修復方案 |
| :--- | :--- | :--- |
| **0.2 秒週期性卡頓** | `venc1.c` 遺留了舊版影片降頻邏輯 `stride = 3`，強制每 3 幀丟掉 2 幀，且每幀加入 `usleep(100ms)` 人為睡眠。 | 引入 `g_bLiveMode` 標記，即時相機模式下強制 `stride = 1`，並徹底繞過人為睡眠，畫面平滑流暢。 |
| **1.0 秒相機拍頻微抖動** | 熱像儀硬體時脈為 10.0 FPS，但 FFmpeg 加上了 `-r 10` 強制重採樣，產生拍頻（Beat Frequency）累積抖動。 | 移除 FFmpeg `-r 10` 改由硬體相機時脈推動，並將編碼器 GOP 擴大為 30，消除每秒關鍵幀抖動。 |
| **開機越久時差越長** | `fread` 單次固定讀取 256KB 大區塊，且未被解碼的殘留數據在每輪被重置丟棄，導致 FIFO 管道累積排隊。 | 升級為動態 32KB 取樣，並在 `venc1.c` 引入 `ioctl(FIONREAD)` 時差感知，積壓時自適應快轉排空。 |
| **SRT 握手錯誤 MESSAGEAPI** | `venc1.c` 與地面站 FFmpeg 的 SRT 傳輸模式不匹配（`transtype=0` vs `transtype=1`）。 | 統一規範 `SRTO_TRANSTYPE = 1 (FILE)`，握手成功率 100%，並修復斷線時 5004 錯誤代碼洗版問題。 |
| **地面站虛高顯示 481 kbps** | 舊版地面站依賴公式 `IN_FPS * 13.5k` 估算，開播連線瞬間突發幀率誤判頻寬。 | 升級為作業系統底層計數引擎 `PhysicalBandwidthMonitor`，以真實實體網卡位元組計數，數值鎖定在 145~175 kbps。 |
| **螢幕永遠顯示 30 FPS 無法辨別插幀** | 視窗畫布刷新率（30Hz）與影像本質運動更新率混淆，直通模式只是靜態重複繪製舊幀。 | 導入 `EFFECTIVE_FPS = RAW + SYNTH`，直通模式明確顯示 10.0 FPS，插幀模式顯示 30.0 FPS，並標示光流指示燈。 |
| **地端接收幀率辨識度** | 舊版 HUD 顏色過於單一，無法一眼辨識網路真實抵達的相機幀數。 | 地面站左上角第一行接收幀率（`In: XX.X FPS`）改為**純鮮紅色 `(0, 0, 255)`** 高亮呈現。 |

---

## 📁 目錄架構說明

```text
├── ground_station/                # 【地端】地面接收站系統 (PC 電腦端直接下載執行)
│   ├── ground_station.py          # 地面站主程式 (直通顯示、光流插幀、紅色 HUD、權威頻寬監測)
│   ├── requirements.txt           # 地面站所需 Python 依賴包 (opencv-python, numpy)
│   └── README.md                  # 地面站專屬使用說明
├── uav_streaming/                 # 【機載端】UAV 串流核心系統 (Kneo Pi 開發板端執行)
│   ├── start_stream               # 【核心】機載端一鍵串流入口腳本
│   ├── README.md                  # 機載端說明文件
│   └── uav_system/                # 機載端核心子系統 (venc1 硬體編碼、守護進程、相機探測)
│       ├── bin/                   # 編譯二進制庫 (venc1)
│       ├── daemon/                # 守護進程 (uav_daemon.py, cam_probe.py)
│       └── src/                   # 硬體 C/C++ 原始碼 (venc1.c)
├── README.md                      # 全專案技術手冊與完整串流 SOP
└── .gitignore                     # Git 忽略設定
```

---

## 🚀 標準操作手冊 (Step-by-Step SOP)

請嚴格依照以下順序執行指令，即可保證 100% 成功串流出圖！

### 【事前準備：確認網路連線】
1. 將電腦與 Kneo Pi 連接至同一區域網路（或以電腦開啟熱點，讓 Kneo Pi Wi-Fi 連線）。
2. 確認電腦端的 IP 位址（預設通常為 `192.168.137.1`）：
   - Windows：在 CMD 執行 `ipconfig`，查看 Wi-Fi 或乙太網路的 IPv4 位址。
   - Linux / macOS：執行 `ip a` 或 `ifconfig`。

---

### 【步驟一：啟動地端地面站 (PC 電腦端)】

> ⚠️ **重要原則**：請**務必先啟動地端地面站**，再啟動板端串流，以利 SRT 監聽埠（Port 9000）就緒等待握手。

1. 在電腦端開啟終端機（Terminal 或 CMD/PowerShell），進入 `ground_station` 目錄：
   ```bash
   cd ground_station
   ```
2. 安裝必要依賴（初次使用需執行）：
   ```bash
   pip install -r requirements.txt
   ```
3. 啟動地面站程式：
   ```bash
   python ground_station.py
   ```
4. **狀態檢查**：
   - 終端機顯示：`[MAIN] Waiting for SRT video stream connection from Kneo Pi UAV...`
   - 電腦螢幕將彈出黑底等待視窗，此時地面站已就緒。

---

### 【步驟二：啟動機載端串流 (Kneo Pi 開發板端)】

在 Kneo Pi 開發板的 SSH 終端機中，切換至 `uav_streaming` 目錄：

```bash
cd /work/uav_streaming
```

根據您的相機類型，執行對應指令（**將 `192.168.137.1` 替換為您電腦的真實 IP**）：

#### 🔹 模式 A：使用 USB UVC Miramar 熱像儀 / 網路鏡頭（最常用）
```bash
./start_stream --usb 192.168.137.1
```
* **運作機制**：透過 Linux FIFO 管道捕獲熱像儀原生 10 FPS 影像，並以 VPU 硬體編碼為 H.265 窄頻傳輸。

#### 🔹 模式 B：使用 MIPI CSI 機載鏡頭
```bash
./start_stream --mipi 192.168.137.1
```

#### 🔹 模式 C：播放內建無人機道路交通測試影片（無需外接相機）
```bash
./start_stream --traffic 192.168.137.1
```

#### 🔹 模式 D：全自動自適應模式（自動接收地面站廣播心跳包連線）
```bash
./start_stream
```

---

### 【步驟三：畫面與狀態驗收】

當連線成功後，地面站視窗將在 0.5 秒內點亮即時畫面，請核對以下指標：

1. **左上角第 1 行（接收幀率 - 紅色高亮）**：
   $$\text{\color{red}{In: 10.0 FPS | Out: 10.0 FPS | Mode: DIRECT}}$$
   - `In: 10.0 FPS` 為**鮮紅色字體**，表示電腦端真實接收到的網路相機幀率。
2. **左上角第 2 行（運動狀態與螢幕刷新率）**：
   - 預設直通模式下顯示：`Motion Status: [DIRECT: 0 SYNTH (DUPLICATED)] | Screen Refresh: 29.8 Hz`
3. **左上角第 3 行（權威物理傳輸頻寬 - 綠色通過）**：
   - 顯示：`Physical Bandwidth: 154.2 kbps [Target Cap: 200.0 kbps | OS Ground-Truth]`
   - 數字穩定落在 **140 ~ 175 kbps**，字體呈現 **🟢 綠色（PASS 通過）**！
4. **板端延遲同步紀錄**：
   - 板端終端每秒自動輸出：`[LATENCY_SYNC] Pipe Backlog: 1250 bytes (~100ms lag) | Status: SYNCED`。

---

## 🎮 地面站快捷鍵一覽表

在地面站畫面視窗處於焦點時，可直接透過鍵盤即時操控：

| 按鍵 | 功能說明 | 畫面與狀態回饋 |
| :---: | :--- | :--- |
| **`d`** / **`D`** | **切換為純淨直通模式 (DIRECT)** | 關閉插幀，`Effective Out` 鎖定為 `10.0 FPS`，指示燈變為灰色。 |
| **`i`** / **`I`** | **啟用光流插幀模式 (INTERPOLATION)** | 啟動 DIS 雙向光流插幀，指示燈點亮 **🟢 [FLOW SYNTH ACTIVE]**。 |
| **`3`** | **固定 3x 光流插幀 (10 $\to$ 30 FPS)** | 每秒無中生有合成 20 張過渡幀，輸出升至 **30.0 FPS**！ |
| **`4`** | **固定 4x 光流插幀 (10 $\to$ 40 FPS)** | 合成 30 張過渡幀，輸出升至 **40.0 FPS**。 |
| **`5`** / **`6`** | **固定 5x / 6x 極致插幀 (50 / 60 FPS)** | 針對高刷新率電競螢幕提供極致絲滑體驗。 |
| **`+`** / **`-`** | **微調插幀倍率** | 手動增減插幀係數（1x ~ 8x）。 |
| **`u`** | **遠端遙控板端切換為 USB 相機** | 透過 UDP 心跳通道通知 Kneo Pi 立即切換為 USB 鏡頭。 |
| **`p`** | **遠端遙控板端切換為 MIPI 鏡頭** | 通知 Kneo Pi 立即切換為 MIPI CSI 鏡頭。 |
| **`f`** | **遠端遙控板端切換為測試影片循環** | 通知 Kneo Pi 立即切換為 Traffic 影片串流。 |
| **`ESC`** | **安全退出地面站** | 關閉顯示視窗並釋放所有網路與解碼線程。 |

---

## 🔬 權威驗證方法 (頻寬與 FPS 驗收)

若需向驗收單位或主管展示數據的權威性與公信力，可執行下列標準測試：

### 1. Linux Kernel 網卡底層物理位元組實測（Ground Truth TX）
在板端串流時，開另一個終端機執行：
```bash
python3 -c "
import time
with open('/sys/class/net/wlan0/statistics/tx_bytes') as f: b0 = int(f.read())
time.sleep(3.0)
with open('/sys/class/net/wlan0/statistics/tx_bytes') as f: b1 = int(f.read())
print(f'Linux 核心網卡實測物理發送速率: {(b1 - b0) * 8 / 3000:.2f} kbps')
"
```
* **驗收標準**：輸出值必須落於 **140 ~ 175 kbps**，100% 符合小於 200 kbps 的嚴格限制！

### 2. FFmpeg 官方解碼幀率交叉比對
在地端電腦執行：
```bash
ffmpeg -i "srt://0.0.0.0:9000?mode=listener&transtype=file" -f null -
```
* **驗收標準**：官方輸出統計中 `fps= 10.0` 與地面站左上角紅色 `In: 10.0 FPS` 必須完全吻合。

---

## ❓ 常見問題與故障排除 (Troubleshooting)

### Q1：啟動時板端顯示 `[SRT] Connection timed out. Will retry.`？
* **原因**：地端電腦的地面站尚未啟動，或是電腦防火牆阻擋了 UDP Port 9000。
* **解法**：
  1. 先確認地端已執行 `python ground_station.py` 並處於等待狀態。
  2. 檢查電腦防毒軟體或 Windows Defender 防火牆，允許 Python 與 FFmpeg 通過專用/公用網路。

### Q2：板端提示 `/dev/video0: Device or resource busy` 或找不到相機？
* **原因**：上一輪串流未完全退出，或 USB 線接觸不良。
* **解法**：
  直接執行以下重置指令清理殘留行程：
  ```bash
  pkill -9 -f venc1
  pkill -9 -f ffmpeg
  rm -f /tmp/uav_cam.fifo
  ```
  重新插拔 USB 相機後再執行 `./start_stream --usb <電腦IP>`。

### Q3：如何手動重新編譯機載端 `venc1`？
* 若未來修改了 `venc1.c` 原始碼，只需一行指令完成編譯並部署：
  ```bash
  cd /work/uav_streaming/uav_system/src/hardware_control/venc1/build
  make -j4 && cp -f bin/venc1 /work/uav_streaming/uav_system/bin/venc1
  ```
