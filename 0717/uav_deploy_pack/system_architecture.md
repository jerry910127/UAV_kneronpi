# Kneo Pi (KL730) 無人機低頻寬圖傳與 AI 推理系統架構圖與技術說明書

**系統名稱**：UAV Low-Bandwidth SRT Streaming, NPU AI Telemetry & DIS Frame Interpolation System  
**文件版本**：v1.0  
**位置**：`/work/kneopi-examples/0717/uav_deploy_pack/system_architecture.md`  

---

## 📐 一、 端到端整體系統架構圖 (End-to-End Architecture)

本系統採用 **Scheme B 邊緣分工架構**：機載端 (Kneo Pi) 專注於 **硬體視訊編碼壓縮** 與 **NPU 物件偵測**；地面站 (Ground PC) 則透過 **SRT 協議接收低碼率視訊** 並結合 **DIS 光流法進行即時 30 FPS 插幀補償** 與 **HUD 物件邊框繪製**。

```mermaid
flowchart TB
    subgraph AIRBORNE["機載端：Kneo Pi (KL730 UAV)"]
        direction TB
        
        subgraph VPU_PIPE["硬體視訊編解碼管線 (VDEC/VENC)"]
            SRC["視訊源 / 暫存檔<br/>(/tmp/uav_test_720p.h264)"] -->|720p @ 10 FPS| VDEC["VPU 硬體解碼器 (VDEC)<br/>(無 B-Frame 裸流)"]
            VDEC -->|DMA 零拷貝 YUV420| VENC["VPU 硬體 H.265 編碼器 (VENC)<br/>(Bitrate Clamped ≤ 85kbps)"]
            VENC -->|H.265 NAL Units| SRT_SND["非同步 SRT 發送模組 (venc1.c)<br/>(SRTO_SNDTIMEO = 10ms)"]
        end
        
        subgraph NPU_PIPE["NPU AI 推理與遙測管線 (example_nnm_sensor)"]
            NPU["Kneo Pi NPU 加速器<br/>(YOLOv5 物體偵測模組)"] -->|Bounding Box 數據| ANNOT["C++ OpenCV 標註與整理<br/>(display_liveview.cpp)"]
            ANNOT -->|UDP 打包 (24B/Box)| UDP_SND["UDP 遙測發送端<br/>(Port 9001)"]
        end
    end

    AIRBORNE -->|SRT 視訊流 (Port 9000)<br/>H.265 720p @ 10 FPS| NET_SRT["無線傳輸通道<br/>(嚴格 ≤ 300kbps)"]
    AIRBORNE -->|UDP AI 數據 (Port 9001)<br/>YOLOv5 座標與信心度| NET_UDP["無線遙測通道"]

    NET_SRT -->|srt://0.0.0.0:9000| GROUND
    NET_UDP -->|UDP Socket:9001| GROUND

    subgraph GROUND["地面站：Ground Station PC"]
        direction TB
        
        subgraph FFMPEG_DEC["FFmpeg 低延遲接收管線"]
            FFMPEG["FFmpeg 雙輸出接收進程<br/>(-flush_packets 1 -flags low_delay)"] -->|Pipe:1 rawvideo bgr24| FRAME_Q["影格佇列 (Queue maxsize=30)"]
            FFMPEG -->|Output: srt_download_temp.h265| BITRATE_MON["實體頻寬監測器<br/>(Payload + 44B Header)"]
        end
        
        subgraph DIS_INTERP["DIS 光流插幀引擎 (MotionInterpolator)"]
            FRAME_Q -->|10 FPS 原始影格| DIS["OpenCV DIS 光流計算<br/>(FLOW_SCALE = 0.25, 320x180)"]
            DIS -->|補幀計算 (2x/3x)| WARP["cv2.remap 雙向運動扭曲與融合"]
        end
        
        subgraph HUD_DISP["HUD 渲染與顯示器"]
            UDP_RCV["UDP 遙測接收器<br/>(TelemetryReceiver)"] -->|YOLO Boxes| HUD["HUD 圖層疊加器<br/>(FPS / Bitrate / Mode / YOLO BBox)"]
            WARP -->|插幀後 30 FPS 畫面| HUD
            HUD -->|cv2.imshow| DISPLAY["即時顯示視窗 (1280x720 @ 30 FPS)"]
        end
    end
```

---

## ⚙️ 二、 機載端關鍵組件與技術參數 (Airborne Component Specifications)

### 1. 硬體編解碼管線 (`venc1.c`)
- **輸入格式**：H.264 裸流 (`/tmp/uav_test_720p.h264`)，關閉 B-Frame (`-bf 0`)。
- **解析度**：`1280x720` (720p HD)。
- **編碼格式**：H.265 (HEVC Main Profile)。
- **位元率硬性限制 (Bitrate Hard Limit)**：
  - `g_dwBitrate` 被強制鎖定在 **`85,000 bps` (85 kbps)**。
  - 控制台寫入安全攔截警告：`[venc1][PERMANENT_LIMIT]`。
- **硬體 QP 峰值抑制**：
  - `dwMinIQp = 26`（關鍵影格最小量化參數，封死 I-Frame 位元爆發）。
  - `dwMinPQp = 26`（預測影格最小量化參數）。
- **非同步網路超時保護**：
  - `SRTO_SNDTIMEO = 10` (10ms)。遇到網路卡頓或 `SRT_ETIMEOUT` 時**主動拋棄影格**，確保機載端永遠保持 10 FPS 即時發送律動。

### 2. AI 推理與遙測模組 (`example_nnm_sensor` / `display_liveview.cpp`)
- **NPU 模型**：YOLOv5 物態偵測網路。
- **發送協定**：UDP Socket（發送至地端 `192.168.168.17:9001`）。
- **封包結構**：
  ```
  [0..3] Bytes  : Sequence Number (uint32)
  [4..7] Bytes  : Box Count (uint32)
  [8..]  Bytes  : Array of Bounding Boxes (每框 24 Bytes: class_id[u32], score[float], x1[u32], y1[u32], x2[u32], y2[u32])
  ```

---

## 🖥️ 三、 地面站端關鍵組件與技術參數 (Ground Station Specifications)

### 1. FFmpeg 雙輸出管線
- **命令參數**：
  `ffmpeg -fflags nobuffer+discardcorrupt -flags low_delay -i srt://0.0.0.0:9000?mode=listener&transtype=file -map 0:v -c:v copy -flush_packets 1 srt_download_temp.h265 -map 0:v -vf scale=1280:720 -f rawvideo -pix_fmt bgr24 pipe:1`
- **目的**：
  1. `pipe:1` 提供無延遲圖像串流給 Python。
  2. `srt_download_temp.h265` 即時物理寫入，進行純位元率精確測量。

### 2. DIS 光流插幀補償器 (`MotionInterpolator`)
- **演算法**：`cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_ULTRAFAST)`。
- **縮放計算 (FLOW_SCALE)**：`0.25`（將 720p 影像降至 320x180 進行光流計算，運算時間 **< 1.5ms**）。
- **插幀邏輯**：
  - 輸入：10 FPS 影像串流。
  - 補幀：每張真實影格之間插入 2~3 張經由 `cv2.remap` 扭曲融合的虛擬影格。
  - 輸出：**流暢 30 FPS 畫面**。

### 3. 精確實體頻寬監測公式 (Physical Bitrate Calculation)
$$\text{Physical Bitrate (kbps)} = \frac{\left(\Delta \text{Payload Bytes} + \text{Input FPS} \times 44 \text{ Bytes}\right) \times 8}{1000 \times \Delta t}$$
- **44 Bytes 標頭組成**：`IP Header (20B) + UDP Header (8B) + SRT Header (16B)`。

---

## 📊 四、 頻寬與編碼結構對照表 (Bandwidth & Hybrid Coding Budget)

| 項目 | 設定與數值 | 技術效益與備註 |
| :--- | :--- | :--- |
| **H.265 視訊動態碼率 (ABR)** | `40 kbps ~ 220 kbps` | 根據 SRT Buffer/RTT 自動階梯調節 |
| **實體開銷與標頭 (Overhead)** | ~60 ~ 80 kbps | 含 IP/UDP/SRT 標頭 (44B)、AES 加密與 UDP AI 遙測 |
| **最終總實體頻寬** | **動態壓滿 ~280 ~ 300 kbps** | **精確跑滿 300kbps 頻寬上限** |
| **混合編碼結構 (Hybrid Coding)** | Intra-frame (I) + Inter-frame (P) | `GOP = 10` (每秒 1 張 I 幀，9 張 P 幀) |
| **B-Frame 設定** | 0 (已強制關閉 `-bf 0`) | 消除雙向預測延遲，確保即時圖傳 |
| **機載端發送影格率** | 10 FPS | 極低頻寬模式 |
| **地面站顯示影格率** | **30 FPS** | DIS 光流插幀補齊，觀感高度流暢 |

---

## 📄 五、 相關原始碼檔案索引 (Source Code Index)

* **機載端編碼主程式**：[`/work/kneopi-examples/0717/uav_deploy_pack/venc1.c`](file:///work/kneopi-examples/0717/uav_deploy_pack/venc1.c)
* **機載端遙測與標註**：[`/work/kneopi-examples/0717/uav_deploy_pack/display_liveview.cpp`](file:///work/kneopi-examples/0717/uav_deploy_pack/display_liveview.cpp)
* **自動部署與編譯腳本**：[`/work/kneopi-examples/0717/uav_deploy_pack/deploy.sh`](file:///work/kneopi-examples/0717/uav_deploy_pack/deploy.sh)
* **地面站 Python 接收程式**：[`ground_station.py`](file:///work/kneopi-examples/0717/uav_deploy_pack/ground_station.py)
