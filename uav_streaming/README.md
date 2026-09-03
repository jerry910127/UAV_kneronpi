# Kneo-Pi UAV Streaming System (自研無人機串流獨立模組)

本資料夾彙整所有與 UAV 自適應串流、雙鏡頭推論、地面站接收器及部屬工具相關之自研組件：

## 📁 目錄結構說明

- **`uav_system/`**：核心獨立模組（方案 A 模組化架構）
  - `bin/`：已編譯完成之 VPU 硬體編碼器 (`venc1`, `v4l2_h26xenc`)、AI 推論檔與模型 (`nef/`)
  - `daemon/`：`uav_daemon.py`（自適應探測與握手核心）、`cam_probe.py`（相機診斷）
  - `config/`：雙鏡頭相機校正參數 (`camera_params.txt`, `homography_table.txt`, `rgbir.cfg`)
  - `src/`：自研完整原始碼（可於板端重新編譯）
  - `start_stream`：一鍵啟動串流開播腳本
  - `setup_uav_system.sh`：板載模組自初始化腳本
- **`ground_station/`**：地端地面站接收端（SRT/UDP 接收器、低延遲解碼預覽）
- **`modified_rgbir_src/`**：雙鏡頭 RGB-IR 影像推論之客製修改原始碼
- **`top_level_bin/`**：板端頂層編譯輸出之工具與二進位執行檔
- **`top_level_ai_application/`**：板端頂層 AI NNM 應用程式碼
- **`dev_pack_0717/`**：歷史開發套件與效能基準測試記錄
- **`release/`**：打包釋出檔 (`uav_system_v1.0.tar.gz`, `ground_station_v1.0.zip`)
- **`setup_fresh_board.sh`**：新板子自動初始化工具
