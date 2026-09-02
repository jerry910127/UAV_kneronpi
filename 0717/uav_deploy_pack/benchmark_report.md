# Kneo Pi 效能基準測試數據報告 (Resource Benchmark Report)

**測試時間**: 2026-08-04  
**測試對象**: 720p H.264 影片串流 (@10fps, 140 kbps Bitrate, SRT 協定)  
**腳本位置**: [benchmark_resource.py](file:///work/kneopi-examples/bin/benchmark_resource.py)  

---

## 1. 測試結果數據總覽

| 監測項目 | FFmpeg 軟體編碼 (`libx264`) | Kneo Pi VPU 硬體編碼 (`venc1`) | 效能差異與效益 |
| :--- | :--- | :--- | :--- |
| **進程 CPU 平均使用率 (proc_cpu_avg)** | **12.23%** | **1.70%** | **CPU 負載降低約 86%** (~7.2倍效能提升) |
| **進程 CPU 最高使用率 (proc_cpu_max)** | **35.40%** | **1.70%** | 硬體編碼無 CPU 突發高負載現象 |
| **系統 CPU 平均使用率 (sys_cpu_avg)** | 17.07% | 16.96% | 系統整體 CPU 保持平穩 |
| **進程記憶體平均佔用 (proc_mem_avg)** | 8.60 MB | 18.46 MB | VPU 預先配置固定 DMA/硬體緩衝區 |
| **進程記憶體最高佔用 (proc_mem_max)** | **51.59 MB** | **18.46 MB** | VPU 記憶體佔用極度穩定 |
| **晶片平均溫度 (temp_avg)** | 39.0 °C | 39.0 °C | 短期 15 秒測試期間溫度維持穩定 |

---

## 2. JSON 原始數據內容

原始 JSON 報告已同步存放在 [benchmark_results.json](file:///work/kneopi-examples/0717/uav_deploy_pack/benchmark_results.json)：

```json
{
  "ffmpeg_soft": {
    "sys_cpu_avg": 17.066666666666666,
    "sys_cpu_max": 32.0,
    "proc_cpu_avg": 12.225,
    "proc_cpu_max": 35.4,
    "proc_mem_avg": 8.598307291666666,
    "proc_mem_max": 51.58984375,
    "temp_avg": 39.0,
    "temp_max": 39.0
  },
  "vpu_hard": {
    "sys_cpu_avg": 16.958333333333332,
    "sys_cpu_max": 29.0,
    "proc_cpu_avg": 1.7,
    "proc_cpu_max": 1.7,
    "proc_mem_avg": 18.45703125,
    "proc_mem_max": 18.45703125,
    "temp_avg": 39.0,
    "temp_max": 39.0
  }
}
```

---

## 3. 結論與建議

1. **資源釋放**：採用 VPU 硬體編碼 (`venc1`) 可大幅節省 **86% 的進程 CPU 運算資源**，使主晶片能將 CPU 核心用於執行 AI 神經網絡推論、感測器數據處理或無人機控制邏輯。
2. **記憶體穩定度**：`venc1` 記憶體佔用恆定為 18.46 MB，相較 FFmpeg 峰值 51.59 MB 更具可預測性與穩定性。
