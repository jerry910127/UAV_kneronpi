#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import json

def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            temp_raw = float(f.read().strip())
            return temp_raw / 1000.0 if temp_raw > 1000 else temp_raw
    except Exception:
        return 0.0

def get_system_cpu_and_mem():
    try:
        top_out = subprocess.check_output(["top", "-b", "-n", "1"]).decode("utf-8", errors="ignore")
        cpu_idle = 0.0
        for line in top_out.splitlines():
            if "%Cpu" in line or "CPU:" in line or "Cpu(s):" in line:
                parts = line.split(",")
                for p in parts:
                    if "id" in p:
                        cpu_idle = float(p.strip().split()[0].replace("%", "").replace("id", ""))
                        break
        cpu_usage = max(0.0, 100.0 - cpu_idle)
        return cpu_usage
    except Exception:
        return 0.0

def get_process_stats(proc_name):
    try:
        pids = subprocess.check_output(["pgrep", "-f", proc_name]).decode().strip().split()
        if not pids:
            return 0.0, 0.0
        pid = pids[0]
        ps_out = subprocess.check_output(["ps", "-p", pid, "-o", "%cpu,rss"]).decode().splitlines()
        if len(ps_out) > 1:
            parts = ps_out[1].strip().split()
            pcpu = float(parts[0])
            rss_mb = float(parts[1]) / 1024.0
            return pcpu, rss_mb
    except Exception:
        pass
    return 0.0, 0.0

def profile_target(target_cmd, proc_keyword, duration=15):
    print(f"[Benchmark] Launching: {' '.join(target_cmd)}")
    proc = subprocess.Popen(target_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)  # Wait for startup

    cpu_usages = []
    proc_cpus = []
    proc_mems = []
    temps = []

    start_t = time.time()
    while time.time() - start_t < duration:
        sys_cpu = get_system_cpu_and_mem()
        pcpu, pmem = get_process_stats(proc_keyword)
        temp = get_cpu_temp()

        cpu_usages.append(sys_cpu)
        proc_cpus.append(pcpu)
        proc_mems.append(pmem)
        temps.append(temp)
        time.sleep(1)

    try:
        proc.terminate()
        proc.wait(timeout=2)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass

    subprocess.run(["pkill", "-9", "-f", proc_keyword], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if not cpu_usages:
        return {}

    return {
        "sys_cpu_avg": sum(cpu_usages) / len(cpu_usages),
        "sys_cpu_max": max(cpu_usages),
        "proc_cpu_avg": sum(proc_cpus) / len(proc_cpus),
        "proc_cpu_max": max(proc_cpus),
        "proc_mem_avg": sum(proc_mems) / len(proc_mems),
        "proc_mem_max": max(proc_mems),
        "temp_avg": sum(temps) / len(temps),
        "temp_max": max(temps)
    }

if __name__ == "__main__":
    print("==================================================")
    print("  Kneo Pi Resource Benchmark Execution Utility   ")
    print("==================================================")
    
    test_video = "/tmp/uav_test_720p.h264"
    if not os.path.exists(test_video):
        print(f"[Benchmark Error] {test_video} does not exist.")
        sys.exit(1)

    print("\n---> 1. Profiling FFmpeg CPU Software Encoding Stream...")
    cmd_ffmpeg = [
        "ffmpeg", "-re", "-i", test_video,
        "-c:v", "libx264", "-preset", "ultrafast",
        "-b:v", "140k", "-r", "10",
        "-f", "mpegts", "srt://127.0.0.1:9000?mode=caller"
    ]
    res_ffmpeg = profile_target(cmd_ffmpeg, "ffmpeg", duration=15)

    print("\n---> 2. Profiling Kneo Pi VPU Hardware Encoder Stream (venc1)...")
    cmd_venc = [
        "/work/kneopi-examples/bin/venc1",
        "-i", test_video,
        "-o", "127.0.0.1",
        "-p", "9000",
        "-f", "10",
        "-b", "140000",
        "-g", "10"
    ]
    res_venc = profile_target(cmd_venc, "venc1", duration=15)

    results = {
        "ffmpeg_soft": res_ffmpeg,
        "vpu_hard": res_venc
    }
    print("\n================ BENCHMARK RESULTS ================")
    print(json.dumps(results, indent=2))
    with open("/tmp/benchmark_results.json", "w") as f:
        json.dump(results, f, indent=2)
