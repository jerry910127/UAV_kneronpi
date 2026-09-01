#!/usr/bin/env python3
"""
Ground Station Live Receiver & HUD Interface for Kneo Pi (KL730 UAV)
"""

import sys
import os
import time
import socket
import struct
import threading
import queue
import subprocess
import numpy as np
import cv2

# ==========================================
# Configuration (Matches Kneo Pi venc1.c & uav_daemon.py)
# ==========================================
SRT_PORT = 9000
UDP_TELEMETRY_PORT = 9001
UDP_HEARTBEAT_PORT = 9002
TARGET_FPS = 30.0
WIDTH = 1280
HEIGHT = 720
DISP_WIDTH = 1280
DISP_HEIGHT = 720
LOG_FILE = "fps_performance.log"

def write_log(msg, level="INFO"):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] [{level}] {msg}"
    print(log_entry)
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f_log:
            f_log.write(log_entry + "\n")
    except Exception as e:
        print(f"[Log Write Error] {e}")

class HeartbeatSender(threading.Thread):
    """Broadcasts GS_HEARTBEAT over UDP 9002 every 1 second to wake up Kneo Pi uav_daemon"""
    def __init__(self, port=UDP_HEARTBEAT_PORT, source_mode="auto"):
        super().__init__(daemon=True)
        self.port = port
        self.source_mode = source_mode
        self.running = True

    def send_cmd(self, cmd_str):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            sock.sendto(cmd_str.encode('utf-8'), ('<broadcast>', self.port))
            write_log(f"Sent remote command to Kneo Pi: {cmd_str}", "CMD")
        except Exception as e:
            write_log(f"Failed to send command {cmd_str}: {e}", "ERROR")

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        write_log(f"Broadcasting Auto-Discovery Heartbeat on UDP port {self.port} (Source: {self.source_mode})...", "HEARTBEAT")
        while self.running:
            try:
                msg = f"GS_HEARTBEAT|SRC:{self.source_mode.upper()}".encode('utf-8')
                sock.sendto(msg, ('<broadcast>', self.port))
            except Exception:
                pass
            time.sleep(1.0)

    def stop(self):
        self.running = False


class TelemetryReceiver(threading.Thread):
    """Receives YOLO metadata over UDP port 9001 sent by Kneo Pi"""
    def __init__(self, port=UDP_TELEMETRY_PORT):
        super().__init__(daemon=True)
        self.port = port
        self.boxes = []
        self.seq_num = 0
        self.lock = threading.Lock()
        self.running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', self.port))
            write_log(f"Listening for YOLO metadata on UDP port {self.port}...", "TELEMETRY")
        except Exception as e:
            write_log(f"Could not bind UDP port {self.port}: {e}", "ERROR")
            return

        while self.running:
            try:
                data, _ = sock.recvfrom(2048)
                if len(data) < 8:
                    continue
                seq_num, box_count = struct.unpack('<II', data[:8])
                offset = 8
                new_boxes = []
                for _ in range(min(box_count, 40)):
                    if offset + 24 > len(data):
                        break
                    class_id, score, x1, y1, x2, y2 = struct.unpack('<IfIIII', data[offset:offset+24])
                    new_boxes.append({
                        'class': class_id, 
                        'score': score, 
                        'box': (x1, y1, x2, y2)
                    })
                    offset += 24
                with self.lock:
                    self.boxes = new_boxes
                    self.seq_num = seq_num
            except Exception:
                if not self.running:
                    break

    def get_telemetry(self):
        with self.lock:
            return list(self.boxes), self.seq_num

    def stop(self):
        self.running = False


class MotionInterpolator:
    """Enhanced DIS Optical Flow Interpolator with Motion Vector Smoothing"""
    def __init__(self, scale_factor=0.25):
        self.dis = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_FAST)
        self.dis.setUseMeanNormalization(True)
        self.scale_factor = scale_factor
        self.grid_x = None
        self.grid_y = None
        self.grid_shape = None

    def _init_grid(self, h, w):
        if self.grid_shape != (h, w):
            gx, gy = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
            self.grid_x = gx
            self.grid_y = gy
            self.grid_shape = (h, w)

    def compute_flow(self, prev_gray, next_gray):
        h, w = prev_gray.shape[:2]
        sw, sh = int(w * self.scale_factor), int(h * self.scale_factor)
        small_prev = cv2.resize(prev_gray, (sw, sh), interpolation=cv2.INTER_AREA)
        small_next = cv2.resize(next_gray, (sw, sh), interpolation=cv2.INTER_AREA)
        
        flow_fw_small = self.dis.calc(small_prev, small_next, None)
        flow_bw_small = self.dis.calc(small_next, small_prev, None)
        
        inv_scale = 1.0 / self.scale_factor
        flow_fw = cv2.resize(flow_fw_small, (w, h), interpolation=cv2.INTER_LINEAR) * inv_scale
        flow_bw = cv2.resize(flow_bw_small, (w, h), interpolation=cv2.INTER_LINEAR) * inv_scale
        
        flow_fw = np.clip(flow_fw, -35.0, 35.0)
        flow_bw = np.clip(flow_bw, -35.0, 35.0)
        return flow_fw, flow_bw

    def interpolate_frame(self, img1, img2, flow_fw, flow_bw, alpha):
        if alpha <= 0.0:
            return img1
        if alpha >= 1.0:
            return img2

        h, w = img1.shape[:2]
        self._init_grid(h, w)

        map_fw_x = self.grid_x - alpha * flow_fw[:, :, 0]
        map_fw_y = self.grid_y - alpha * flow_fw[:, :, 1]
        warped1 = cv2.remap(img1, map_fw_x, map_fw_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)

        map_bw_x = self.grid_x - (1.0 - alpha) * flow_bw[:, :, 0]
        map_bw_y = self.grid_y - (1.0 - alpha) * flow_bw[:, :, 1]
        warped2 = cv2.remap(img2, map_bw_x, map_bw_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)

        return cv2.addWeighted(warped1, 1.0 - alpha, warped2, alpha, 0)


def start_ffmpeg_srt_receiver(srt_port=SRT_PORT, out_w=DISP_WIDTH, out_h=DISP_HEIGHT):
    """Launches zero-latency FFmpeg process listening on SRT port 9000 (Stream mode)"""
    srt_url = f"srt://0.0.0.0:{srt_port}?mode=listener&transtype=file&latency=120"
    cmd = [
        "ffmpeg",
        "-y",
        "-loglevel", "quiet",
        "-fflags", "nobuffer+discardcorrupt",
        "-flags", "low_delay",
        "-threads", "1",
        "-avioflags", "direct",
        "-f", "hevc",
        "-i", srt_url,
        "-vf", f"scale={out_w}:{out_h}",
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "pipe:1"
    ]
    write_log(f"Launching zero-latency FFmpeg receiver listening on {srt_url}...", "SRT")
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        return proc
    except Exception as e:
        write_log(f"Failed to launch FFmpeg: {e}", "ERROR")
        return None


def main():
    source_mode = "auto"
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        arg = args[i]
        if arg in ("--source", "-s"):
            if i + 1 < len(args):
                source_mode = args[i+1].lower()
                i += 1
        elif arg in ("--usb", "-u"):
            source_mode = "usb"
        elif arg in ("--mipi", "-m"):
            source_mode = "mipi"
        elif arg in ("--file", "-f"):
            source_mode = "file"
        i += 1

    print("==================================================")
    print("   Ground Station Live Receiver & Telemetry HUD   ")
    print(f"   Auto-Discovery Heartbeat (Initial Source: {source_mode.upper()})")
    print("==================================================")

    # 1. Start Auto-Discovery Heartbeat Broadcast Thread
    heartbeat = HeartbeatSender(source_mode=source_mode)
    heartbeat.start()

    # 2. Start Telemetry Receiver Thread
    telemetry = TelemetryReceiver()
    telemetry.start()

    # 3. Multi-threaded Architecture Queues & FFmpeg SRT Receiver
    raw_queue = queue.Queue(maxsize=3)
    display_queue = queue.Queue(maxsize=15)
    proc = start_ffmpeg_srt_receiver()

    running_reader = True

    def reader_thread():
        nonlocal proc
        expected_size = DISP_WIDTH * DISP_HEIGHT * 3
        frame_bytes = bytearray()
        while running_reader:
            if proc is None or proc.poll() is not None:
                write_log("FFmpeg SRT receiver listener process exited/disconnected. Re-opening listener on Port 9000...", "SRT")
                if proc:
                    try:
                        proc.kill()
                    except Exception:
                        pass
                proc = start_ffmpeg_srt_receiver()
                time.sleep(0.5)
                continue

            try:
                chunk = proc.stdout.read(expected_size - len(frame_bytes))
                if not chunk:
                    time.sleep(0.01)
                    continue

                frame_bytes.extend(chunk)
                if len(frame_bytes) == expected_size:
                    img = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((DISP_HEIGHT, DISP_WIDTH, 3))
                    frame_bytes = bytearray()
                    
                    try:
                        raw_queue.put_nowait(img)
                    except queue.Full:
                        try:
                            raw_queue.get_nowait()
                            raw_queue.put_nowait(img)
                        except queue.Empty:
                            pass
            except Exception as e:
                write_log(f"Reader thread warning: {e}", "WARN")
                time.sleep(0.1)

    t_reader = threading.Thread(target=reader_thread, daemon=True)
    t_reader.start()

    interpolator = MotionInterpolator(scale_factor=0.25)
    
    user_mode = "AUTO"  # Modes: "AUTO", "DIRECT", "INTERPOLATION"
    auto_adaptive_mode = True
    manual_factor = 3
    input_fps_measure = 10.0
    input_count_interp = 0
    start_time_interp = time.time()
    running_interp = True

    def push_display(frame_tuple):
        try:
            display_queue.put_nowait(frame_tuple)
        except queue.Full:
            try:
                display_queue.get_nowait()
                display_queue.put_nowait(frame_tuple)
            except queue.Empty:
                pass

    def interpolation_worker():
        nonlocal input_fps_measure, input_count_interp, start_time_interp
        prev_f = None
        prev_g = None

        while running_interp:
            try:
                curr_f = raw_queue.get(timeout=0.2)
                input_count_interp += 1

                while raw_queue.qsize() > 0:
                    try:
                        curr_f = raw_queue.get_nowait()
                        input_count_interp += 1
                    except queue.Empty:
                        break

                now = time.time()
                elapsed = now - start_time_interp
                if elapsed >= 1.0:
                    input_fps_measure = input_count_interp / elapsed
                    input_count_interp = 0
                    start_time_interp = now

                curr_g = cv2.cvtColor(curr_f, cv2.COLOR_BGR2GRAY)
                if prev_f is None:
                    prev_f = curr_f.copy()
                    prev_g = curr_g.copy()

                active = user_mode
                if user_mode == "AUTO":
                    active = "INTERPOLATION" if (input_fps_measure > 0 and input_fps_measure <= 18.0) else "DIRECT"

                if active == "DIRECT":
                    push_display((curr_f, "DIRECT", 1, input_fps_measure))
                else:
                    if auto_adaptive_mode:
                        K = max(1, min(6, int(round(TARGET_FPS / max(1.0, input_fps_measure))))) if input_fps_measure > 0 else 3
                        mode_str = f"AUTO({K}x)"
                    else:
                        K = max(1, min(8, manual_factor))
                        mode_str = f"MANUAL({K}x)"

                    try:
                        flow_fw, flow_bw = interpolator.compute_flow(prev_g, curr_g)
                        for i in range(K):
                            alpha = float(i) / float(K)
                            interp_f = interpolator.interpolate_frame(prev_f, curr_f, flow_fw, flow_bw, alpha)
                            push_display((interp_f, mode_str, K, input_fps_measure))
                    except Exception as err:
                        write_log(f"Optical flow compute err: {err}", "ERROR")
                        push_display((curr_f, "DIRECT", 1, input_fps_measure))

                prev_f = curr_f.copy()
                prev_g = curr_g.copy()

            except queue.Empty:
                pass

    t_interp = threading.Thread(target=interpolation_worker, daemon=True)
    t_interp.start()

    write_log("Waiting for SRT video stream connection from Kneo Pi UAV...", "MAIN")

    window_name = "Kneo Pi Ground Station (Native 30 FPS Live)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, DISP_WIDTH, DISP_HEIGHT)

    fps_count = 0
    start_time = time.time()
    display_fps = 0.0
    input_fps = 0.0
    current_kbps = 0.0
    media_kbps = 0.0
    overhead_kbps = 0.0

    last_displayed_frame = None
    last_mode_tag = "WAITING"

    def handle_key(key):
        nonlocal user_mode, auto_adaptive_mode, manual_factor
        if key == 27:  # ESC
            raise KeyboardInterrupt
        elif key in (ord('a'), ord('A')):
            user_mode = "AUTO"
            write_log("User set mode to AUTO", "MODE")
        elif key in (ord('d'), ord('D')):
            user_mode = "DIRECT"
            write_log("User set mode to DIRECT", "MODE")
        elif key in (ord('i'), ord('I')):
            user_mode = "INTERPOLATION"
            write_log("User set mode to INTERPOLATION", "MODE")
        elif key in (ord('m'), ord('M')):
            auto_adaptive_mode = not auto_adaptive_mode
            write_log(f"Switched to {'AUTO Adaptive' if auto_adaptive_mode else 'MANUAL'} factor mode.", "MODE")
        elif key == ord('3'):
            manual_factor = 3
            auto_adaptive_mode = False
            user_mode = "INTERPOLATION"
            write_log("User manually set factor to 3x", "FACTOR")
        elif key == ord('4'):
            manual_factor = 4
            auto_adaptive_mode = False
            user_mode = "INTERPOLATION"
            write_log("User manually set factor to 4x", "FACTOR")
        elif key == ord('5'):
            manual_factor = 5
            auto_adaptive_mode = False
            user_mode = "INTERPOLATION"
            write_log("User manually set factor to 5x", "FACTOR")
        elif key == ord('6'):
            manual_factor = 6
            auto_adaptive_mode = False
            user_mode = "INTERPOLATION"
            write_log("User manually set factor to 6x", "FACTOR")
        elif key in (ord('u'), ord('U')):
            heartbeat.source_mode = "usb"
            heartbeat.send_cmd("CMD:SET_SOURCE:USB")
            write_log("Sent remote request to Kneo Pi: Switch to USB Camera", "SOURCE_CMD")
        elif key in (ord('p'), ord('P')):
            heartbeat.source_mode = "mipi"
            heartbeat.send_cmd("CMD:SET_SOURCE:MIPI")
            write_log("Sent remote request to Kneo Pi: Switch to MIPI Camera", "SOURCE_CMD")
        elif key in (ord('f'), ord('F')):
            heartbeat.source_mode = "file"
            heartbeat.send_cmd("CMD:SET_SOURCE:FILE")
            write_log("Sent remote request to Kneo Pi: Switch to Video File stream", "SOURCE_CMD")
        elif key in (ord('+'), ord('=')):
            manual_factor = min(8, manual_factor + 1)
            auto_adaptive_mode = False
            write_log(f"Manual factor increased to {manual_factor}x", "FACTOR")
        elif key in (ord('-'), ord('_')):
            manual_factor = max(1, manual_factor - 1)
            auto_adaptive_mode = False
            write_log(f"Manual factor decreased to {manual_factor}x", "FACTOR")

    try:
        target_frame_time = 1.0 / TARGET_FPS  # ~33.33ms

        while True:
            t_frame_start = time.perf_counter()

            try:
                frame_data = display_queue.get_nowait()
                last_displayed_frame, last_mode_tag, _, input_fps = frame_data
            except queue.Empty:
                pass

            boxes, seq_num = telemetry.get_telemetry()
            scale_x = DISP_WIDTH / float(WIDTH)
            scale_y = DISP_HEIGHT / float(HEIGHT)

            if last_displayed_frame is not None:
                out_img = last_displayed_frame.copy()
                for b in boxes:
                    x1, y1, x2, y2 = b['box']
                    sx1, sy1 = int(x1 * scale_x), int(y1 * scale_y)
                    sx2, sy2 = int(x2 * scale_x), int(y2 * scale_y)
                    score = b['score']
                    cv2.rectangle(out_img, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
                    cv2.putText(out_img, f"YOLO {score:.2f}", (sx1, max(sy1 - 5, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                cv2.putText(out_img, f"In: {input_fps:.1f} FPS | Out: {display_fps:.1f} FPS | Mode: {last_mode_tag}",
                            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(out_img, f"Bitrate: {current_kbps:.1f} kbps [Media: {media_kbps:.1f}k + Protocol/Crypto: {overhead_kbps:.1f}k]",
                            (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                cv2.putText(out_img, f"YOLO: {len(boxes)} | Press 'm' (Toggle AUTO/MANUAL), '+' / '-' (Factor), '3','4','5','6'",
                            (15, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                cv2.imshow(window_name, out_img)
                fps_count += 1
            else:
                placeholder = np.zeros((DISP_HEIGHT, DISP_WIDTH, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for SRT video stream connection from Kneo Pi UAV...", 
                            (100, DISP_HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow(window_name, placeholder)

            # Continuous non-blocking key processing (< 1ms responsiveness)
            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                handle_key(key)

            elapsed_total = time.time() - start_time
            if elapsed_total >= 1.0:
                display_fps = fps_count / elapsed_total
                media_kbps = max(0.0, input_fps * 13.5)
                overhead_kbps = (max(1.0, input_fps) * 1.5 * 44 * 8 / 1000.0) + 15.0
                current_kbps = media_kbps + overhead_kbps

                write_log(f"IN_FPS: {input_fps:.2f} | OUT_FPS: {display_fps:.2f} | BITRATE: {current_kbps:.1f} kbps | MODE: {last_mode_tag} | YOLO: {len(boxes)}", "STAT")
                fps_count = 0
                start_time = time.time()

            t_elapsed = time.perf_counter() - t_frame_start
            remain = target_frame_time - t_elapsed
            if remain > 0.001:
                time.sleep(remain)

    except KeyboardInterrupt:
        write_log("Shutting down Ground Station receiver...", "INFO")
    finally:
        running_interp = False
        heartbeat.stop()
        telemetry.stop()
        if proc:
            proc.kill()
        cv2.destroyAllWindows()
        write_log("Clean exit completed.", "INFO")

if __name__ == "__main__":
    main()
