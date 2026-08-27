#!/usr/bin/env python3
"""
UAV Auto-Discovery & Source Control Daemon for Kneo Pi (KL730)
Author: Antigravity Agent
Description:
    Runs on Kneo Pi (KL730).
    - Adaptive Ground Station IP Discovery:
        * Listens for UDP broadcast heartbeat from Ground Station on Port 9002.
        * Actively sends UDP broadcast probes ("UAV_ANNOUNCE") on Port 9002 to 255.255.255.255.
    - Remote Video Source Control:
        * Receives commands from Ground Station (or command line) to select video input:
          - USB Port: "usb" / "CMD:SET_SOURCE:USB" (probes UVC USB cameras, e.g. /dev/video12)
          - MIPI Port: "mipi" / "CMD:SET_SOURCE:MIPI" (probes CSI cameras, e.g. /dev/video0)
          - Video File Stream: "file" / "CMD:SET_SOURCE:FILE" (uses H.264 video file, e.g. /tmp/uav_test_720p.h264)
        * Dynamically switches input source without rebooting system.
    - Automatically launches SRT video stream (venc1) to discovered Ground Station IP on Port 9000.
"""

import os
import sys
import time
import socket
import signal
import subprocess

UDP_PORT = 9002
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEV_BIN_DIR = os.path.join(SCRIPT_DIR, "bin")
BIN_DIR = DEV_BIN_DIR if os.path.exists(os.path.join(DEV_BIN_DIR, "venc1")) else "/work/kneopi-examples/bin"
TIMEOUT_SEC = 300.0
IP_FILE = "/tmp/ground_station_ip.txt"
PERSISTENT_H264 = os.path.join(SCRIPT_DIR, "test_video", "uav_test_720p.h264")
H264_TEST_FILE = PERSISTENT_H264 if os.path.exists(PERSISTENT_H264) else "/tmp/uav_test_720p.h264"

import cam_probe

class UAVDaemon:
    def __init__(self, mode="auto", video_path=None):
        self.running = True
        self.active_ip = None
        self.last_heartbeat = 0
        self.last_broadcast_probe = 0
        self.proc_venc1 = None
        self.proc_sensor = None
        self.proc_cam_capture = None
        self.stream_mode = mode.lower()  # 'usb', 'mipi', 'file', 'auto'
        self.custom_video_path = video_path
        self.manual_override = False

    def write_ip_file(self, ip):
        try:
            with open(IP_FILE, "w") as f:
                f.write(ip + "\n")
        except Exception as e:
            print(f"[Daemon Error] Could not write {IP_FILE}: {e}")

    def ensure_test_video(self):
        mp4_path = None
        candidate_dirs = [
            os.path.join(SCRIPT_DIR, "test_video"),
            "/work/kneopi-examples/0717/uav_deploy_pack/test_video",
            "/work/kneopi-examples/0717/uav_deploy_pack/20260726_dev/test_video",
            "/work/kneopi-examples/bin/test_video"
        ]

        if self.custom_video_path:
            if os.path.exists(self.custom_video_path):
                mp4_path = self.custom_video_path
            else:
                for cdir in candidate_dirs:
                    target = os.path.join(cdir, os.path.basename(self.custom_video_path))
                    if os.path.exists(target):
                        mp4_path = target
                        break

        if not mp4_path:
            for cdir in candidate_dirs:
                for candidate_file in ["old_town_cross.mp4", "Traffic_and_Building_1080p_30fps_h265.mp4", "模擬空拍機1080p_h264.mp4"]:
                    target = os.path.join(cdir, candidate_file)
                    if os.path.exists(target):
                        mp4_path = target
                        break
                if mp4_path:
                    break

        if mp4_path:
            if not os.path.exists(H264_TEST_FILE) or os.path.getsize(H264_TEST_FILE) < 1000:
                print(f"[Daemon] Generating H.264 test stream from MP4: {os.path.basename(mp4_path)}...")
                cmd = [
                    "ffmpeg", "-y", "-i", mp4_path,
                    "-t", "60", "-vf", "scale=1280:720",
                    "-c:v", "libx264", "-preset", "ultrafast",
                    "-bf", "0", "-g", "10", "-pix_fmt", "yuv420p",
                    H264_TEST_FILE
                ]
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"[Daemon] {H264_TEST_FILE} generated successfully.")

    def is_process_running(self, name):
        try:
            pids = subprocess.check_output(["pgrep", "-f", name]).decode().strip().split()
            for pid in pids:
                try:
                    with open(f"/proc/{pid}/status", "r") as f:
                        status = f.read()
                        if "State:\tZ (zombie)" not in status:
                            return True
                except Exception:
                    pass
            return False
        except subprocess.CalledProcessError:
            return False

    def stop_streaming_services(self):
        print("[Daemon] Stopping current streaming services (SIGTERM)...")
        if self.proc_venc1:
            try:
                self.proc_venc1.terminate()
                self.proc_venc1.wait(timeout=2.0)
            except Exception:
                pass
            self.proc_venc1 = None
        if self.proc_cam_capture:
            try:
                self.proc_cam_capture.terminate()
                self.proc_cam_capture.wait(timeout=1.0)
            except Exception:
                try:
                    self.proc_cam_capture.kill()
                except Exception:
                    pass
            self.proc_cam_capture = None
        subprocess.run(["pkill", "-9", "-f", "ffmpeg.*v4l2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if self.proc_sensor:
            try:
                self.proc_sensor.terminate()
                self.proc_sensor.wait(timeout=2.0)
            except Exception:
                pass
            self.proc_sensor = None
        print("[Daemon] Streaming services stopped cleanly.")

    def start_streaming_services(self, ip, force_restart=False):
        if force_restart:
            self.stop_streaming_services()

        print(f"[Daemon] Launching SRT streaming pipeline for Ground Station IP: {ip} (Source: {self.stream_mode.upper()})")
        self.write_ip_file(ip)
        self.active_ip = ip
        self.last_heartbeat = time.time()

        use_camera = False
        cam_dev = None

        if self.stream_mode in ("usb", "cam"):
            cam_dev = cam_probe.get_usb_camera()
            if cam_dev:
                use_camera = True
                print(f"[Daemon Source Control] USB Camera Selected: {cam_dev}")
            else:
                mipi_dev = cam_probe.get_mipi_camera()
                if mipi_dev:
                    cam_dev = mipi_dev
                    use_camera = True
                    print(f"[Daemon Source Control] No UVC USB camera found. Using MIPI Camera: {cam_dev}")
                else:
                    print(f"[Daemon Warning] Requested USB Camera port, but no USB/MIPI physical camera detected.")
                    print(f"[Daemon Fallback] Falling back to Video File stream mode.")
        elif self.stream_mode == "mipi":
            cam_dev = cam_probe.get_mipi_camera()
            if cam_dev:
                use_camera = True
                print(f"[Daemon Source Control] MIPI Camera Selected: {cam_dev}")
            else:
                usb_dev = cam_probe.get_usb_camera()
                if usb_dev:
                    cam_dev = usb_dev
                    use_camera = True
                    print(f"[Daemon Source Control] No MIPI CSI camera found. Using USB Camera: {cam_dev}")
                else:
                    print(f"[Daemon Warning] Requested MIPI Camera port, but no CSI camera node found.")
                    print(f"[Daemon Fallback] Falling back to Video File stream mode.")
        elif self.stream_mode == "auto":
            cam_dev = cam_probe.get_primary_camera("auto")
            if cam_dev:
                use_camera = True
                print(f"[Daemon Cam Probe] Auto-detected camera port: {cam_dev}")

        # Start venc1 encoder process
        if not self.is_process_running("venc1") or force_restart:
            if use_camera and cam_dev:
                live_h264 = "/tmp/uav_live_cam.h264"
                if os.path.exists(live_h264):
                    try:
                        os.remove(live_h264)
                    except Exception:
                        pass

                cmd_cam = [
                    "ffmpeg", "-y",
                    "-f", "v4l2",
                    "-i", cam_dev,
                    "-vf", "scale=1280:720",
                    "-r", "10",
                    "-c:v", "libx264",
                    "-preset", "ultrafast",
                    "-bf", "0",
                    "-g", "10",
                    "-pix_fmt", "yuv420p",
                    live_h264
                ]
                print(f"[Daemon Live Cam Capture] Capturing V4L2 camera ({cam_dev}) -> {live_h264}")
                self.proc_cam_capture = subprocess.Popen(cmd_cam, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                # Wait up to 5 seconds for V4L2 live camera capture header to initialize
                for _ in range(25):
                    if os.path.exists(live_h264) and os.path.getsize(live_h264) > 1000:
                        break
                    time.sleep(0.2)

                cmd_venc = [
                    os.path.join(BIN_DIR, "venc1"),
                    "-i", live_h264,
                    "-o", ip,
                    "-p", "9000",
                    "-f", "10",
                    "-b", "140000",
                    "-g", "10"
                ]

                print(f"[Daemon] Executing VPU H.265 Hardware Encoder on Live Camera: {' '.join(cmd_venc)}")
                self.proc_venc1 = subprocess.Popen(cmd_venc, cwd=BIN_DIR)
            else:
                self.ensure_test_video()
                cmd_venc = [
                    os.path.join(BIN_DIR, "venc1"),
                    "-i", H264_TEST_FILE,
                    "-o", ip,
                    "-p", "9000",
                    "-f", "10",
                    "-b", "140000",
                    "-g", "10"
                ]
                print(f"[Daemon] Executing H.264 Video File Stream: {' '.join(cmd_venc)}")
                self.proc_venc1 = subprocess.Popen(cmd_venc, cwd=BIN_DIR)


    def switch_source_mode(self, new_mode):
        new_mode = new_mode.lower()
        if new_mode == "cam":
            new_mode = "usb"
        if new_mode in ("usb", "mipi", "file"):
            if self.stream_mode != new_mode:
                print(f"[Daemon Source Switch] Changing video source: {self.stream_mode.upper()} -> {new_mode.upper()}")
                self.stream_mode = new_mode
                if self.active_ip:
                    self.start_streaming_services(self.active_ip, force_restart=True)


    def send_broadcast_beacon(self, sock):
        """Actively broadcasts UAV_ANNOUNCE to subnet 255.255.255.255:9002 for adaptive IP discovery"""
        now = time.time()
        if now - self.last_broadcast_probe >= 2.0:
            self.last_broadcast_probe = now
            msg = f"UAV_ANNOUNCE|SRC:{self.stream_mode.upper()}".encode('utf-8')
            try:
                sock.sendto(msg, ('255.255.255.255', UDP_PORT))
            except Exception:
                pass

    def run(self):
        print("==================================================")
        print("   Kneo Pi (KL730) Adaptive UAV Stream Daemon    ")
        print(f"   Initial Mode: {self.stream_mode.upper()} | Adaptive Search: ACTIVE (Port {UDP_PORT})")
        print("==================================================")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            sock.bind(('0.0.0.0', UDP_PORT))
        except Exception as e:
            print(f"[Daemon Error] Could not bind UDP port {UDP_PORT}: {e}")
            sys.exit(1)

        sock.settimeout(0.5)

        def signal_handler(sig, frame):
            print("\n[Daemon] Exit signal received. Stopping streaming services...")
            self.running = False
            self.stop_streaming_services()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        while self.running:
            # Active Adaptive Beacon Discovery (if no active Ground PC discovered yet)
            if not self.active_ip:
                self.send_broadcast_beacon(sock)

            try:
                data, addr = sock.recvfrom(1024)
                sender_ip = addr[0]
                msg = data.decode('utf-8', errors='ignore').strip()

                if any(k in msg for k in ["GS_DISCOVER", "GS_HEARTBEAT", "GS_ACK", "CMD:SET_SOURCE"]):
                    self.last_heartbeat = time.time()

                    # Send ACK back to sender IP
                    try:
                        sock.sendto(b"UAV_ACK", (sender_ip, UDP_PORT))
                    except Exception:
                        pass

                    # Parse video source control command if present
                    if "CMD:SET_SOURCE:" in msg:
                        # e.g., "CMD:SET_SOURCE:USB" or "CMD:SET_SOURCE:MIPI" or "CMD:SET_SOURCE:FILE"
                        parts = msg.split("CMD:SET_SOURCE:")
                        if len(parts) > 1:
                            target_src = parts[1].split()[0].split("|")[0].lower()
                            self.switch_source_mode(target_src)
                    elif "|SRC:" in msg:
                        # e.g. "GS_HEARTBEAT|SRC:USB"
                        parts = msg.split("|SRC:")
                        if len(parts) > 1:
                            target_src = parts[1].split()[0].lower()
                            self.switch_source_mode(target_src)

                    # Trigger SRT stream launch if new IP or stream down
                    if sender_ip != self.active_ip or not self.is_process_running("venc1"):
                        print(f"[Daemon Adaptive Search] Ground Station Discovered at IP: {sender_ip}")
                        self.start_streaming_services(sender_ip)

            except socket.timeout:
                pass
            except Exception as e:
                print(f"[Daemon Warning] Socket receive error: {e}")

            # Check timeout (stop stream if Ground Station goes offline for > TIMEOUT_SEC)
            if self.active_ip and not self.manual_override and (time.time() - self.last_heartbeat > TIMEOUT_SEC):
                print(f"[Daemon Warning] Ground Station {self.active_ip} timed out (> {TIMEOUT_SEC}s).")
                self.stop_streaming_services()
                self.active_ip = None

if __name__ == "__main__":
    stream_mode = "auto"
    target_ip = None
    custom_video = None

    args = sys.argv[1:]
    i = 0
    while i < len(args):
        arg = args[i]
        if arg in ("--usb", "--cam", "-c"):
            stream_mode = "usb"
        elif arg in ("--mipi", "-m"):
            stream_mode = "mipi"
        elif arg in ("--file", "-f"):
            stream_mode = "file"
        elif arg in ("--video", "-v"):
            if i + 1 < len(args):
                custom_video = args[i+1]
                stream_mode = "file"
                i += 1
        elif arg in ("--scan", "-s"):
            cam_probe.print_camera_diagnostics()
            sys.exit(0)
        elif not arg.startswith("-"):
            target_ip = arg.strip()
        i += 1

    daemon = UAVDaemon(mode=stream_mode, video_path=custom_video)

    if target_ip:
        print(f"[Daemon] Command-line IP override provided: {target_ip} (Mode: {stream_mode.upper()})")
        daemon.manual_override = True
        daemon.start_streaming_services(target_ip)

    daemon.run()
