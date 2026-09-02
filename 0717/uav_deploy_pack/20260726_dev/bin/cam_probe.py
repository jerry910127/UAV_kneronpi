#!/usr/bin/env python3
"""
USB / V4L2 Camera Auto-Detection & Probing Module for Kneo Pi (KL730)
Author: Antigravity Agent
Description:
    Scans Linux /dev/video* devices and probes for real physical USB UVC / MIPI CSI cameras.
    Distinguishes real video capture sensors (e.g. RealSense RGB YUYV on /dev/video12)
    from VPU encoder/decoder hardware nodes and Z16 Depth sensors.
"""

import sys
import os
import glob
import fcntl
import struct

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
VIDIOC_ENUM_FMT = 0xc0405602

def probe_camera_formats(dev_path):
    formats = []
    try:
        fd = open(dev_path, 'rb')
        fmt_idx = 0
        while True:
            req = struct.pack('IIII32s4I', fmt_idx, V4L2_BUF_TYPE_VIDEO_CAPTURE, 0, 0, b'', 0, 0, 0, 0)
            res = fcntl.ioctl(fd, VIDIOC_ENUM_FMT, req)
            _, _, _, _, description, _, _, _, _ = struct.unpack('IIII32s4I', res)
            desc_str = description.strip(b'\x00').decode('utf-8', errors='ignore')
            formats.append(desc_str)
            fmt_idx += 1
    except Exception:
        pass
    return formats

def scan_v4l2_cameras():
    """
    Scans /sys/class/video4linux/video* and returns a list of dictionaries with camera info:
    [{'device': '/dev/video12', 'name': 'Intel(R) RealSense(TM) Depth Ca', 'is_color': True}, ...]
    """
    cameras = []
    video_nodes = sorted(glob.glob("/sys/class/video4linux/video*"))

    for node in video_nodes:
        dev_name = os.path.basename(node)  # e.g. 'video0'
        dev_path = f"/dev/{dev_name}"
        name_file = os.path.join(node, "name")

        cam_name = "Unknown Camera"
        if os.path.exists(name_file):
            try:
                with open(name_file, "r") as f:
                    cam_name = f.read().strip()
            except Exception:
                pass

        # Filter out VPU / codec / internal driver memory nodes (e.g. vpl_voc2-0-0)
        lower_name = cam_name.lower()
        if "vpl" in lower_name or "vpu" in lower_name or "m2m" in lower_name or "codec" in lower_name:
            continue

        if os.path.exists(dev_path):
            fmts = probe_camera_formats(dev_path)
            fmts_upper = " ".join(fmts).upper()
            is_yuyv = "YUYV" in fmts_upper
            is_color = any(k in fmts_upper for k in ['YUYV', 'UYVY', 'MJPEG', 'NV12', 'YUV', 'RGB'])
            cameras.append({
                'device': dev_path,
                'name': cam_name,
                'node': dev_name,
                'formats': fmts,
                'is_yuyv': is_yuyv,
                'is_color': is_color
            })

    # Sort YUYV cameras first, then other color cameras
    cameras.sort(key=lambda x: (x['is_yuyv'], x['is_color']), reverse=True)
    return cameras

def get_primary_camera():
    """
    Returns the primary USB RGB camera device path (e.g. '/dev/video12') or None if no camera detected.
    """
    cams = scan_v4l2_cameras()
    if cams:
        for c in cams:
            if c['is_color']:
                return c['device']
        return cams[0]['device']
    return None

def print_camera_diagnostics():
    print("==================================================")
    print("   Kneo Pi (KL730) USB Camera Diagnostic Tool    ")
    print("==================================================")
    cams = scan_v4l2_cameras()
    if not cams:
        print("[Cam Probe Warning] No physical USB / V4L2 cameras detected on /dev/video*!")
        print("[Cam Probe Hint] Please check USB cable connection or camera power.")
    else:
        print(f"[Cam Probe Success] Found {len(cams)} physical video camera device(s):")
        for i, c in enumerate(cams):
            tag = " (RGB Color Camera)" if c['is_color'] else ""
            print(f"  [{i+1}] Device: {c['device']} | Name: '{c['name']}'{tag}")
            if c['formats']:
                print(f"      Formats: {', '.join(c['formats'])}")
    print("==================================================")
    primary = get_primary_camera()
    if primary:
        print(f"[Cam Probe Auto-Selected Primary Camera]: {primary}")
    print("==================================================")

if __name__ == "__main__":
    print_camera_diagnostics()
