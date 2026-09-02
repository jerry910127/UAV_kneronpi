#!/usr/bin/env python3
import os, sys, time, struct, select, termios, fcntl

READMEM_CMD  = 0x0800
READMEM_ACK  = 0x0801
WRITEMEM_CMD = 0x0802
WRITEMEM_ACK = 0x0803

def set_baud(fd, baud):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
    attrs[2] = (attrs[2] & ~termios.CSIZE) | termios.CS8
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    attrs[3] = 0  # lflag (raw)

    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if baud not in baud_map:
        raise ValueError("Unsupported baud in this script")

    # KL730 的 Python termios 可能沒有 cfsetispeed/cfsetospeed
    speed = baud_map[baud]
    if hasattr(termios, "cfsetispeed") and hasattr(termios, "cfsetospeed"):
        termios.cfsetispeed(attrs, speed)
        termios.cfsetospeed(attrs, speed)
    else:
        # fallback: attrs[4]=ispeed, attrs[5]=ospeed
        attrs[4] = speed
        attrs[5] = speed

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def set_dtr_rts(fd, dtr=True, rts=True):
    TIOCMGET = getattr(termios, "TIOCMGET", 0x5415)
    TIOCMSET = getattr(termios, "TIOCMSET", 0x5418)
    TIOCM_DTR = getattr(termios, "TIOCM_DTR", 0x002)
    TIOCM_RTS = getattr(termios, "TIOCM_RTS", 0x004)

    buf = struct.pack("I", 0)
    m = struct.unpack("I", fcntl.ioctl(fd, TIOCMGET, buf))[0]
    m = (m | TIOCM_DTR) if dtr else (m & ~TIOCM_DTR)
    m = (m | TIOCM_RTS) if rts else (m & ~TIOCM_RTS)
    fcntl.ioctl(fd, TIOCMSET, struct.pack("I", m))


def read_exact(fd, n, timeout=2.0):
    buf = bytearray()
    end = time.time() + timeout
    while len(buf) < n and time.time() < end:
        r, _, _ = select.select([fd], [], [], max(0.0, end - time.time()))
        if not r:
            continue
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            break
        buf += chunk
    return bytes(buf)

def gen_read(fd, request_id, addr, length):
    # packet layout follows the vendor sample code:
    # flags(2) + cmd_id(2) + scd_len(2) + req_id(2) + addr(8 LE) + reserved(2) + read_len(2)
    flags = 0x4000  # RequestAck
    scd_len = 0x000C
    pkt = struct.pack("<HHHHQHH", flags, READMEM_CMD, scd_len, request_id, addr, 0, length)

    # flush input
    try:
        while True:
            if not select.select([fd], [], [], 0)[0]:
                break
            os.read(fd, 4096)
    except Exception:
        pass

    os.write(fd, pkt)
    ack = read_exact(fd, 8 + length, timeout=2.0)
    if len(ack) < 8:
        raise RuntimeError(f"short ack: {len(ack)} bytes")

    status, cmd_id, ack_len, ack_req = struct.unpack("<HHHH", ack[:8])
    if status != 0:
        print("[RAW]", ack.hex(" "))
        raise RuntimeError(f"status=0x{status:04x}")
    if cmd_id != READMEM_ACK:
        raise RuntimeError(f"unexpected ack cmd_id=0x{cmd_id:04x}")
    if ack_req != request_id:
        raise RuntimeError(f"request_id mismatch {ack_req} != {request_id}")
    return ack[8:]

def gen_write(fd, request_id, addr, data: bytes):
    flags = 0x4000  # RequestAck
    scd_len = 8 + len(data)
    hdr = struct.pack("<HHHHQ", flags, WRITEMEM_CMD, scd_len, request_id, addr)
    pkt = hdr + data

    # flush input
    try:
        while True:
            if not select.select([fd], [], [], 0)[0]:
                break
            os.read(fd, 4096)
    except Exception:
        pass

    os.write(fd, pkt)
    ack = read_exact(fd, 12, timeout=2.0)
    if len(ack) < 8:
        raise RuntimeError(f"short ack: {len(ack)} bytes")

    status, cmd_id, ack_len, ack_req = struct.unpack("<HHHH", ack[:8])
    if status != 0:
        raise RuntimeError(f"status=0x{status:04x}")
    if cmd_id != WRITEMEM_ACK:
        raise RuntimeError(f"unexpected ack cmd_id=0x{cmd_id:04x}")
    if ack_req != request_id:
        raise RuntimeError(f"request_id mismatch {ack_req} != {request_id}")

def try_port(dev, baud):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY)
    try:
        set_baud(fd, baud)
        set_dtr_rts(fd, dtr=True, rts=True)
        # 文件說：讀 0x08 的 64 bytes 會看到 "OBSIDIAN SENSORS INC."
        # 但有些 sample 會從 0x04 開始讀 manufacturer string，所以兩個都試
        rid = 0
        s1 = gen_read(fd, rid, 0x08, 64).split(b"\x00")[0].decode("utf-8", "ignore")
        rid += 1
        s2 = gen_read(fd, rid, 0x04, 64).split(b"\x00")[0].decode("utf-8", "ignore")
        return (s1, s2)
    finally:
        os.close(fd)

def main():
    devs = ["/dev/ttyACM0", "/dev/ttyACM1"]
    bauds = [115200, 9600]

    for dev in devs:
        for baud in bauds:
            if not os.path.exists(dev):
                continue
            try:
                s1, s2 = try_port(dev, baud)
                print(f"[OK] {dev} baud={baud} 0x08='{s1}' 0x04='{s2}'")
            except Exception as e:
                print(f"[FAIL] {dev} baud={baud} err={e}")

if __name__ == "__main__":
    main()
