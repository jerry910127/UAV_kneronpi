#!/usr/bin/env python3
import os
import sys
import time
import termios
import fcntl
import struct

# Probe based on vendor Python sample (GenicamPythonSampleCode.txt):
# - NO GenCP serial prefix/CRC
# - Little-endian fields
# - Vendor sample uses 115200 8N1
#
# It tries ReadMem(0x08, 64) and ReadMem(0x04, 64) and prints the decoded ASCII.

READMEM_CMD = 0x0800
READMEM_ACK = 0x0801

# ioctl constants (Linux)
TIOCMGET = getattr(termios, "TIOCMGET", 0x5415)
TIOCMSET = getattr(termios, "TIOCMSET", 0x5418)
TIOCM_DTR = getattr(termios, "TIOCM_DTR", 0x002)
TIOCM_RTS = getattr(termios, "TIOCM_RTS", 0x004)


def set_dtr_rts(fd: int, dtr: bool = True, rts: bool = True) -> None:
    """Some CDC devices only respond once DTR/RTS is asserted."""
    try:
        bits = struct.unpack("I", fcntl.ioctl(fd, TIOCMGET, struct.pack("I", 0)))[0]
        if dtr:
            bits |= TIOCM_DTR
        else:
            bits &= ~TIOCM_DTR
        if rts:
            bits |= TIOCM_RTS
        else:
            bits &= ~TIOCM_RTS
        fcntl.ioctl(fd, TIOCMSET, struct.pack("I", bits))
    except Exception:
        # Not fatal; some platforms/drivers may not expose these ioctls.
        pass


def setup_tty_115200_8n1(fd: int):
    attrs = termios.tcgetattr(fd)

    # raw mode-ish
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD  # cflag
    attrs[3] = 0  # lflag

    # 8N1
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSTOPB
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8

    # baud 115200 (matches vendor sample)
    attrs[4] = termios.B115200  # ispeed
    attrs[5] = termios.B115200  # ospeed

    # non-blocking reads with VTIME/VMIN
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1  # 0.1s

    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def build_readmem_cmd_le(addr: int, length: int, request_id: int) -> bytes:
    """Build 20-byte ReadMem command exactly like vendor sample."""
    cmd = bytearray(20)

    # CCD
    cmd[0] = 0x00
    cmd[1] = 0x40  # flags: bit14 RequestAck

    # command_id = 0x0800, little-endian -> 00 08
    cmd[2] = 0x00
    cmd[3] = 0x08

    # SCD length = 12 bytes, little-endian
    cmd[4] = 0x0C
    cmd[5] = 0x00

    # request_id, little-endian
    cmd[6] = request_id & 0xFF
    cmd[7] = (request_id >> 8) & 0xFF

    # SCD
    cmd[8:16] = int(addr).to_bytes(8, "little", signed=False)
    cmd[16] = 0
    cmd[17] = 0

    # read length, little-endian
    cmd[18] = length & 0xFF
    cmd[19] = (length >> 8) & 0xFF

    return bytes(cmd)


def hexdump(b: bytes, maxlen: int = 64) -> str:
    b = b[:maxlen]
    return " ".join(f"{x:02x}" for x in b)


def read_exact(fd: int, n: int, timeout_sec: float = 2.0) -> bytes:
    buf = bytearray()
    t0 = time.time()
    while len(buf) < n and (time.time() - t0) < timeout_sec:
        try:
            chunk = os.read(fd, n - len(buf))
            if chunk:
                buf += chunk
            else:
                time.sleep(0.01)
        except BlockingIOError:
            time.sleep(0.01)
    return bytes(buf)


def do_readmem(fd: int, addr: int, length: int, request_id: int, timeout_sec: float = 2.5):
    # clear input buffer
    try:
        os.read(fd, 4096)
    except Exception:
        pass

    cmd = build_readmem_cmd_le(addr, length, request_id)
    os.write(fd, cmd)

    rx = read_exact(fd, 8 + length, timeout_sec=timeout_sec)
    if len(rx) < 8:
        return None, rx

    status = int.from_bytes(rx[0:2], "little")
    cmd_id = int.from_bytes(rx[2:4], "little")
    # len_field = int.from_bytes(rx[4:6], "little")  # not needed for fixed read
    # req_id = int.from_bytes(rx[6:8], "little")

    if status != 0:
        return (False, f"status=0x{status:04X} cmd=0x{cmd_id:04X}"), rx

    if cmd_id != READMEM_ACK:
        return (False, f"not READMEM_ACK (cmd=0x{cmd_id:04X})"), rx

    if len(rx) < 8 + length:
        return (False, f"short read: got {len(rx)} bytes, need {8+length}"), rx

    payload = rx[8:8 + length]
    return (True, payload), rx


def decode_ascii(payload: bytes) -> str:
    s = payload.split(b"\x00", 1)[0]
    return s.decode("ascii", errors="replace")


def probe_port(port: str):
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        print(f"[{port}] open failed: {e}")
        return

    try:
        setup_tty_115200_8n1(fd)
        set_dtr_rts(fd, True, True)
        time.sleep(0.05)

        # Try both addresses (docs mention 0x08; vendor sample uses 0x04)
        for addr in (0x08, 0x04):
            ok_payload, raw = do_readmem(fd, addr=addr, length=64, request_id=1, timeout_sec=3.0)
            if ok_payload is None:
                print(f"[{port}] addr=0x{addr:X}: no response (len={len(raw)}). raw={hexdump(raw)}")
                continue

            ok, val = ok_payload
            if not ok:
                print(f"[{port}] addr=0x{addr:X}: {val}. raw={hexdump(raw)}")
                continue

            text = decode_ascii(val)
            print(f"[{port}] addr=0x{addr:X}: '{text}'")
            if "OBSIDIAN" in text.upper():
                print(f"[{port}] >>> looks like CONTROL CHANNEL (found OBSIDIAN)")
                return

        print(f"[{port}] no OBSIDIAN string found")

    finally:
        os.close(fd)


if __name__ == "__main__":
    ports = sys.argv[1:] or ["/dev/ttyACM0", "/dev/ttyACM1"]
    for p in ports:
        probe_port(p)
