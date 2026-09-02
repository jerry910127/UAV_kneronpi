#!/usr/bin/env python3
# gencp_probe_official_v3.py
# 115200, little-endian, no serial prefix/CRC (per Obsidian sample),
# plus DTR/RTS assert and robust ACK scanning.

import os
import sys
import time
import argparse
import termios
import fcntl
import struct
import select

# ioctl constants (fallback values for Linux)
TIOCMGET = getattr(termios, "TIOCMGET", 0x5415)
TIOCMSET = getattr(termios, "TIOCMSET", 0x5418)
TIOCM_DTR = getattr(termios, "TIOCM_DTR", 0x002)
TIOCM_RTS = getattr(termios, "TIOCM_RTS", 0x004)

def set_dtr_rts(fd: int, dtr: bool = True, rts: bool = True):
    buf = fcntl.ioctl(fd, TIOCMGET, struct.pack("I", 0))
    (bits,) = struct.unpack("I", buf)
    if dtr:
        bits |= TIOCM_DTR
    else:
        bits &= ~TIOCM_DTR
    if rts:
        bits |= TIOCM_RTS
    else:
        bits &= ~TIOCM_RTS
    fcntl.ioctl(fd, TIOCMSET, struct.pack("I", bits))

def setup_tty_115200_8n1(fd: int):
    attrs = termios.tcgetattr(fd)

    # iflag
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                  termios.ISTRIP | termios.INLCR | termios.IGNCR |
                  termios.ICRNL | termios.IXON | termios.IXOFF | termios.IXANY)

    # oflag
    attrs[1] &= ~termios.OPOST

    # cflag: 8N1, enable receiver, ignore modem ctrl (CLOCAL)
    attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
    attrs[2] |= (termios.CS8 | termios.CREAD | termios.CLOCAL)

    # lflag
    attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                  termios.ISIG | termios.IEXTEN)

    # speed
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200

    # cc: non-blocking read with small timeout
    # VTIME is in deciseconds
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1  # 0.1s

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def hex_dump(b: bytes, max_len=64) -> str:
    b = b[:max_len]
    return " ".join(f"{x:02X}" for x in b)

def looks_like_ascii_log(buf: bytes) -> bool:
    if not buf:
        return False
    sample = buf[:256]
    printable = sum((32 <= c <= 126) or c in (9, 10, 13) for c in sample)
    return printable / max(1, len(sample)) > 0.85

def build_readmem_cmd(address: int, length: int, request_id: int) -> bytes:
    # exactly as Obsidian python sample: 20 bytes total, little-endian
    cmd = bytearray(20)
    # CCD
    cmd[0] = 0x00
    cmd[1] = 0x40  # flags, request ACK (bit 14)
    cmd[2] = 0x00
    cmd[3] = 0x08  # READMEM_CMD (0x0800)
    cmd[4] = 0x0C
    cmd[5] = 0x00  # SCD length = 12
    cmd[6] = request_id & 0xFF
    cmd[7] = (request_id >> 8) & 0xFF
    # SCD
    cmd[8:16] = int(address).to_bytes(8, "little", signed=False)
    cmd[16] = 0
    cmd[17] = 0
    cmd[18] = length & 0xFF
    cmd[19] = (length >> 8) & 0xFF
    return bytes(cmd)

def find_readmem_ack(buf: bytes, request_id: int, payload_len: int):
    # Scan for: status/flags=0x0000, command_id=0x0801 => bytes 00 00 01 08
    sig = b"\x00\x00\x01\x08"
    i = 0
    while True:
        j = buf.find(sig, i)
        if j < 0:
            return None
        # need 8-byte header
        if len(buf) >= j + 8:
            rid = buf[j+6] | (buf[j+7] << 8)
            if rid == (request_id & 0xFFFF):
                need = 8 + payload_len
                if len(buf) >= j + need:
                    return buf[j:j+need], j
        i = j + 1

def read_with_timeout(fd: int, total_timeout_s: float, max_bytes: int = 8192) -> bytes:
    end = time.time() + total_timeout_s
    out = bytearray()
    while time.time() < end and len(out) < max_bytes:
        r, _, _ = select.select([fd], [], [], 0.1)
        if not r:
            continue
        chunk = os.read(fd, 4096)
        if not chunk:
            continue
        out += chunk
        # if stream is extremely chatty, stop early
        if len(out) >= max_bytes:
            break
    return bytes(out)

def probe_one_port(dev: str, verbose: bool = False) -> bool:
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        setup_tty_115200_8n1(fd)

        # Assert DTR/RTS (important for many CDC ACM devices)
        try:
            set_dtr_rts(fd, True, True)
        except Exception as e:
            if verbose:
                print(f"[{dev}] warn: cannot set DTR/RTS ({e})")

        termios.tcflush(fd, termios.TCIOFLUSH)

        # Peek without sending: if it's a log console, you'll see ASCII immediately.
        peek = read_with_timeout(fd, 0.3, max_bytes=512)
        if peek:
            if verbose:
                print(f"[{dev}] peek({len(peek)}): {hex_dump(peek, 64)}")
            if looks_like_ascii_log(peek):
                print(f"[{dev}] looks like ASCII log/console channel (not control).")
                return False

        # Try read manufacturer string using address 0x04 and 0x08 (both seen in docs/samples)
        for addr in (0x04, 0x08):
            request_id = 1 if addr == 0x04 else 2
            payload_len = 64

            cmd = build_readmem_cmd(addr, payload_len, request_id)
            if verbose:
                print(f"[{dev}] TX(20) addr=0x{addr:X}: {hex_dump(cmd, 80)}")

            os.write(fd, cmd)

            buf = read_with_timeout(fd, 1.5, max_bytes=8192)
            if verbose and buf:
                print(f"[{dev}] RX({len(buf)}): {hex_dump(buf, 96)}")

            hit = find_readmem_ack(buf, request_id, payload_len)
            if not hit:
                continue

            pkt, off = hit
            data = pkt[8:8+payload_len]
            try:
                s = data.decode("utf-8", errors="ignore")
            except Exception:
                s = repr(data)

            if "OBSIDIAN" in s.upper():
                print(f"[{dev}] ✅ GenICam control channel found! (addr=0x{addr:X})")
                print(f"[{dev}] Manufacturer snippet: {s.strip()}")
                return True
            else:
                if verbose:
                    print(f"[{dev}] got ACK but no OBSIDIAN string. decoded: {s!r}")

        print(f"[{dev}] no valid READMEM_ACK / no manufacturer string.")
        return False

    finally:
        os.close(fd)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ports", nargs="+", help="e.g. /dev/ttyACM0 /dev/ttyACM1")
    ap.add_argument("-d", "--debug", action="store_true")
    args = ap.parse_args()

    ok_any = False
    for p in args.ports:
        try:
            ok = probe_one_port(p, verbose=args.debug)
            ok_any = ok_any or ok
        except PermissionError:
            print(f"[{p}] permission denied.")
        except FileNotFoundError:
            print(f"[{p}] not found.")
        except Exception as e:
            print(f"[{p}] error: {e}")

    sys.exit(0 if ok_any else 2)

if __name__ == "__main__":
    main()
