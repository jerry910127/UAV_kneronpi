#!/usr/bin/env python3
import os
import sys
import time
import termios

# ---- GenCP constants (big-endian over serial) ----
PREAMBLE = b"\x01\x00"
CHANNEL_ID = 0x0000  # default control channel over serial:contentReference[oaicite:6]{index=6}
READMEM_CMD = 0x0800
READMEM_ACK = 0x0801

def ones_complement_checksum16(data: bytes) -> int:
    """
    GenCP serial prefix checksum uses 16-bit one's complement checksum (RFC768-style):contentReference[oaicite:7]{index=7}
    We'll apply it to the byte ranges specified by the spec for CCD-CRC and SCD-CRC:contentReference[oaicite:8]{index=8}.
    """
    if len(data) % 2 == 1:
        data += b"\x00"
    s = 0
    for i in range(0, len(data), 2):
        s += (data[i] << 8) | data[i + 1]
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF

def build_readmem_packet(addr: int, length: int, request_id: int = 1) -> bytes:
    # CCD layout: flags(2) + command_id(2) + ccd_length(2) + request_id(2):contentReference[oaicite:9]{index=9}
    # flags: set RequestAck bit (bit14) to ask for acknowledge:contentReference[oaicite:10]{index=10}
    flags = 0x4000
    ccd_length = 12  # ReadMem SCD is 8(addr)+2(res)+2(len) = 12:contentReference[oaicite:11]{index=11}

    ccd = (
        flags.to_bytes(2, "big") +
        READMEM_CMD.to_bytes(2, "big") +
        ccd_length.to_bytes(2, "big") +
        request_id.to_bytes(2, "big")
    )

    # SCD for ReadMem: 8-byte address + 2-byte reserved(0) + 2-byte read length:contentReference[oaicite:12]{index=12}
    scd = (
        addr.to_bytes(8, "big") +
        (0).to_bytes(2, "big") +
        length.to_bytes(2, "big")
    )

    channel = CHANNEL_ID.to_bytes(2, "big")

    # Serial Prefix: preamble + CCD-CRC16 + SCD-CRC16 + channel_id:contentReference[oaicite:13]{index=13}
    ccd_crc = ones_complement_checksum16(channel + ccd)  # built from channel_id and CCD:contentReference[oaicite:14]{index=14}
    scd_crc = ones_complement_checksum16(channel + ccd + scd)  # built from channel_id, CCD, SCD:contentReference[oaicite:15]{index=15}

    prefix = (
        PREAMBLE +
        ccd_crc.to_bytes(2, "big") +
        scd_crc.to_bytes(2, "big") +
        channel
    )

    return prefix + ccd + scd

def setup_tty_9600_8n1(fd: int):
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

    # baud 9600 default:contentReference[oaicite:16]{index=16}
    attrs[4] = termios.B115200  # ispeed
    attrs[5] = termios.B115200  # ospeed

    # read timeout: VTIME in 0.1s units, VMIN=0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 20  # 2.0 seconds

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def read_with_timeout(fd: int, total_timeout_sec: float = 2.5) -> bytes:
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < total_timeout_sec:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                buf += chunk
                # heuristic: if we already have enough to parse length, we can break later
            else:
                time.sleep(0.02)
        except BlockingIOError:
            time.sleep(0.02)
    return bytes(buf)

def parse_ack_payload(data: bytes):
    # find preamble
    i = data.find(PREAMBLE)
    if i < 0 or len(data) < i + 16:
        return None

    # prefix = 8 bytes: preamble(2) + ccd_crc(2) + scd_crc(2) + channel_id(2):contentReference[oaicite:17]{index=17}
    p = i
    prefix = data[p:p+8]
    if len(prefix) < 8:
        return None

    # CCD-ACK begins after prefix: status(2)+cmd_id(2)+len(2)+request_id(2):contentReference[oaicite:18]{index=18}
    ccd_ack = data[p+8:p+16]
    if len(ccd_ack) < 8:
        return None

    status = int.from_bytes(ccd_ack[0:2], "big")
    cmd_id = int.from_bytes(ccd_ack[2:4], "big")
    scd_len = int.from_bytes(ccd_ack[4:6], "big")
    req_id = int.from_bytes(ccd_ack[6:8], "big")

    # SCD data follows
    need_total = (p + 16 + scd_len)
    if len(data) < need_total:
        return None

    payload = data[p+16:p+16+scd_len]
    return status, cmd_id, scd_len, req_id, payload

def probe_port(port: str):
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        print(f"[{port}] open failed: {e}")
        return

    try:
        setup_tty_9600_8n1(fd)

        # Miramar doc: read 64 bytes from address 0x08 should return "OBSIDIAN SENSORS INC." on the correct port:contentReference[oaicite:19]{index=19}
        pkt = build_readmem_packet(addr=0x08, length=64, request_id=1)
        os.write(fd, pkt)

        raw = read_with_timeout(fd, total_timeout_sec=2.5)
        parsed = parse_ack_payload(raw)

        if not parsed:
            print(f"[{port}] no valid GenCP ack (timeout or wrong channel).")
            return

        status, cmd_id, scd_len, req_id, payload = parsed
        if cmd_id != READMEM_ACK:
            print(f"[{port}] got ack but cmd_id=0x{cmd_id:04X} (expected 0x{READMEM_ACK:04X}). status=0x{status:04X}")
            return

        # status 0x0000 = success:contentReference[oaicite:20]{index=20}
        text = payload.split(b"\x00", 1)[0].decode("ascii", errors="replace")
        print(f"[{port}] status=0x{status:04X}, len={scd_len}, text='{text}'")

    finally:
        os.close(fd)

if __name__ == "__main__":
    ports = sys.argv[1:] or ["/dev/ttyACM0", "/dev/ttyACM1"]
    for p in ports:
        probe_port(p)
